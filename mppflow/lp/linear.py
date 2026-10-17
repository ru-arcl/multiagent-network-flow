"""Solver-neutral linear models and solutions.

:class:`LinearModel` holds a (mixed-integer) linear program in exactly the order it was
built: columns are numbered in the order they are appended, rows are kept in the order they
are appended, and the terms of each row keep their insertion order (duplicates included).
The model builders in :mod:`mppflow.model` append columns and rows in the same order as the
original Java code calls ``GRBModel.addVar`` / ``addConstr``, so the Gurobi backend
(:mod:`mppflow.lp.gurobi`) recreates the original's model one call at a time.

:class:`Solution` is what every backend returns.

Storage is compact (``array`` buffers), because the original's large experiments build
models with millions of columns. All accessors return plain Python values.

Text dumps. :meth:`LinearModel.to_canonical_text` writes the **golden canonical model
dump** (the format of ``tools/golden/README.md``, "Canonical model dump"): the model as
Gurobi holds it (row terms sorted by column, duplicates summed, zeros dropped), and
:meth:`LinearModel.sha256` hashes it. With the column names of
:meth:`mppflow.model.MPPModel.column_names` it equals the dumps recorded from the original;
:func:`mppflow.model.canonical_text` and :meth:`mppflow.model.MPPModel.canonical_text` are
thin wrappers around it. :meth:`LinearModel.to_raw_text` is a second, lossless listing in
construction order (term order, duplicates, row names, objective constant), for debugging;
it is not the golden format.

This module is new (the original has no solver-neutral model); it has no
``compat`` switch.
"""

from __future__ import annotations

import hashlib
from array import array
from dataclasses import dataclass, field
from typing import Iterable, Sequence

from .._java import java_double_str

INF = float("inf")

VTYPES = ("C", "B", "I")
"""Column types: continuous, binary, general integer (Gurobi's ``VType`` characters)."""

_SENSE_ALIASES = {
    "<=": "<=", "<": "<=", "L": "<=",
    "==": "==", "=": "==", "E": "==",
    ">=": ">=", ">": ">=", "G": ">=",
}

STATUSES = ("optimal", "feasible", "infeasible", "unbounded", "time_limit", "node_limit")
"""Values of :attr:`Solution.status`.

``optimal``: proven optimal (within the requested ``mip_gap``, if any).
``feasible``: a solution exists but optimality was not proven, for a reason other than a
time or node limit (e.g. Gurobi ``SUBOPTIMAL`` or ``SOLUTION_LIMIT``; an ``INTERRUPTED``
Gurobi solve raises :class:`KeyboardInterrupt`). Like ``time_limit`` and ``node_limit``,
it is no proof of anything about the model.
``infeasible``: proven infeasible (Gurobi ``INF_OR_UNBD`` is also reported as
``infeasible``; see :attr:`Solution.raw_status`).
``unbounded``: the objective is unbounded (the LP relaxation, for a MIP).
``time_limit`` / ``node_limit``: the limit was reached. ``x`` holds the incumbent if one was
found (``sol_count > 0``), otherwise ``x`` is ``None``.
"""


def normalize_sense(sense: str) -> str:
    """Return the canonical row sense ``'<='``, ``'=='`` or ``'>='`` for ``sense``.

    Accepts ``'<='``, ``'<'``, ``'L'``, ``'=='``, ``'='``, ``'E'``, ``'>='``, ``'>'``, ``'G'``.
    """
    try:
        return _SENSE_ALIASES[sense]
    except KeyError:
        raise ValueError(f"unknown row sense {sense!r}") from None


def _normalize_obj_sense(sense: str) -> str:
    s = sense.lower()
    if s in ("max", "maximize"):
        return "max"
    if s in ("min", "minimize"):
        return "min"
    raise ValueError(f"objective sense must be 'max' or 'min', not {sense!r}")


def _num(x: float) -> str:
    """Number formatting for :meth:`LinearModel.to_raw_text`: Java ``Double.toString`` style."""
    return java_double_str(float(x))


def golden_number(x: float) -> str:
    """Number formatting of the golden canonical dump: an integral value below 1e15 in
    magnitude as an integer (``-0.0`` as ``0``), ``>= 1e100`` as ``inf``, ``<= -1e100`` as
    ``-inf``, anything else as Java's ``Double.toString``."""
    x = float(x)
    if x >= 1e100:
        return "inf"
    if x <= -1e100:
        return "-inf"
    if x == int(x) and abs(x) < 1e15:
        return str(int(x))
    return java_double_str(x)


_GOLDEN_SENSE = {"<=": "<=", ">=": ">=", "==": "="}


class LinearModel:
    """A linear or mixed-integer linear program, in construction order.

    Columns have a lower bound, an upper bound (``inf`` allowed), a type (``'C'``, ``'B'``,
    ``'I'``), an objective coefficient and an optional name. Rows are sparse term lists
    ``[(col, coef), ...]`` with a sense (``'<='``, ``'=='``, ``'>='``) and a right-hand side.

    The objective is ``obj_constant + sum_j obj[j] * x_j``, maximized or minimized according to
    :attr:`sense`. :meth:`set_objective` additionally remembers the term list it was given,
    so that the Gurobi backend can call ``setObjective`` with the same expression, as the
    original does (its columns are created with objective coefficient 0).

    Example::

        m = LinearModel("max")
        x = m.add_column(0, 1, "B")
        y = m.add_column(0, 1, "B")
        m.add_row([(x, 1.0), (y, 1.0)], "<=", 1.0)
        m.set_objective([(x, 2.0), (y, 1.0)])
    """

    def __init__(self, sense: str = "max", name: str | None = None):
        self.name = name
        self.sense = _normalize_obj_sense(sense)
        self.obj_constant = 0.0
        self._lb = array("d")
        self._ub = array("d")
        self._obj = array("d")
        self._vtype = bytearray()
        self._col_names: dict[int, str] = {}
        self._row_start = array("q", [0])
        self._term_col = array("q")
        self._term_coef = array("d")
        self._row_sense = bytearray()
        self._rhs = array("d")
        self._row_names: dict[int, str] = {}
        self.objective_terms: list[tuple[int, float]] | None = None
        """The term list given to :meth:`set_objective` (``None`` if it was never called)."""

    # ------------------------------------------------------------------ columns
    def add_column(
        self,
        lb: float = 0.0,
        ub: float = INF,
        vtype: str = "C",
        obj: float = 0.0,
        name: str | None = None,
    ) -> int:
        """Append a column and return its index (0, 1, 2, ... in call order).

        A nonzero ``obj`` given after :meth:`set_objective` is appended to
        :attr:`objective_terms` (like Gurobi's ``addVar(..., obj, ...)`` after
        ``setObjective``), so every backend solves the same objective.
        """
        if vtype not in VTYPES:
            raise ValueError(f"vtype must be one of {VTYPES}, not {vtype!r}")
        j = len(self._lb)
        self._lb.append(float(lb))
        self._ub.append(float(ub))
        self._obj.append(float(obj))
        if self.objective_terms is not None and float(obj) != 0.0:
            self.objective_terms.append((j, float(obj)))
        self._vtype.append(ord(vtype))
        if name is not None:
            self._col_names[j] = name
        return j

    add_var = add_column

    @property
    def num_columns(self) -> int:
        return len(self._lb)

    @property
    def lb(self) -> list[float]:
        return self._lb.tolist()

    @property
    def ub(self) -> list[float]:
        return self._ub.tolist()

    @property
    def obj(self) -> list[float]:
        return self._obj.tolist()

    @property
    def vtypes(self) -> list[str]:
        return [chr(c) for c in self._vtype]

    def column(self, j: int) -> tuple[float, float, str, float, str | None]:
        """``(lb, ub, vtype, obj, name)`` of column ``j``."""
        return (self._lb[j], self._ub[j], chr(self._vtype[j]), self._obj[j],
                self._col_names.get(j))

    def column_name(self, j: int) -> str | None:
        return self._col_names.get(j)

    def set_bounds(self, j: int, lb: float | None = None, ub: float | None = None) -> None:
        """Change the bounds of column ``j`` (``None`` keeps the current value)."""
        if lb is not None:
            self._lb[j] = float(lb)
        if ub is not None:
            self._ub[j] = float(ub)

    @property
    def is_mip(self) -> bool:
        """True if any column is binary or integer."""
        c = ord("C")
        return any(t != c for t in self._vtype)

    # --------------------------------------------------------------------- rows
    def add_row(
        self,
        terms: Iterable[tuple[int, float]],
        sense: str,
        rhs: float,
        name: str | None = None,
    ) -> int:
        """Append the row ``sum(coef * x[col] for col, coef in terms) <sense> rhs``.

        Terms keep their order; repeated columns are kept (and summed by the solvers).
        Returns the row index.
        """
        s = normalize_sense(sense)
        n = len(self._lb)
        tc, tv = self._term_col, self._term_coef
        for col, coef in terms:
            if not 0 <= col < n:
                raise IndexError(f"row term refers to column {col}, model has {n} columns")
            tc.append(col)
            tv.append(float(coef))
        i = len(self._rhs)
        self._row_start.append(len(tc))
        self._row_sense.append(ord(s[0]))
        self._rhs.append(float(rhs))
        if name is not None:
            self._row_names[i] = name
        return i

    add_constr = add_row

    @property
    def num_rows(self) -> int:
        return len(self._rhs)

    @property
    def num_terms(self) -> int:
        return len(self._term_col)

    def row(self, i: int) -> list[tuple[int, float]]:
        """The terms of row ``i`` in insertion order."""
        a, b = self._row_start[i], self._row_start[i + 1]
        return list(zip(self._term_col[a:b].tolist(), self._term_coef[a:b].tolist()))

    def merged_row(self, i: int) -> list[tuple[int, float]]:
        """The terms of row ``i`` as Gurobi stores them: sorted by column, duplicate
        columns summed, zero coefficients dropped."""
        acc: dict[int, float] = {}
        a, b = self._row_start[i], self._row_start[i + 1]
        for c, v in zip(self._term_col[a:b].tolist(), self._term_coef[a:b].tolist()):
            acc[c] = acc.get(c, 0.0) + v
        return [(c, acc[c]) for c in sorted(acc) if acc[c] != 0.0]

    def row_sense(self, i: int) -> str:
        c = chr(self._row_sense[i])
        return "<=" if c == "<" else ("==" if c == "=" else ">=")

    def row_rhs(self, i: int) -> float:
        return self._rhs[i]

    def row_name(self, i: int) -> str | None:
        return self._row_names.get(i)

    def rows(self):
        """Iterate ``(terms, sense, rhs)`` over the rows in order."""
        for i in range(len(self._rhs)):
            yield self.row(i), self.row_sense(i), self._rhs[i]

    def csr(self):
        """``(indptr, indices, data)`` as plain lists (row-major, insertion order)."""
        return self._row_start.tolist(), self._term_col.tolist(), self._term_coef.tolist()

    # ---------------------------------------------------------------- objective
    def set_objective(
        self,
        terms: Iterable[tuple[int, float]],
        sense: str | None = None,
        constant: float = 0.0,
    ) -> None:
        """Replace the objective by ``constant + sum(coef * x[col])``.

        Mirrors Gurobi's ``setObjective(expr, sense)``: every column's objective coefficient
        is reset to 0, then the terms are accumulated in order (a repeated column adds up).
        The term list is kept in :attr:`objective_terms` for the Gurobi backend.
        """
        if sense is not None:
            self.sense = _normalize_obj_sense(sense)
        terms = [(int(c), float(v)) for c, v in terms]
        n = len(self._obj)
        obj = array("d", bytes(8 * n))
        for col, coef in terms:
            if not 0 <= col < n:
                raise IndexError(f"objective term refers to column {col}, model has {n} columns")
            obj[col] += coef
        self._obj = obj
        self.obj_constant = float(constant)
        self.objective_terms = terms

    def objective_value(self, x: Sequence[float]) -> float:
        """``obj_constant + sum_j obj[j] * x[j]``, accumulated in column order."""
        v = self.obj_constant
        obj = self._obj
        for j in range(len(obj)):
            c = obj[j]
            if c != 0.0:
                v += c * x[j]
        return v

    # ------------------------------------------------------------------ copies
    def copy(self) -> "LinearModel":
        m = LinearModel(self.sense, self.name)
        m.obj_constant = self.obj_constant
        m._lb = array("d", self._lb)
        m._ub = array("d", self._ub)
        m._obj = array("d", self._obj)
        m._vtype = bytearray(self._vtype)
        m._col_names = dict(self._col_names)
        m._row_start = array("q", self._row_start)
        m._term_col = array("q", self._term_col)
        m._term_coef = array("d", self._term_coef)
        m._row_sense = bytearray(self._row_sense)
        m._rhs = array("d", self._rhs)
        m._row_names = dict(self._row_names)
        m.objective_terms = None if self.objective_terms is None else list(self.objective_terms)
        return m

    def relax(self) -> "LinearModel":
        """The LP relaxation, as a new model (like Gurobi's ``GRBModel.relax()``).

        Every column becomes continuous. Binary columns keep their bounds intersected with
        ``[0, 1]``; integer columns keep their bounds.
        """
        m = self.copy()
        lb, ub, vt = m._lb, m._ub, m._vtype
        b, c = ord("B"), ord("C")
        for j in range(len(vt)):
            if vt[j] == b:
                if lb[j] < 0.0:
                    lb[j] = 0.0
                if ub[j] > 1.0:
                    ub[j] = 1.0
            vt[j] = c
        return m

    # ------------------------------------------------------------------- dumps
    def to_canonical_text(self, names: bool | Sequence[str] = True) -> str:
        """The golden canonical model dump (format: ``tools/golden/README.md``)::

            SENSE max|min
            [OBJCONST <c>]                       only when the constant is not 0
            NCOLS <n>
            NROWS <m>
            C<j> <lb> <ub> <B|C|I> <obj>[ <name>]
            R<i> <=|=|>= <rhs> :[ (<col>,<coef>)]...

        Row terms are those of :meth:`merged_row` (sorted, summed, zeros dropped); numbers
        are formatted by :func:`golden_number`; row names are not printed. ``names``:
        ``True`` prints each column's name (``C<j>`` for an unnamed column), a sequence
        prints ``names[j]`` (e.g. :meth:`mppflow.model.MPPModel.column_names`), ``False``
        leaves the name field out. The MPP models have no objective constant, so their dumps
        are exactly the recorded ones; ``sha256`` in the golden records is
        :meth:`sha256` of this text.
        """
        out = [f"SENSE {self.sense}"]
        if self.obj_constant != 0.0:
            out.append(f"OBJCONST {golden_number(self.obj_constant)}")
        n = len(self._lb)
        out += [f"NCOLS {n}", f"NROWS {len(self._rhs)}"]
        lb, ub, obj, vt = self._lb, self._ub, self._obj, self._vtype
        fmt = golden_number
        for j in range(n):
            line = f"C{j} {fmt(lb[j])} {fmt(ub[j])} {chr(vt[j])} {fmt(obj[j])}"
            if names is True:
                line += " " + self._col_names.get(j, f"C{j}")
            elif names is not False and names is not None:
                line += f" {names[j]}"
            out.append(line)
        for i in range(len(self._rhs)):
            line = f"R{i} {_GOLDEN_SENSE[self.row_sense(i)]} {fmt(self._rhs[i])} :"
            terms = " ".join(f"({c},{fmt(v)})" for c, v in self.merged_row(i))
            if terms:
                line += " " + terms
            out.append(line)
        return "\n".join(out) + "\n"

    def sha256(self, names: bool | Sequence[str] = True) -> str:
        """SHA-256 (hex) of :meth:`to_canonical_text` (``names`` as there), UTF-8 encoded."""
        return hashlib.sha256(self.to_canonical_text(names).encode("utf-8")).hexdigest()

    def to_raw_text(self, names: bool = True) -> str:
        """A lossless listing of the model in construction order (not the golden format;
        see :meth:`to_canonical_text`).

        Format (one item per line, numbers in Java ``Double.toString`` form)::

            LINEARMODEL 1
            SENSE max|min
            OBJCONST <c>
            COLUMNS <n>
            V <j> <lb> <ub> <C|B|I> <obj>[ <name>]
            ROWS <m>
            R <i> <=|==|>= <rhs> :[ <coef>*<col>]...[ ; <name>]

        Row terms are listed in insertion order (duplicates kept). With ``names=False``,
        column and row names are omitted (two models that differ only in names then give
        the same text).
        """
        out = ["LINEARMODEL 1", f"SENSE {self.sense}", f"OBJCONST {_num(self.obj_constant)}",
               f"COLUMNS {len(self._lb)}"]
        lb, ub, obj, vt = self._lb, self._ub, self._obj, self._vtype
        for j in range(len(lb)):
            line = f"V {j} {_num(lb[j])} {_num(ub[j])} {chr(vt[j])} {_num(obj[j])}"
            if names and j in self._col_names:
                line += " " + self._col_names[j]
            out.append(line)
        out.append(f"ROWS {len(self._rhs)}")
        rs, tc, tv = self._row_start, self._term_col, self._term_coef
        for i in range(len(self._rhs)):
            parts = [f"R {i} {self.row_sense(i)} {_num(self._rhs[i])} :"]
            for k in range(rs[i], rs[i + 1]):
                parts.append(f"{_num(tv[k])}*{tc[k]}")
            line = " ".join(parts)
            if names and i in self._row_names:
                line += " ; " + self._row_names[i]
            out.append(line)
        return "\n".join(out) + "\n"

    def __repr__(self) -> str:
        return (f"LinearModel(sense={self.sense!r}, columns={self.num_columns}, "
                f"rows={self.num_rows}, terms={self.num_terms}, mip={self.is_mip})")


@dataclass
class Solution:
    """The result of solving a :class:`LinearModel`.

    Attributes:
        status: one of :data:`STATUSES`.
        objective: objective value of ``x`` (``None`` when there is no solution).
        x: column values in column order, exactly as the backend reports them (``None``
            when there is no solution).
        mip_gap: relative gap ``|bound - objective| / |objective|`` as the backend reports it
            (Gurobi ``MIPGap``; 0 for a proven optimum; ``inf`` when undefined).
        bound: best objective bound (Gurobi ``ObjBound``; for an LP, the objective).
        sol_count: number of solutions found (Gurobi ``SolCount``; 0 or 1 for the others).
        backend: ``'highs'``, ``'gurobi'`` or ``'builtin'``.
        raw_status: the backend's own status (Gurobi status code, HiGHS message, ...).
        runtime: wall-clock seconds spent in the backend.
        node_count: branch-and-bound nodes explored, if reported.
        iterations: simplex iterations, if reported.
    """

    status: str
    objective: float | None = None
    x: list[float] | None = None
    mip_gap: float | None = None
    bound: float | None = None
    sol_count: int = 0
    backend: str = ""
    raw_status: object = None
    runtime: float = 0.0
    node_count: int | None = None
    iterations: int | None = None
    extra: dict = field(default_factory=dict)

    @property
    def has_solution(self) -> bool:
        """True if ``x`` holds a solution (optimal, or an incumbent at a limit)."""
        return self.x is not None

    @property
    def is_optimal(self) -> bool:
        return self.status == "optimal"


def relative_gap(objective: float, bound: float) -> float:
    """Gurobi's MIP gap definition: ``|bound - objective| / |objective|``.

    0 when both are 0, ``inf`` when only the objective is 0.
    """
    d = abs(bound - objective)
    if objective == 0.0:
        return 0.0 if d == 0.0 else INF
    return d / abs(objective)
