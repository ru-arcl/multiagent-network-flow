"""Gurobi backend (optional; requires ``gurobipy`` and a license).

The Gurobi model is built exactly as the original Java code builds it:

1. A Gurobi environment is created empty, its parameters are set (``OutputFlag`` 0 unless
   given, then ``params``, then ``TimeLimit`` / ``MIPGap`` from the arguments), and only then
   is it started and the model created from it. The original also sets its parameters on
   the environment before ``new GRBModel(env)``.
2. One ``addVar(lb, ub, 0.0, vtype, name)`` per column, in column order (objective
   coefficient 0, as in the original; unnamed columns get Gurobi's default names).
3. One ``addLConstr(LinExpr(coefs, vars), sense, rhs)`` per row, in row order, with the
   terms in insertion order (repeated terms are passed as they are).
4. ``setObjective(expr, sense)``, where ``expr`` is the term list given to
   :meth:`LinearModel.set_objective` (extended by later ``add_column(obj=...)`` calls; the
   nonzero objective coefficients in column order when there is no such list or it no
   longer matches), plus the objective constant.

With the same Gurobi version and ``Threads`` setting, the solution is therefore the one the
original gets. ``x`` is returned exactly as Gurobi reports it (``X`` attribute), because the
original tests ``X == 1.0`` exactly.

Status mapping: ``OPTIMAL`` -> ``optimal``; ``INFEASIBLE`` and ``INF_OR_UNBD`` ->
``infeasible``; ``UNBOUNDED`` -> ``unbounded``; ``TIME_LIMIT``, ``WORK_LIMIT`` and
``ITERATION_LIMIT`` -> ``time_limit``; ``NODE_LIMIT`` -> ``node_limit``; ``INTERRUPTED``
(Ctrl-C during ``optimize()``, which gurobipy turns into a status instead of an exception)
raises :class:`KeyboardInterrupt`; any other status (``SOLUTION_LIMIT``, ``SUBOPTIMAL``,
``USER_OBJ_LIMIT``, ...) -> ``feasible`` if ``SolCount > 0``, else :class:`RuntimeError` is
raised (``NUMERIC``, ``MEM_LIMIT``, ``CUTOFF``, ...: a solver stop without a solution is not a
proof of infeasibility, as with the HiGHS backend). ``raw_status`` is the Gurobi code.
An ``OPTIMAL`` result with an infinite objective (an unbounded MIP solved with
``DualReductions=0``) is reported as ``unbounded``. An ``unbounded`` result carries no
solution (``x``, ``objective``, ``bound`` and ``mip_gap`` are ``None``, ``sol_count`` 0), as
with the other backends, although Gurobi reports a point with ``X = 1e30`` and ``SolCount 1``.

The objective is the term list of :meth:`LinearModel.set_objective` when it still matches the
model's objective coefficients, otherwise the nonzero coefficients in column order (see
:func:`_objective_terms`).

No time limit is set when ``time_limit`` is ``None``; the original's ``TimeLimit = 1e10``
convention belongs to the callers (``mppflow.solvers``). A ``time_limit`` covers building the
Gurobi model, too: ``TimeLimit`` is lowered by the build time before ``optimize()``.

This backend is new as a module (the original calls Gurobi inline); it has no ``compat``
switch.
"""

from __future__ import annotations

import math
import time

from .linear import LinearModel, Solution

_MISSING = (
    "The 'gurobi' backend needs gurobipy and a Gurobi license "
    "(pip install gurobipy; see https://www.gurobi.com). "
    "Use backend='highs' (default) or 'builtin' instead."
)


def _import_gurobipy():
    try:
        import gurobipy  # noqa: F401
    except ImportError as exc:  # pragma: no cover - depends on the environment
        raise ImportError(_MISSING) from exc
    return gurobipy


def available() -> bool:
    """True if gurobipy can be imported (a license is not checked)."""
    try:
        _import_gurobipy()
    except ImportError:
        return False
    return True


def solve(
    model: LinearModel,
    time_limit: float | None = None,
    mip_gap: float | None = None,
    params: dict | None = None,
    relax: bool = False,
) -> Solution:
    """Solve ``model`` with Gurobi, building the model in the original's call order.

    Args:
        model: the model.
        time_limit: seconds, set as ``TimeLimit`` (``None``: not set).
        mip_gap: set as ``MIPGap`` (``None``: not set).
        params: Gurobi parameters, e.g. ``{"Threads": 1}``. ``OutputFlag`` defaults to 0.
        relax: solve :meth:`LinearModel.relax` instead.
    """
    gp = _import_gurobipy()
    GRB = gp.GRB
    if relax:
        model = model.relax()
    t0 = time.perf_counter()
    p = {"OutputFlag": 0}
    if params:
        p.update(params)
    if time_limit is not None:
        p["TimeLimit"] = float(time_limit)
    if mip_gap is not None:
        p["MIPGap"] = float(mip_gap)
    env = gp.Env(empty=True)
    try:
        for k, v in p.items():
            env.setParam(k, v)
        env.start()
        gm = gp.Model(env=env)
        try:
            return _build_and_solve(gp, GRB, gm, model, t0, p.get("TimeLimit") if time_limit is not None else None)
        finally:
            gm.dispose()
    finally:
        env.dispose()


def _status_name(GRB, code) -> str:
    """The name of a Gurobi status code (``GRB.Status``), or ``"?"``."""
    for k in dir(GRB.Status):
        if not k.startswith("_") and getattr(GRB.Status, k) == code:
            return k
    return "?"


def _attr(gp, gm, name):
    """A model attribute, or ``None`` when Gurobi cannot provide it."""
    try:
        return getattr(gm, name)
    except (gp.GurobiError, AttributeError):
        return None


def _objective_terms(model: LinearModel) -> list[tuple[int, float]]:
    """The objective as Gurobi gets it.

    The term list of :meth:`LinearModel.set_objective` is used (the original's ``LinExpr``
    order) when it still adds up, column by column in the same order, to ``model.obj``.
    Otherwise (``set_objective`` never called, or the coefficients changed afterwards) the
    nonzero coefficients of ``model.obj`` are used in column order, so that this backend
    always solves the objective the builtin and HiGHS backends solve.
    """
    obj = model.obj
    n = len(obj)
    terms = model.objective_terms
    if terms is not None:
        acc = [0.0] * n
        for j, c in terms:
            acc[j] += c
        if acc == obj:
            return list(terms)
    return [(j, obj[j]) for j in range(n) if obj[j] != 0.0]


def _build_and_solve(gp, GRB, gm, model: LinearModel, t0: float, time_limit: float | None = None) -> Solution:
    vtmap = {"C": GRB.CONTINUOUS, "B": GRB.BINARY, "I": GRB.INTEGER}
    n = model.num_columns
    lb, ub, vt = model.lb, model.ub, model.vtypes
    gvars = []
    for j in range(n):
        name = model.column_name(j)
        if name is None:
            v = gm.addVar(lb=lb[j], ub=ub[j], obj=0.0, vtype=vtmap[vt[j]])
        else:
            v = gm.addVar(lb=lb[j], ub=ub[j], obj=0.0, vtype=vtmap[vt[j]], name=name)
        gvars.append(v)
    smap = {"<=": GRB.LESS_EQUAL, "==": GRB.EQUAL, ">=": GRB.GREATER_EQUAL}
    indptr, indices, data = model.csr()
    for i in range(model.num_rows):
        a, b = indptr[i], indptr[i + 1]
        expr = gp.LinExpr(data[a:b], [gvars[k] for k in indices[a:b]])
        name = model.row_name(i)
        if name is None:
            gm.addLConstr(expr, smap[model.row_sense(i)], model.row_rhs(i))
        else:
            gm.addLConstr(expr, smap[model.row_sense(i)], model.row_rhs(i), name)
    terms = _objective_terms(model)
    oexpr = gp.LinExpr([c for _, c in terms], [gvars[j] for j, _ in terms])
    if model.obj_constant != 0.0:
        oexpr.addConstant(model.obj_constant)
    gm.setObjective(oexpr, GRB.MAXIMIZE if model.sense == "max" else GRB.MINIMIZE)
    if time_limit is not None:
        # building the Gurobi model (seconds for millions of columns) counts against the limit
        gm.Params.TimeLimit = max(0.0, float(time_limit) - (time.perf_counter() - t0))
    gm.optimize()
    runtime = time.perf_counter() - t0

    code = gm.Status
    sol_count = gm.SolCount
    if code == GRB.OPTIMAL:
        status = "optimal"
    elif code in (GRB.INFEASIBLE, GRB.INF_OR_UNBD):
        status = "infeasible"
    elif code == GRB.UNBOUNDED:
        status = "unbounded"
    elif code in (GRB.TIME_LIMIT, GRB.WORK_LIMIT, GRB.ITERATION_LIMIT):
        status = "time_limit"
    elif code == GRB.NODE_LIMIT:
        status = "node_limit"
    elif code == GRB.INTERRUPTED:
        # gurobipy catches SIGINT and ends optimize() with this status; an interrupted solve
        # decides nothing, so pass the interrupt on instead of reporting a result
        raise KeyboardInterrupt("Gurobi optimization interrupted")
    elif sol_count > 0:
        status = "feasible"
    else:
        raise RuntimeError(f"Gurobi stopped with status {_status_name(GRB, code)} ({code}) and no solution; "
                           "this is not a proof of infeasibility")
    x = objective = bound = gap = None
    if sol_count > 0:
        x = gm.getAttr("X", gvars) if n > 0 else []
        objective = gm.ObjVal
    is_mip = gm.IsMIP == 1
    if is_mip:
        bound = _attr(gp, gm, "ObjBound")
        if sol_count > 0:
            gap = _attr(gp, gm, "MIPGap")
    elif objective is not None:
        bound = objective
        gap = 0.0
    if status == "optimal" and objective is not None and not math.isfinite(objective):
        # with DualReductions=0 Gurobi can report an unbounded MIP as OPTIMAL with an
        # infinite objective
        status = "unbounded"
    if status == "unbounded":
        # Gurobi's "solution" of an unbounded model is a ray-like point (X = 1e30, ...),
        # not a solution
        x = objective = bound = gap = None
        sol_count = 0
    node_count = None
    if is_mip:
        nc = _attr(gp, gm, "NodeCount")
        node_count = None if nc is None else int(nc)
    it = _attr(gp, gm, "IterCount")
    iterations = None if it is None else int(it)
    return Solution(status=status, objective=objective, x=x, mip_gap=gap, bound=bound,
                    sol_count=sol_count, backend="gurobi", raw_status=code,
                    runtime=runtime, node_count=node_count, iterations=iterations,
                    extra={"gurobi_version": gp.gurobi.version()})
