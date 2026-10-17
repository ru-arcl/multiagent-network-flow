"""ILP model builders: the original's ``prepareModel`` methods, column for column and row
for row, plus new models (max distance, and the paper's uncompressed full network).

Every builder returns an :class:`MPPModel`: a :class:`~mppflow.lp.linear.LinearModel` plus
the maps from columns to their meaning (arc ``(a, u, v, t)``, loopback, ``y`` variables)
and the original's ``edgeVarMap``.

Ports (original ``ILP/*.java``):

=====================================  ============================================
builder                                original
=====================================  ============================================
:func:`build_time_model`               ``MultiagentGraphSolverGurobiTime.prepareModel``
:func:`build_time_r_model`             ``MultiagentGraphSolverGurobiTimeR.prepareModel``
:func:`build_time_puzzle_model`        ``MultiagentGraphSolverGurobiTimePuzzle.prepareModel``
:func:`build_total_time_model`         ``MultiagentGraphSolverGurobiTotalTime.prepareModel``
:func:`build_distance_model`           ``MultiagentGraphSolverGurobiDistance.prepareModel``
:func:`build_max_distance_model`       *new* (T-RO 2016 eqs. (11)-(13); not in the original)
:func:`build_full_network_model`       *new* (eqs. (8)-(10) on the Fig. 5 network)
key helpers (:func:`edge_id`, ...)     ``getEdgeId``, ``getVertexTimeId``,
                                       ``getAgentVertexTimeId``, ``getEdgeTimeId``,
                                       TimeR's ``getTimeFromEdgeId``, ``isV1V2Same``
=====================================  ============================================

Column order (all compact models)
---------------------------------
1. Arc columns ``x[a,u,v,t]`` in :func:`~mppflow.network.compact_arcs` order (agent-major,
   frontier ``HashSet`` order, neighbour order), binary (continuous when
   ``integer=False``), bounds ``[0, 1]``.
2. One loopback ``x[a,g,s,T]`` per agent (lower bound 1 when the model fixes it, eq. (11)).
3. Total time only: per agent ``yT[a]`` (integer ``[0, T]``) then ``y[a,T-1], ..., y[a,0]``.
4. Max distance only: ``x_max`` (integer ``[0, T]``).

Row order
---------
TimeR first adds ``T + 1`` one-move-per-step rows (after the arc columns, before the
loopbacks). TotalTime first adds its ``y`` chain (eq. (14)). Then every model adds
(a) vertex capacity ``sum(in(v, t+1)) <= 1`` for ``t = 0..T-1``, ``v = 0..n-1``, only rows
with >= 2 terms; (b) flow conservation ``-sum(in) + sum(out) = 0`` for each agent, ``t = 0..T``,
``v = 0..n-1`` (every non-empty group); (c) head-on ``sum <= 1`` over both directions of an
edge at a step, all agents, >= 2 terms, in the ``TreeMap`` order of the **int32-wrapped**
key ``getEdgeTimeId``. Max distance adds one row per agent at the end.

compat modes
------------
``compat="original"`` keys every map by the original's ``int``/``long`` keys, with their
32-bit wrap-around, so key collisions merge rows or overwrite ``edgeVarMap`` entries
exactly as in Java. ``compat="fixed"`` (default) keys the maps by tuples and orders the
head-on rows by ``(wrapped key, tuple)``; without collisions (every model of the paper's
experiments) the two modes build identical models. Differences of ``fixed``:

* key collisions (``getEdgeTimeId`` for large graphs with long horizons, ``getEdgeId``,
  and ``v*10000 + t`` once ``T >= 10000``) are detected (:attr:`MPPModel.key_collisions`)
  and the colliding groups are kept separate;
* :func:`build_time_r_model`: the one-move-per-step rows contain every move arc (the
  original decodes the step from the wrapped ``getEdgeId`` and therefore leaves out every
  arc whose tail vertex id is >= 22);
* :func:`build_max_distance_model`: new; ``original`` raises
  :class:`~mppflow.errors.NotInOriginal`;
* the default ``reach`` (:func:`~mppflow.network.update_reachable_sets`): on one-way
  graphs ``fixed`` prunes with ``dist(v, g_a)`` (reversed-arc BFS), ``original`` with the
  forward ``dist(g_a, v)``, so the column sets differ there (and the original's models can
  be infeasible at feasible horizons). Symmetric graphs give identical columns.

Canonical text
--------------
:func:`canonical_text` (and :meth:`MPPModel.canonical_text`) write the line format of the
golden model dumps (``tools/golden/README.md``): ``SENSE``, ``NCOLS``, ``NROWS``, one
``C<i> lb ub type obj name`` line per column, one ``R<i> sense rhs : (col,coef) ...`` line
per row with terms sorted by column, duplicates summed and zeros dropped. In ``original``
mode with colliding ``getEdgeId`` keys the column names follow the dump writer's recovery
from ``edgeVarMap`` (``x[..]|x[..]`` for shared keys, ``?<j>`` for overwritten entries; see
:meth:`MPPModel.column_names`), so the dumps match the golden ones in that case too.

Path extraction
---------------
:meth:`MPPModel.retrieve_paths` in ``original`` mode tests ``X == 1.0`` exactly, as the
original does; this needs a solver that returns exact binaries (Gurobi). With HiGHS the test
can fail on an optimal solution, and :class:`InexactSolutionWarning` is issued.


The golden format is produced by :meth:`LinearModel.to_canonical_text
<mppflow.lp.linear.LinearModel.to_canonical_text>`; :func:`canonical_text` here is an alias
that defaults the names to the model's own (``canonical_text(lm, names)`` ==
``lm.to_canonical_text(names)``), and :meth:`MPPModel.canonical_text` passes
:meth:`MPPModel.column_names`. (``LinearModel.to_raw_text`` is a different, construction-order
debugging listing.)
"""
from __future__ import annotations

import hashlib
import warnings
from dataclasses import dataclass
from typing import Sequence

from ._java import i32, i64
from .errors import InexactSolutionWarning, NotInOriginal
from .graph import Graph, as_ids, check_compat
from .lp.linear import LinearModel, golden_number
from .network import FullNetwork, Reachability, compact_arcs, update_reachable_sets

__all__ = [
    "MPPModel",
    "FullNetworkModel",
    "edge_id",
    "vertex_time_id",
    "agent_vertex_time_id",
    "edge_time_id",
    "time_from_edge_id",
    "is_v1v2_same",
    "key_helpers",
    "build_time_model",
    "build_time_r_model",
    "build_time_puzzle_model",
    "build_total_time_model",
    "build_distance_model",
    "build_max_distance_model",
    "build_full_network_model",
    "canonical_text",
    "model_summary",
    "TIME_LIMIT_NONE",
    "InexactSolutionWarning",
    "BuildStopped",
]


class BuildStopped(Exception):
    """Raised by a model builder when its ``stop`` callback returns ``True`` (the solvers
    pass one that checks their time limit, so that building a large model counts against
    the budget)."""


_STOP_EVERY = 4096
"""The builders call ``stop()`` once per this many columns / rows."""


def _stop_check(stop):
    """A per-item check for ``stop`` (``None``: a no-op)."""
    if stop is None:
        return None
    cnt = [0]

    def tick():
        cnt[0] += 1
        if cnt[0] >= _STOP_EVERY:
            cnt[0] = 0
            if stop():
                raise BuildStopped("model building stopped (time limit)")
    return tick

# InexactSolutionWarning is defined in mppflow.errors and re-exported here.

TIME_LIMIT_NONE = 1e10
"""The ``TimeLimit`` the original sets when there is no limit (``10000000000.0``)."""

DEFAULT_MIP_GAP = 1e-4
"""Gurobi's default ``MIPGap`` (the makespan models never set it)."""


# ------------------------------------------------------------------ key helpers
def _jdiv(a: int, b: int) -> int:
    """Java ``long`` division (truncates toward zero)."""
    q = abs(a) // abs(b)
    return q if (a >= 0) == (b > 0) else -q


def _jrem(a: int, b: int) -> int:
    """Java ``long`` remainder (sign of the dividend)."""
    return a - b * _jdiv(a, b)


def edge_id(a: int, v1: int, v2: int, t: int) -> int:
    """``getEdgeId(a, v1, v2, t)`` = ``a*1000000000000L + v1*100000000 + v2*10000 + t``
    with the two ``int`` products wrapped to 32 bits (``v1*10^8`` wraps for ``v1 >= 22``)."""
    return i64(a * 1000000000000 + i32(v1 * 100000000) + i32(v2 * 10000) + t)


def vertex_time_id(v: int, t: int) -> int:
    """``getVertexTimeId(v, t)`` = ``v*10000 + t`` in ``int`` arithmetic."""
    return i32(v * 10000 + t)


def agent_vertex_time_id(a: int, v: int, t: int) -> int:
    """``getAgentVertexTimeId(a, v, t)`` = ``a*1000000000000L + v*10000 + t``."""
    return i64(a * 1000000000000 + i32(v * 10000) + t)


def edge_time_id(v1: int, v2: int, t: int) -> int:
    """``getEdgeTimeId(v1, v2, t)``: ``min*1000000000 + max*10000 + t`` in ``int``
    arithmetic (wraps for ``min >= 3``)."""
    if v1 > v2:
        v1, v2 = v2, v1
    return i32(v1 * 1000000000 + v2 * 10000 + t)


def time_from_edge_id(eid: int) -> int:
    """TimeR's ``getTimeFromEdgeId``: ``(int)(edgeId % 10000)`` (Java remainder)."""
    return i32(_jrem(eid, 10000))


def is_v1v2_same(eid: int) -> bool:
    """TimeR's ``isV1V2Same``: ``(id % 10^8) / 10^4 == (id % 10^12) / 10^8`` (Java ``long``
    arithmetic; wrong for wrapped ids, as in the original)."""
    return _jdiv(_jrem(eid, 100000000), 10000) == _jdiv(_jrem(eid, 1000000000000), 100000000)


def key_helpers(a: int, v1: int, v2: int, t: int, time_r: bool = False) -> dict:
    """All key helpers for one tuple, in the golden ``keyHelpers`` record format."""
    e = edge_id(a, v1, v2, t)
    out = {"edgeId": e, "vertexTimeId": vertex_time_id(v1, t),
           "agentVertexTimeId": agent_vertex_time_id(a, v1, t), "edgeTimeId": edge_time_id(v1, v2, t)}
    if time_r:
        out["timeFromEdgeId"] = time_from_edge_id(e)
        out["isV1V2Same"] = is_v1v2_same(e)
    return out


# ------------------------------------------------------------------- the model
@dataclass
class MPPModel:
    """A compact time-expanded ILP and the meaning of its columns.

    Attributes:
        model: the :class:`LinearModel`.
        kind: ``"time"``, ``"time_r"``, ``"time_puzzle"``, ``"total_time"``,
            ``"distance"`` or ``"max_distance"``.
        T: the horizon.
        starts, goals: the instance.
        compat: the mode the model was built in.
        arcs: ``arcs[j] = (a, u, v, t)`` for the arc columns ``j = 0..len(arcs)-1``.
        loopbacks: the loopback column of each agent.
        y_total, y: TotalTime's ``yT[a]`` column and ``y[a][t]`` columns (``y[a]`` is a list
            indexed by ``t``); ``None`` for the other kinds.
        x_max: the max-distance column (``max_distance`` only).
        edge_var_map: the original's ``edgeVarMap``: key -> column, keyed by
            :func:`edge_id` (``original``) or by ``(a, u, v, t)`` (``fixed``); loopbacks are
            stored under ``(a, goal, start, T)``.
        key_collisions: number of colliding keys per map family
            (``edgeVar``, ``vertexTime``, ``agentVertexTime``, ``edgeTime``); merged in
            ``original``, kept apart in ``fixed``.
        params: the Gurobi parameters the original sets for this model
            (``TimeLimit``, ``MIPGap``, ``OutputFlag``).
        reach: the reachability used.
        graph: the graph the model was built on (used by :meth:`column_names` to recover
            the golden dump names when the original's keys collide).
    """

    model: LinearModel
    kind: str
    T: int
    starts: list[int]
    goals: list[int]
    compat: str
    arcs: list[tuple[int, int, int, int]]
    loopbacks: list[int]
    edge_var_map: dict
    key_collisions: dict
    params: dict
    reach: Reachability | None = None
    y_total: list[int] | None = None
    y: list[list[int]] | None = None
    x_max: int | None = None
    graph: Graph | None = None

    @property
    def n_agents(self) -> int:
        return len(self.starts)

    def edge_var(self, a: int, u: int, v: int, t: int) -> int | None:
        """The column of ``edgeVarMap.get(getEdgeId(a, u, v, t))`` (``None`` if absent)."""
        if self.compat == "original":
            return self.edge_var_map.get(edge_id(a, u, v, t))
        return self.edge_var_map.get((a, u, v, t))

    def column_names(self) -> list[str]:
        """Semantic column names, as in the golden dumps: ``x[a,u,v,t]``, loopbacks
        ``x[a,g,s,T]``, ``yT[a]``, ``y[a,t]``, ``x_max``.

        In ``original`` mode, when the original's ``getEdgeId`` keys are not injective on
        the model's domain (vertex ids >= 22 or ``T >= 10000``), the names are recovered
        from ``edgeVarMap`` exactly as the golden dump writer (``tools/golden/Harness.java``,
        ``dumpModel``) does: every tuple ``(a, u, v, t)`` (``t < T``, ``u < n``, ``v`` in
        ``adj[u]``) and loopback whose key is in the map contributes its name to that key,
        names sharing a key or a column are joined with ``|`` (e.g.
        ``x[0,0,1,0]|x[0,0,0,10000]``), and a column no key points to any more (its entry
        was overwritten) reads ``?<j>``. Without collisions both ways give the same names.
        """
        if self.compat == "original" and self.graph is not None and not self._edge_keys_injective():
            return self._golden_names()
        names = [f"x[{a},{u},{v},{t}]" for (a, u, v, t) in self.arcs]
        names += [""] * (self.model.num_columns - len(names))
        for a, j in enumerate(self.loopbacks):
            names[j] = f"x[{a},{self.goals[a]},{self.starts[a]},{self.T}]"
        if self.y_total is not None:
            for a, j in enumerate(self.y_total):
                names[j] = f"yT[{a}]"
            for a, ya in enumerate(self.y):
                for t, j in enumerate(ya):
                    names[j] = f"y[{a},{t}]"
        if self.x_max is not None:
            names[self.x_max] = "x_max"
        return names

    def _edge_keys_injective(self) -> bool:
        """``getEdgeId`` is injective (no int wrap, no carry between its decimal fields)
        when every vertex id is in ``0..21`` and every time index (``<= T``) is below 10000."""
        if self.T >= 10000:
            return False
        g = self.graph
        for v in list(g.vertices) + list(g.adj):
            if not 0 <= v < 22:
                return False
        for nb in g.adj.values():
            for w in nb:
                if not 0 <= w < 22:
                    return False
        return True

    def _golden_names(self) -> list[str]:
        """The names ``Harness.dumpModel`` writes (``original`` mode; see :meth:`column_names`)."""
        import numpy as np

        g, T = self.graph, self.T
        evm = self.edge_var_map
        keys_sorted = np.array(sorted(evm), dtype=np.int64)
        pairs = []
        for v1 in range(g.n):
            nb = g.neighbors(v1)
            if nb is None:
                continue
            for v2 in nb:
                pairs.append((v1, v2))
        E = len(pairs)
        base = np.array([i32(v1 * 100000000) + i32(v2 * 10000) for v1, v2 in pairs], dtype=np.int64)
        key_name: dict[int, str] = {}

        def put(k: int, nm: str) -> None:
            old = key_name.get(k)
            key_name[k] = nm if old is None else old + "|" + nm

        step = max(1, 2_000_000 // max(E, 1))
        for a in range(len(self.starts)):
            if E:
                for t0 in range(0, T, step):
                    ts = np.arange(t0, min(T, t0 + step), dtype=np.int64)
                    ks = (a * 1000000000000 + base[None, :] + ts[:, None]).ravel()
                    hit = np.flatnonzero(np.isin(ks, keys_sorted))
                    for i in hit.tolist():
                        t, (v1, v2) = t0 + i // E, pairs[i % E]
                        put(int(ks[i]), f"x[{a},{v1},{v2},{t}]")
            ga, sa = self.goals[a], self.starts[a]
            put(edge_id(a, ga, sa, T), f"x[{a},{ga},{sa},{T}]")
        col_name: dict[int, str] = {}
        for k in sorted(evm):
            nm = key_name.get(k)
            if nm is None:
                nm = f"?key{k}"
            j = evm[k]
            old = col_name.get(j)
            col_name[j] = nm if old is None else old + "|" + nm
        if self.y_total is not None:
            for a, j in enumerate(self.y_total):
                col_name[j] = f"yT[{a}]"
            for a, ya in enumerate(self.y):
                for t, j in enumerate(ya):
                    col_name[j] = f"y[{a},{t}]"
        if self.x_max is not None:
            col_name[self.x_max] = "x_max"
        return [col_name.get(j, f"?{j}") for j in range(self.model.num_columns)]

    def canonical_text(self) -> str:
        """The golden model-dump text (see :func:`canonical_text`)."""
        return canonical_text(self.model, self.column_names())

    def sha256(self) -> str:
        return hashlib.sha256(self.canonical_text().encode("utf-8")).hexdigest()

    def summary(self, with_dump: bool = True) -> dict:
        """The golden ``prepareModel`` record fields (see :func:`model_summary`)."""
        return model_summary(self, with_dump)

    def retrieve_paths(self, x: Sequence[float], graph: Graph) -> list[list[int]]:
        """``retrievePaths``: follow each agent from its start, taking at each step the
        first neighbour (graph iteration order, self included) whose arc variable is set.

        ``original``: "set" means ``X == 1.0`` exactly, and when no arc is set the vertex
        stays 0 (the Java ``int`` default) and the walk continues from vertex 0. This is
        faithful only for solvers that return exact binaries, as Gurobi (the original's
        solver) normally does: with ``backend="highs"`` values such as
        ``0.9999999999993`` are common, the exact test fails and the plan is invalid. An
        :class:`InexactSolutionWarning` is issued whenever the fallback happens although an
        arc leaving the vertex has ``X > 0.5``; use ``backend="gurobi"`` for ``original``
        mode.
        Differs from the original: ``fixed`` uses ``X > 0.5`` and raises ``RuntimeError``
        when no arc is set (an inconsistent solution).
        """
        orig = self.compat == "original"
        inexact_warned = False
        paths = []
        for a in range(len(self.starts)):
            p = [self.starts[a]]
            for t in range(1, self.T + 1):
                prev = p[-1]
                nb = graph.neighbors(prev)
                nxt = 0 if orig else None
                if nb is None:
                    if orig:
                        from .errors import JavaNullPointerException
                        raise JavaNullPointerException(
                            'Cannot invoke "java.util.Set.toArray(Object[])" because the return value of '
                            '"java.util.Map.get(Object)" is null')
                    nb = ()
                matched = False
                for nv in nb:
                    j = self.edge_var(a, prev, nv, t - 1)
                    if j is not None and ((x[j] == 1.0) if orig else (x[j] > 0.5)):
                        nxt = nv
                        matched = True
                        break
                if nxt is None:
                    raise RuntimeError(f"no arc of agent {a} leaves vertex {prev} at time {t - 1} in the solution")
                if orig and not matched and not inexact_warned:
                    near = [x[j] for j in (self.edge_var(a, prev, nv, t - 1) for nv in nb)
                            if j is not None and x[j] > 0.5]
                    if near:
                        warnings.warn(InexactSolutionWarning(
                            f"compat='original' path extraction: agent {a} at vertex {prev}, time {t - 1}: "
                            f"no arc has X == 1.0 but one has X = {near[0]!r}; the original's exact test "
                            "falls back to vertex 0 and the plan is invalid (use backend='gurobi')"),
                            stacklevel=2)
                        inexact_warned = True
                p.append(nxt)
            paths.append(p)
        return paths


# ----------------------------------------------------------------- canonical text
_fmt = golden_number


def _row_terms(lm: LinearModel, i: int) -> list[tuple[int, float]]:
    return lm.merged_row(i)


def canonical_text(lm: LinearModel, names: Sequence[str] | None = None) -> str:
    """The golden canonical model dump of ``lm`` (format: ``tools/golden/README.md``): an
    alias of :meth:`LinearModel.to_canonical_text` (``names=None``: ``lm``'s own column
    names, ``C<j>`` for unnamed columns)."""
    return lm.to_canonical_text(True if names is None else names)


def model_summary(m: MPPModel, with_dump: bool = True) -> dict:
    """``{"ncols", "nrows", "nnz", "sense", "edgeVarMap_size", "sha256", "dump"?, "params"}``
    as in the golden ``*.prepareModel`` records (``nnz`` counts the merged, nonzero terms;
    the golden record stores ``dump`` only up to 20000 characters)."""
    text = m.canonical_text()
    lm = m.model
    nnz = sum(len(_row_terms(lm, i)) for i in range(lm.num_rows))
    out = {"ncols": lm.num_columns, "nrows": lm.num_rows, "nnz": nnz, "sense": lm.sense,
           "edgeVarMap_size": len(m.edge_var_map),
           "sha256": hashlib.sha256(text.encode("utf-8")).hexdigest(), "params": dict(m.params)}
    if with_dump:
        out["dump"] = text
    return out


# ------------------------------------------------------------------- the builder
_KINDS = ("time", "time_r", "time_puzzle", "total_time", "distance", "max_distance")


def _params(kind: str, time_limit: float, gap: float | None, debug: bool) -> dict:
    """The Gurobi parameters in effect for the model, as the original sets them.

    TimePuzzle's ``prepareModel`` sets none: Gurobi's defaults (``TimeLimit`` = 1e100,
    i.e. none) apply; its ``solve`` then sets ``OutputFlag 1`` with ``LogToConsole 0`` and
    a log file, which does not change the model.
    """
    if kind == "time_puzzle":
        return {"TimeLimit": 1e100, "MIPGap": DEFAULT_MIP_GAP, "OutputFlag": 0}
    tl = time_limit if time_limit > 0 else TIME_LIMIT_NONE
    of = (1 if debug else 0) if kind in ("time", "time_r", "total_time") else 0
    return {"TimeLimit": tl, "MIPGap": DEFAULT_MIP_GAP if gap is None else float(gap), "OutputFlag": of}


def _build(kind: str, graph: Graph, starts: Sequence[int], goals: Sequence[int], T: int, *,
           integer: bool, set_optimal: bool, reach: Reachability | None, compat: str,
           time_limit: float, gap: float | None, debug: bool, stop=None) -> MPPModel:
    orig = check_compat(compat)
    tick = _stop_check(stop)
    starts, goals = as_ids(starts, "starts"), as_ids(goals, "goals")
    if len(starts) != len(goals):
        raise ValueError("starts and goals must have the same length")
    if reach is None:
        reach = update_reachable_sets(graph, starts, goals, T, compat)
    elif reach.T != T:
        raise ValueError(f"reachability was computed for T={reach.T}, not T={T}")
    A, n = len(starts), graph.n
    lm = LinearModel("max")
    vt = "B" if integer else "C"
    if orig:
        k_e, k_vt, k_avt, k_et = edge_id, vertex_time_id, agent_vertex_time_id, edge_time_id
    else:
        def k_e(a, u, v, t):
            return (a, u, v, t)

        def k_vt(v, t):
            return (v, t)

        def k_avt(a, v, t):
            return (a, v, t)

        def k_et(u, v, t):
            return (u, v, t) if u < v else (v, u, t)

    ev: dict = {}
    vin: dict = {}
    ain: dict = {}
    aout: dict = {}
    et: dict = {}
    vin_tuples: set = set()
    avt_tuples: set = set()
    arcs: list[tuple[int, int, int, int]] = []
    move_terms: list[tuple[int, float]] = []
    moves_of: list[list[int]] = [[] for _ in range(A)]

    # (1) arc columns, in the original's order
    for arc in compact_arcs(graph, starts, reach, compat):
        if tick is not None:
            tick()
        a, cur, nxt, t0 = arc
        t1 = t0 + 1
        j = lm.add_column(0.0, 1.0, vt)
        arcs.append(arc)
        ev[k_e(a, cur, nxt, t0)] = j
        vin.setdefault(k_vt(nxt, t1), []).append(j)
        vin_tuples.add((t1, nxt))
        aout.setdefault(k_avt(a, cur, t0), []).append(j)
        ain.setdefault(k_avt(a, nxt, t1), []).append(j)
        avt_tuples.add((a, t0, cur))
        avt_tuples.add((a, t1, nxt))
        if cur != nxt:
            et.setdefault(k_et(cur, nxt, t0), []).append(j)
            move_terms.append((j, 1.0))
            moves_of[a].append(j)

    # (TimeR) one move per time step, before the loopback columns
    if kind == "time_r":
        buckets: list[list[int]] = [[] for _ in range(T + 1)]
        if orig:
            for k in sorted(ev):
                tt = time_from_edge_id(k)
                if 0 <= tt <= T and not is_v1v2_same(k):
                    buckets[tt].append(ev[k])
        else:
            for k in sorted(ev):
                if k[1] != k[2]:
                    buckets[k[3]].append(ev[k])
        for t in range(T + 1):
            lm.add_row([(j, 1.0) for j in buckets[t]], "<=", 1.0)

    # (DIST) the objective is set before the loopbacks exist
    if kind == "distance":
        lm.set_objective(move_terms, "min")

    # (2) loopback columns
    loop_cols: list[int] = []
    if kind == "total_time":
        loop_cols = [lm.add_column(1.0, 1.0, "B") for _ in range(A)]
    for a in range(A):
        if kind == "total_time":
            j = loop_cols[a]
        else:
            if kind == "distance":
                lb = 1.0 if integer else 0.0
            elif kind == "max_distance":
                lb = 1.0
            else:
                lb = 1.0 if set_optimal else 0.0
            j = lm.add_column(lb, 1.0, vt)
            loop_cols.append(j)
        s, g = starts[a], goals[a]
        vin.setdefault(k_vt(s, 0), []).append(j)
        vin_tuples.add((0, s))
        aout.setdefault(k_avt(a, g, T), []).append(j)
        ain.setdefault(k_avt(a, s, 0), []).append(j)
        avt_tuples.add((a, T, g))
        avt_tuples.add((a, 0, s))
        ev[k_e(a, g, s, T)] = j
    if kind in ("time", "time_r", "time_puzzle"):
        lm.set_objective([(j, 1.0) for j in loop_cols], "max")

    # (3) TotalTime's y variables and chain (eq. (14))
    y_total = y = None
    if kind == "total_time":
        y_total, y = [], []
        for a in range(A):
            y_total.append(lm.add_column(0.0, float(T), "I"))
            ya = [0] * T
            for t in range(T - 1, -1, -1):
                ya[t] = lm.add_column(0.0, 1.0, "B")
            y.append(ya)
        for a in range(A):
            g = goals[a]
            yt_terms = [(y_total[a], 1.0)]
            for t in range(T - 1, -1, -1):
                k = k_e(a, g, g, t)
                if k in ev:
                    xj = ev[k]
                    if t == T - 1:
                        lm.add_row([(y[a][t], 1.0), (xj, -1.0)], "==", 0.0)
                    else:
                        lm.add_row([(y[a][t], 1.0), (y[a][t + 1], -1.0), (xj, -1.0)], ">=", -1.0)
                        lm.add_row([(y[a][t], 1.0), (y[a][t + 1], -1.0)], "<=", 0.0)
                        lm.add_row([(y[a][t], 1.0), (xj, -1.0)], "<=", 0.0)
                    yt_terms.append((y[a][t], -1.0))
            lm.add_row(yt_terms, "==", 0.0)
        lm.set_objective([(j, 1.0) for j in y_total], "max")

    x_max = None
    if kind == "max_distance":
        x_max = lm.add_column(0.0, float(T), "I")
        lm.set_objective([(x_max, 1.0)], "min")

    # The R1/R2 loops of the original visit (t, v) and (a, t, v) for every v < n.
    # Visiting only the registered tuples in sorted order is equivalent whenever the
    # keys are injective on that domain (always for tuple keys).
    injective = (not orig) or (T < 10000 and (max(n, 1) - 1) * 10000 + T < 2 ** 31)

    # (a) vertex capacity (eq. (18))
    if injective:
        for (t1, v) in sorted(vin_tuples):
            if tick is not None:
                tick()
            if t1 == 0:
                continue
            lst = vin[k_vt(v, t1)]
            if len(lst) > 1:
                lm.add_row([(j, 1.0) for j in lst], "<=", 1.0)
    else:
        for t in range(T):
            for v in range(n):
                lst = vin.get(k_vt(v, t + 1))
                if lst is not None and len(lst) > 1:
                    lm.add_row([(j, 1.0) for j in lst], "<=", 1.0)

    # (b) flow conservation (eq. (9))
    def flow_row(k):
        i_ = ain.get(k)
        o_ = aout.get(k)
        if i_ is None and o_ is None:
            return
        lm.add_row([(j, -1.0) for j in (i_ or ())] + [(j, 1.0) for j in (o_ or ())], "==", 0.0)

    if injective:
        for (a, t, v) in sorted(avt_tuples):
            if tick is not None:
                tick()
            flow_row(k_avt(a, v, t))
    else:
        for a in range(A):
            for t in range(T + 1):
                for v in range(n):
                    flow_row(k_avt(a, v, t))

    # (c) head-on collisions (eq. (17)), in TreeMap order of the wrapped int keys
    if orig:
        et_keys = sorted(et)
    else:
        et_keys = sorted(et, key=lambda k: (edge_time_id(*k), k))
    for k in et_keys:
        if tick is not None:
            tick()
        lst = et[k]
        if len(lst) > 1:
            lm.add_row([(j, 1.0) for j in lst], "<=", 1.0)

    # max distance (eqs. (12)-(13)): one row per agent
    if kind == "max_distance":
        for a in range(A):
            lm.add_row([(j, 1.0) for j in moves_of[a]] + [(x_max, -1.0)], "<=", 0.0)

    collisions = _collisions(arcs, starts, goals, T, tick)
    return MPPModel(model=lm, kind=kind, T=T, starts=starts, goals=goals, compat=compat, arcs=arcs,
                    loopbacks=loop_cols, edge_var_map=ev, key_collisions=collisions,
                    params=_params(kind, time_limit, gap, debug), reach=reach,
                    y_total=y_total, y=y, x_max=x_max, graph=graph)


def _collisions(arcs, starts, goals, T, tick=None) -> dict:
    """Count, per key family, how many distinct tuples share a Java key with another."""
    e_t, e_j = set(), set()
    vt_t, vt_j = set(), set()
    avt_t, avt_j = set(), set()
    et_t, et_j = set(), set()

    def reg(a, u, v, t0, t1, move):
        e_t.add((a, u, v, t0))
        e_j.add(edge_id(a, u, v, t0))
        vt_t.add((v, t1))
        vt_j.add(vertex_time_id(v, t1))
        for (x, tt) in ((u, t0), (v, t1)):
            avt_t.add((a, x, tt))
            avt_j.add(agent_vertex_time_id(a, x, tt))
        if move:
            k = (u, v, t0) if u < v else (v, u, t0)
            et_t.add(k)
            et_j.add(edge_time_id(u, v, t0))

    for (a, u, v, t) in arcs:
        if tick is not None:
            tick()
        reg(a, u, v, t, t + 1, u != v)
    for a in range(len(starts)):
        reg(a, goals[a], starts[a], T, 0, False)
    return {"edgeVar": len(e_t) - len(e_j), "vertexTime": len(vt_t) - len(vt_j),
            "agentVertexTime": len(avt_t) - len(avt_j), "edgeTime": len(et_t) - len(et_j)}


def build_time_model(graph: Graph, starts: Sequence[int], goals: Sequence[int], T: int,
                     integer: bool = True, set_optimal: bool = True, reach: Reachability | None = None,
                     *, time_limit: float = -1.0, debug: bool = False, compat: str = "fixed", stop=None) -> MPPModel:
    """``MultiagentGraphSolverGurobiTime.prepareModel``: the makespan feasibility model at
    horizon ``T`` (maximize the sum of the loopbacks, eq. (10); with ``set_optimal`` the
    loopbacks are fixed to 1, eq. (11)). ``integer=False`` gives the LP relaxation the
    original solves first (continuous columns; the caller passes ``set_optimal=False``).

    ``reach`` defaults to :func:`update_reachable_sets` for ``T``. ``time_limit`` and
    ``debug`` only fill :attr:`MPPModel.params` (``TimeLimit``, ``OutputFlag``). ``stop``
    (every builder but :func:`build_time_puzzle_model` takes it): a callable polled while
    building; when it returns ``True``, :class:`BuildStopped` is raised.

    Differs from the original only on key collisions (see the module docstring).
    """
    return _build("time", graph, starts, goals, T, integer=integer, set_optimal=set_optimal, reach=reach,
                  compat=compat, time_limit=time_limit, gap=None, debug=debug, stop=stop)


def build_time_r_model(graph: Graph, starts: Sequence[int], goals: Sequence[int], T: int,
                       integer: bool = True, set_optimal: bool = True, reach: Reachability | None = None,
                       *, time_limit: float = -1.0, debug: bool = False, compat: str = "fixed", stop=None) -> MPPModel:
    """``MultiagentGraphSolverGurobiTimeR.prepareModel``: the makespan model plus ``T + 1``
    rows "at most one agent moves from ``t`` to ``t + 1``" (``t = 0..T``; the last one is
    always empty), added right after the arc columns. Row ``t``'s terms are the move arcs
    in ascending ``edgeVarMap`` key order.

    Differs from the original: ``original`` decodes ``t`` and ``u != v`` from the wrapped
    ``getEdgeId`` key (``getTimeFromEdgeId``, ``isV1V2Same``), so every arc leaving a
    vertex with id >= 22 is missing from these rows and agents may move simultaneously
    from there; ``fixed`` uses the true ``(u, v, t)``.
    """
    return _build("time_r", graph, starts, goals, T, integer=integer, set_optimal=set_optimal, reach=reach,
                  compat=compat, time_limit=time_limit, gap=None, debug=debug, stop=stop)


def build_time_puzzle_model(graph: Graph, starts: Sequence[int], goals: Sequence[int], T: int,
                            integer: bool = True, set_optimal: bool = True,
                            reach: Reachability | None = None, *, compat: str = "fixed") -> MPPModel:
    """``MultiagentGraphSolverGurobiTimePuzzle.prepareModel``: the same model as
    :func:`build_time_model`, with the reachability supplied by the caller (``PuzzleSolver``)
    and no time limit."""
    return _build("time_puzzle", graph, starts, goals, T, integer=integer, set_optimal=set_optimal,
                  reach=reach, compat=compat, time_limit=-1.0, gap=None, debug=False)


def build_total_time_model(graph: Graph, starts: Sequence[int], goals: Sequence[int], T: int,
                           integer: bool = True, reach: Reachability | None = None, *,
                           time_limit: float = -1.0, gap: float | None = None, debug: bool = False,
                           compat: str = "fixed", stop=None) -> MPPModel:
    """``MultiagentGraphSolverGurobiTotalTime.prepareModel``: maximize ``sum_a yT[a]`` with
    ``yT[a] = sum_t y[a][t]`` and ``y[a][t]`` = "agent ``a`` waits at its goal on every step
    from ``t`` to ``T``" (eq. (14), indexed on the goal wait arcs ``t = 0..T-1``, which is
    exact: ``A*T - objective`` is the sum of the final arrival times; the paper's eq. (15)
    is off by one per moving robot). Loopbacks are always fixed to 1. ``y[a][t]`` for
    ``t`` below the agent's distance are created but unconstrained, as in the original.
    """
    return _build("total_time", graph, starts, goals, T, integer=integer, set_optimal=True, reach=reach,
                  compat=compat, time_limit=time_limit, gap=gap, debug=debug, stop=stop)


def build_distance_model(graph: Graph, starts: Sequence[int], goals: Sequence[int], T: int,
                         integer: bool = True, reach: Reachability | None = None, *,
                         time_limit: float = -1.0, gap: float | None = None,
                         compat: str = "fixed", stop=None) -> MPPModel:
    """``MultiagentGraphSolverGurobiDistance.prepareModel``: minimize the number of move
    arcs (eq. (16) with zero-cost waits; the paper's unit-cost green arcs would make it the
    constant ``n*T``). The objective is set before the loopbacks are created; the loopbacks
    have lower bound ``1`` when ``integer`` (else 0, as in the original's dead relaxed
    branch)."""
    return _build("distance", graph, starts, goals, T, integer=integer, set_optimal=True, reach=reach,
                  compat=compat, time_limit=time_limit, gap=gap, debug=False, stop=stop)


def build_max_distance_model(graph: Graph, starts: Sequence[int], goals: Sequence[int], T: int,
                             integer: bool = True, reach: Reachability | None = None, *,
                             time_limit: float = -1.0, gap: float | None = None,
                             compat: str = "fixed", stop=None) -> MPPModel:
    """*New* (not in the original): MINMAXDIST, T-RO 2016 eqs. (11)-(13), on the compact
    model: the :func:`build_distance_model` columns and rows, loopbacks fixed to 1, an
    integer column ``x_max`` in ``[0, T]`` (last), and one row per agent
    ``sum(move arcs of a) - x_max <= 0`` (after the head-on rows); minimize ``x_max``.
    Waits cost 0 (eq. (12) with the paper's unit-cost green arcs would be the constant ``T``).

    ``compat="original"`` raises :class:`~mppflow.errors.NotInOriginal`.
    """
    if check_compat(compat):
        raise NotInOriginal("MINMAXDIST (T-RO 2016 eqs. (11)-(13)) is not implemented in the original code")
    return _build("max_distance", graph, starts, goals, T, integer=integer, set_optimal=True, reach=reach,
                  compat=compat, time_limit=time_limit, gap=gap, debug=False, stop=stop)


# ------------------------------------------------------------- full network model
@dataclass
class FullNetworkModel:
    """An ILP on the paper's full network (:class:`~mppflow.network.FullNetwork`).

    ``x[i][j]`` is the column of robot ``i`` on arc ``j``; ``y`` / ``x_max`` hold the
    objective helper columns of the total-time / max-distance variants.
    """

    model: LinearModel
    network: FullNetwork
    objective: str
    x: list[list[int]]
    y: list[list[int]] | None = None
    x_max: int | None = None

    def column_names(self) -> list[str]:
        net = self.network
        names = [""] * self.model.num_columns

        def nd(i):
            k = net.nodes[i]
            if k[0] == "v":
                return f"{k[1]}({k[2]}){chr(39) if k[3] else ''}"
            return f"{k[0]}{k[1]}-{k[2]}({k[3]})"

        for i, row in enumerate(self.x):
            for j, c in enumerate(row):
                a, b, kind = net.arcs[j]
                names[c] = f"x[{i},{kind}:{nd(a)}->{nd(b)}]"
        if self.y is not None:
            for i, row in enumerate(self.y):
                for t, c in enumerate(row):
                    names[c] = f"y[{i},{t}]"
        if self.x_max is not None:
            names[self.x_max] = "x_max"
        return names

    def retrieve_paths(self, x: Sequence[float]) -> list[list[int]]:
        """Positions from the blue arcs (``v(t) -> v(t)'``) each robot uses (``t >= 1``)."""
        net = self.network
        T = net.T
        paths = []
        for i, s in enumerate(net.starts):
            p = [s] + [None] * T
            for j, (a, b, kind) in enumerate(net.arcs):
                if kind == "blue" and x[self.x[i][j]] > 0.5:
                    _, v, t, _ = net.nodes[a]
                    p[t] = v
            if any(v is None for v in p):
                raise RuntimeError(f"robot {i} has no position at some time step")
            paths.append(p)
        return paths


def build_full_network_model(graph: Graph, starts: Sequence[int], goals: Sequence[int], T: int,
                             objective: str = "makespan", set_optimal: bool | None = None,
                             integer: bool = True, paper_costs: bool = False) -> FullNetworkModel:
    """*New*: the paper's uncompressed ILP on the Fig. 5 network (for teaching and tests;
    much larger than the compact models).

    Columns ``x[i, j]`` (robot-major, arc order of :class:`FullNetwork`), binary unless
    ``integer=False``. Rows: (8) ``sum_i x[i, j] <= 1`` for every arc; (9) flow conservation
    ``sum(in) - sum(out) = 0`` for every node and robot. The constraint ``x_{i,j} = 0`` for
    the other robots' loopbacks (``i != j``) is imposed as the column bound ``ub = 0``.

    ``objective``:

    * ``"makespan"`` -- (10) maximize ``sum_i x[i, i]``; the optimum is ``n`` iff ``T`` is
      feasible. ``set_optimal`` (default ``False``) fixes ``x[i, i] = 1`` (eq. (11)).
    * ``"distance"`` -- (11) + (16): minimize the cost of all arcs. Costs are the corrected
      ones (only the gadget middle arc costs 1) unless ``paper_costs=True``, which uses the
      paper's literal costs (green arcs cost 1 too: the objective is then the constant
      ``n*T``).
    * ``"max_distance"`` -- (11)-(13): ``x_max`` (integer, last column) bounds each robot's
      cost; minimize ``x_max``.
    * ``"total_time"`` -- (11) + a corrected eq. (14)-(15): ``y[i, t]`` (``t = 0..T-1``)
      with ``y <= at-goal(t)`` (blue arc at the goal; for ``t = 0`` the start) and
      ``y[i, t] <= y[i, t+1]``; maximize ``sum y`` (total time = ``n*T - sum y``, no
      off-by-one).
    """
    if objective not in ("makespan", "distance", "max_distance", "total_time"):
        raise ValueError(f"unknown objective {objective!r}")
    if set_optimal is None:
        set_optimal = objective != "makespan"
    net = FullNetwork.build(graph, starts, goals, T)
    n = len(net.starts)
    lm = LinearModel("max")
    vt = "B" if integer else "C"
    cost = net.paper_cost if paper_costs else net.arc_cost
    xs: list[list[int]] = []
    for i in range(n):
        row = []
        for j, (_, _, kind) in enumerate(net.arcs):
            lb, ub = 0.0, 1.0
            if kind == "loop":
                if j != i:
                    ub = 0.0
                elif set_optimal:
                    lb = 1.0
            row.append(lm.add_column(lb, ub, vt))
        xs.append(row)
    for j in range(len(net.arcs)):
        lm.add_row([(xs[i][j], 1.0) for i in range(n)], "<=", 1.0)
    ins, outs = net.in_arcs(), net.out_arcs()
    for v in range(len(net.nodes)):
        for i in range(n):
            terms = [(xs[i][j], 1.0) for j in ins[v]] + [(xs[i][j], -1.0) for j in outs[v]]
            if terms:
                lm.add_row(terms, "==", 0.0)
    y = x_max = None
    if objective == "makespan":
        lm.set_objective([(xs[i][i], 1.0) for i in range(n)], "max")
    elif objective == "distance":
        lm.set_objective([(xs[i][j], float(cost[j])) for i in range(n) for j in range(len(net.arcs))
                          if cost[j] != 0], "min")
    elif objective == "max_distance":
        x_max = lm.add_column(0.0, float(T), "I")
        for i in range(n):
            lm.add_row([(xs[i][j], float(cost[j])) for j in range(len(net.arcs)) if cost[j] != 0]
                       + [(x_max, -1.0)], "<=", 0.0)
        lm.set_objective([(x_max, 1.0)], "min")
    else:
        y = []
        blue_at = {}
        for j, (a, b, kind) in enumerate(net.arcs):
            if kind == "blue":
                _, v, t, _ = net.nodes[a]
                blue_at[(v, t)] = j
        for i in range(n):
            g, s = net.goals[i], net.starts[i]
            yi = [lm.add_column(0.0, 1.0 if (t > 0 or s == g) else 0.0, "B") for t in range(T)]
            y.append(yi)
            for t in range(T):
                if t > 0:
                    lm.add_row([(yi[t], 1.0), (xs[i][blue_at[(g, t)]], -1.0)], "<=", 0.0)
                if t + 1 < T:
                    lm.add_row([(yi[t], 1.0), (yi[t + 1], -1.0)], "<=", 0.0)
        lm.set_objective([(c, 1.0) for yi in y for c in yi], "max")
    return FullNetworkModel(model=lm, network=net, objective=objective, x=xs, y=y, x_max=x_max)
