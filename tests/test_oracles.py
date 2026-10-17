"""Randomized tests of mppflow against the independent oracles of ``tests/oracles.py``.

The instances are tiny (paths, cycles, stars, trees, 2x2..3x4 grids with holes, 1..|V|
robots, robots that start at their goal, infeasible instances) and generated from fixed
seeds. For each one the oracle computes the exact optima by joint-state search, and we check
(``compat="fixed"``):

* ``min_makespan`` (highs, builtin, gurobi when available) returns the optimal makespan, a
  valid plan of that makespan, and raises :class:`Infeasible` exactly on infeasible
  instances;
* ``min_total_time`` / ``min_total_distance`` / ``min_max_distance`` (horizon search) return
  the optimum over all horizons, with a valid plan whose metric equals the reported value;
  with ``horizon="single"`` they return the optimum at that horizon;
* :func:`mppflow.planner.plan` (every objective) agrees, its lower bounds are <= the optima;
* the split planners return valid plans (or none), with makespan / distance >= the optimum;
* :func:`mppflow.solvers.joint_feasibility` agrees with the oracle;
* the paper's full network (Fig. 5, solved by our own ILP, scipy and Gurobi) has the same
  optimal makespan and fixed-horizon distance.

The pure-Python backend runs the objectives only on instances whose horizon search stays at
T <= 5 (its branch and bound is exponential in T: 3 robots on 5 vertices take ~2 min at
T = 8).

The default run uses ``N_DEFAULT`` instances (about 1 min). ``MPPFLOW_ORACLE_SWEEP=<n>``
runs the slow sweep over ``n`` more, larger instances (marked ``slow``; skipped without the
variable). Dense instances (5-6 robots on 6-7 vertices) used to make the TT / DIST /
max-distance horizon search run for many minutes (its second horizon ``T_hi`` is up to
``n * T_opt``; such small instances are now settled by ``solvers.joint_optimum``), so the
sweep still runs every call in a forked child with a time limit and reports time-outs as
warnings. The children are forked from a new interpreter that has not solved anything: a
child forked after a HiGHS solve deadlocks (``test_forked_solves_do_not_inherit_highs``).
"""
from __future__ import annotations

import multiprocessing
import os
import random
import sys

import pytest

from mppflow import lp
from mppflow.errors import Infeasible
from mppflow.graph import Graph
from mppflow.pathfinder import get_makespan_lower_bound, get_total_time_lower_bound
from mppflow._java import MathRandom
from mppflow.planner import PathPlanner, plan
from mppflow import solvers
from mppflow.solvers import (
    joint_feasibility,
    min_makespan,
    min_max_distance,
    min_total_distance,
    min_total_time,
)

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import oracles as O  # noqa: E402

N_DEFAULT = 400
SEED = 20161013
STATE_LIMIT = 600  # reachable joint configurations of a default instance
BACKENDS = [b for b in ("highs", "builtin", "gurobi") if b in lp.available_backends()]
"""The backends of the parametrized tests: ``gurobi`` when gurobipy imports (tests/conftest.py
skips those cases without a full-size license, probed once collection is done)."""
GUROBI_PARAMS = {"Threads": 1}


def _params(backend):
    return GUROBI_PARAMS if backend == "gurobi" else None


# ------------------------------------------------------------------ instances
def random_graph(rnd: random.Random):
    kind = rnd.choice(["path", "cycle", "star", "tree", "grid", "grid", "grid"])
    if kind == "path":
        n = rnd.randint(2, 6)
        return f"path{n}", O.path_graph(n)
    if kind == "cycle":
        n = rnd.randint(3, 6)
        return f"cycle{n}", O.cycle_graph(n)
    if kind == "star":
        k = rnd.randint(2, 5)
        return f"star{k}", O.star_graph(k)
    if kind == "tree":
        n = rnd.randint(3, 7)
        return f"tree{n}", O.random_tree(n, rnd)
    rows = rnd.randint(2, 3)
    cols = rnd.randint(2, 4)
    cells = rows * cols
    nh = rnd.choice([0, 0, 1, 1, 2, 3]) if cells > 4 else rnd.choice([0, 0, 1])
    holes = sorted(rnd.sample(range(cells), nh))
    return f"grid{rows}x{cols}-h{holes}", O.grid_graph(rows, cols, holes)


def random_instance(rnd: random.Random, state_limit: int = STATE_LIMIT):
    """``(name, nbrs, starts, goals)`` with at most ``state_limit`` reachable joint
    configurations from the start (so every oracle stays cheap)."""
    while True:
        name, nbrs = random_graph(rnd)
        n = len(nbrs)
        A = rnd.randint(1, n)
        starts = rnd.sample(range(n), A)
        mode = rnd.random()
        if mode < 0.1:
            goals = list(starts)  # everyone already home
        elif mode < 0.3:
            goals = rnd.sample(range(n), A)
            for i in rnd.sample(range(A), rnd.randint(1, A)):
                if starts[i] not in goals:
                    goals[i] = starts[i]  # some robots start at their goal
                else:
                    j = goals.index(starts[i])
                    goals[i], goals[j] = goals[j], goals[i]
        elif mode < 0.33 and A >= 2:
            goals = rnd.sample(range(n), A)
            goals[1] = goals[0]  # two robots share a goal: infeasible
        else:
            goals = rnd.sample(range(n), A)
        while A > 0:
            if O.reachable_configs(nbrs, starts, state_limit) <= state_limit:
                return name, nbrs, starts, goals
            A -= 1
            starts, goals = starts[:A], goals[:A]
        # unreachable for A >= 1 (A == 1 always fits); keep drawing


def make_instances(seed: int, count: int, state_limit: int = STATE_LIMIT):
    rnd = random.Random(seed)
    return [random_instance(rnd, state_limit) for _ in range(count)]


INSTANCES = make_instances(SEED, N_DEFAULT)


def mpp_graph(nbrs) -> Graph:
    return Graph.from_adjacency(nbrs)


_ORACLE_CACHE: dict = {}


def oracle(k, inst=None):
    """Oracle values of instance ``k`` (memoized)."""
    if inst is None:
        inst = INSTANCES[k]
    key = (inst[0], tuple(map(tuple, inst[1])), tuple(inst[2]), tuple(inst[3]))
    r = _ORACLE_CACHE.get(key)
    if r is not None:
        return r
    _, nbrs, s, g = inst
    ms = O.min_makespan(nbrs, s, g)
    r = {"feasible": ms is not None}
    if ms is not None:
        r["makespan"] = ms[0]
        r["total_time"] = O.min_total_time(nbrs, s, g)[0]
        r["total_distance"] = O.min_total_distance(nbrs, s, g)[0]
        r["max_distance"] = O.min_max_distance(nbrs, s, g, upper=ms[0])[0]
        r["at"] = {}
        for T in (ms[0], ms[0] + 1, ms[0] + 2):
            r["at"][T] = {
                "total_time": O.min_total_time_at_horizon(nbrs, s, g, T)[0],
                "total_distance": O.min_total_distance_at_horizon(nbrs, s, g, T)[0],
                "max_distance": O.min_max_distance_at_horizon(nbrs, s, g, T)[0],
            }
    _ORACLE_CACHE[key] = r
    return r


def _ids(insts):
    return [f"{k}-{n}-A{len(s)}" for k, (n, _, s, _) in enumerate(insts)]


IDS = _ids(INSTANCES)
KS = list(range(len(INSTANCES)))


def check_plan(nbrs, s, g, paths, metric=None, value=None, loops=None):
    err = O.validate_plan(nbrs, s, g, paths, loops=loops)
    assert err is None, f"invalid plan: {err}; paths={paths}"
    m = O.plan_metrics(paths, g)
    if metric is not None:
        assert m[metric] == value, f"plan {metric} = {m[metric]} but reported {value}; paths={paths}"
    return m


# ------------------------------------------------------------- the oracles
def test_instance_mix():
    """The default set covers what the task asks for."""
    names = [i[0] for i in INSTANCES]
    for kind in ("path", "cycle", "star", "tree", "grid"):
        assert any(n.startswith(kind) for n in names), kind
    assert any("h[]" not in n and n.startswith("grid") for n in names)
    feas = [oracle(k)["feasible"] for k in KS]
    assert 10 <= feas.count(False) <= len(KS) // 2
    assert any(list(i[2]) == list(i[3]) for i in INSTANCES)
    assert any(any(a == b for a, b in zip(i[2], i[3])) and list(i[2]) != list(i[3]) for i in INSTANCES)
    assert any(len(i[2]) == len(i[1]) for i in INSTANCES)  # fully occupied graphs
    assert any(len(i[2]) == 1 for i in INSTANCES)


def test_oracle_hand_examples():
    P2, C3, S3 = O.path_graph(2), O.cycle_graph(3), O.star_graph(3)
    assert O.min_makespan(P2, [0, 1], [1, 0]) is None  # head-on swap
    assert O.min_makespan(C3, [0, 1, 2], [1, 2, 0])[0] == 1  # rotation of a full cycle
    assert O.min_makespan(O.path_graph(3), [0, 1], [1, 2])[0] == 1  # following
    assert O.min_makespan(O.grid_graph(2, 2), [0, 1, 3, 2], [1, 3, 2, 0])[0] == 1  # full 4-cycle
    # star: swap two leaves through the centre, a third leaf as parking spot
    assert O.min_makespan(S3, [1, 2], [2, 1])[0] == 4
    assert O.min_total_time(S3, [1, 2], [2, 1])[0] == 7
    assert O.min_total_distance(S3, [1, 2], [2, 1])[0] == 6
    assert O.min_max_distance(S3, [1, 2], [2, 1])[0] == 4
    # the distance optimum can need a longer horizon than the makespan optimum
    found = False
    for k in KS:
        r = oracle(k)
        if r["feasible"] and r["at"][r["makespan"]]["total_distance"] > r["total_distance"]:
            found = True
            break
    assert found
    # validator
    assert O.validate_plan(P2, [0, 1], [1, 0], [[0, 1], [1, 0]]).startswith("robots 0 and 1 swap")
    assert O.validate_plan(C3, [0, 1, 2], [1, 2, 0], [[0, 1], [1, 2], [2, 0]]) is None
    assert "meet" in O.validate_plan(O.path_graph(3), [0, 2], [1, 1], [[0, 1], [2, 1]])
    assert "jumps" in O.validate_plan(O.path_graph(3), [0], [2], [[0, 2]])


HAND = [
    ("edge-swap", O.path_graph(2), [0, 1], [1, 0], None),
    ("cycle3-rotation", O.cycle_graph(3), [0, 1, 2], [1, 2, 0], 1),
    ("grid2x2-rotation", O.grid_graph(2, 2), [0, 1, 3, 2], [1, 3, 2, 0], 1),
    ("path3-follow", O.path_graph(3), [0, 1], [1, 2], 1),
    ("star3-swap", O.star_graph(3), [1, 2], [2, 1], 4),
    ("star2-swap", O.star_graph(2), [1, 2], [2, 1], None),
    ("at-goal", O.grid_graph(2, 3), [0, 4], [0, 4], 0),
]


@pytest.mark.parametrize("backend", BACKENDS)
@pytest.mark.parametrize("case", HAND, ids=[h[0] for h in HAND])
def test_mppflow_hand_examples(case, backend):
    """The motion model on hand-made cases: swaps forbidden, rotations and following
    allowed."""
    _, nbrs, s, g, want = case
    assert (O.min_makespan(nbrs, s, g) or (None,))[0] == want
    G = mpp_graph(nbrs)
    if want is None:
        with pytest.raises(Infeasible):
            min_makespan(G, s, g, backend=backend, params=_params(backend))
        return
    res = min_makespan(G, s, g, backend=backend, params=_params(backend))
    assert res.objective == want
    check_plan(nbrs, s, g, res.paths, "makespan", want)


@pytest.mark.parametrize("k", KS[:60], ids=IDS[:60])
def test_oracles_consistent(k):
    """Cross-checks between the oracles: the returned witness plans are valid and have the
    claimed value; all-horizon optima <= fixed-horizon optima, non-increasing in T."""
    name, nbrs, s, g = INSTANCES[k]
    r = oracle(k)
    if not r["feasible"]:
        T = len(nbrs) + 2
        assert O.min_total_distance(nbrs, s, g) is None
        assert O.min_total_time(nbrs, s, g) is None
        assert O.min_total_distance_at_horizon(nbrs, s, g, T) is None
        return
    T = r["makespan"]
    for f, metric in ((O.min_makespan, "makespan"), (O.min_total_time, "total_time"),
                      (O.min_total_distance, "total_distance"), (O.min_max_distance, "max_distance")):
        v, p = f(nbrs, s, g)
        check_plan(nbrs, s, g, p, metric, v)
    if T > 0:
        assert O.min_total_distance_at_horizon(nbrs, s, g, T - 1) is None
    prev = None
    for TT in (T, T + 1, T + 2):
        at = r["at"][TT]
        for metric in ("total_time", "total_distance", "max_distance"):
            assert at[metric] >= r[metric]
            if prev is not None:
                assert at[metric] <= prev[metric]
        prev = at
    for f, metric in ((O.min_total_time_at_horizon, "total_time"),
                      (O.min_total_distance_at_horizon, "total_distance"),
                      (O.min_max_distance_at_horizon, "max_distance")):
        v, p = f(nbrs, s, g, T + 1)
        m = check_plan(nbrs, s, g, p, metric, v)
        assert m["makespan"] == T + 1


FULL_NET_KS = [k for k in KS if k % 4 == 0]


FULL_NET_BACKENDS = ["scipy"] + (["gurobi"] if "gurobi" in BACKENDS else [])


@pytest.mark.parametrize("ilp", FULL_NET_BACKENDS)
@pytest.mark.parametrize("k", FULL_NET_KS, ids=[IDS[k] for k in FULL_NET_KS])
def test_full_network_ilp(k, ilp):
    """Our ILP on the paper's full network (Fig. 5) agrees with the BFS oracle."""
    pytest.importorskip("scipy")
    name, nbrs, s, g = INSTANCES[k]
    r = oracle(k)
    if len(set(g)) < len(g):
        return
    if not r["feasible"]:
        assert O.full_network_ilp(nbrs, s, g, len(nbrs) + 1, backend=ilp) is None
        return
    T = r["makespan"]
    p = O.full_network_ilp(nbrs, s, g, T, backend=ilp)
    check_plan(nbrs, s, g, p, "makespan", T)
    if T > 0:
        assert O.full_network_ilp(nbrs, s, g, T - 1, backend=ilp) is None
    v, p = O.full_network_ilp(nbrs, s, g, T + 1, "distance", backend=ilp)
    assert v == r["at"][T + 1]["total_distance"]
    check_plan(nbrs, s, g, p, "total_distance", v)


# ---------------------------------------------------------------- mppflow
@pytest.mark.parametrize("backend", BACKENDS)
@pytest.mark.parametrize("k", KS, ids=IDS)
def test_min_makespan(k, backend):
    name, nbrs, s, g = INSTANCES[k]
    r = oracle(k)
    G = mpp_graph(nbrs)
    if not r["feasible"]:
        with pytest.raises(Infeasible):
            min_makespan(G, s, g, backend=backend, params=_params(backend))
        return
    res = min_makespan(G, s, g, backend=backend, params=_params(backend))
    assert res.objective == r["makespan"]
    check_plan(nbrs, s, g, res.paths, "makespan", r["makespan"])
    assert res.lower_bound is None or res.lower_bound <= r["makespan"]


OBJ = {
    "total_time": min_total_time,
    "total_distance": min_total_distance,
    "max_distance": min_max_distance,
}
OBJ_BACKENDS = [b for b in BACKENDS if b != "builtin"]


@pytest.mark.parametrize("backend", OBJ_BACKENDS)
@pytest.mark.parametrize("objective", sorted(OBJ))
@pytest.mark.parametrize("k", KS, ids=IDS)
def test_objective_horizon_search(k, objective, backend):
    name, nbrs, s, g = INSTANCES[k]
    r = oracle(k)
    G = mpp_graph(nbrs)
    f = OBJ[objective]
    if not r["feasible"]:
        with pytest.raises(Infeasible):
            f(G, s, g, backend=backend, params=_params(backend))
        return
    res = f(G, s, g, backend=backend, params=_params(backend))
    assert res.paths is not None, res.status
    assert res.objective == r[objective], f"{objective}: mppflow {res.objective}, oracle {r[objective]}"
    check_plan(nbrs, s, g, res.paths, objective, r[objective])
    assert res.lower_bound is None or res.lower_bound <= r[objective]


def _builtin_horizon(k, objective):
    """The largest horizon mppflow's horizon search will solve (predicted from the oracle's
    fixed-horizon optima and the bounds documented in ``mppflow.solvers._search``)."""
    name, nbrs, s, g = INSTANCES[k]
    r = oracle(k)
    if not r["feasible"]:
        return 0
    T0 = r["makespan"]
    d = [O.bfs_distances(nbrs, a)[b] for a, b in zip(s, g)]
    v0 = r["at"][T0][objective]
    lb = max(d) if objective == "max_distance" else sum(d)
    if v0 <= lb:
        return T0
    A = len(s)
    hi = {"total_time": v0 - lb + max(d), "total_distance": v0, "max_distance": A * v0}[objective]
    return max(T0, min(hi, A * T0))


# The pure-Python backend's branch and bound grows exponentially with the horizon (e.g.
# 3 robots on 5 vertices: 117 s at T = 8), so only models with short horizons.
BUILTIN_CASES = [(k, o) for k in KS for o in sorted(OBJ)
                 if len(INSTANCES[k][2]) <= 3 and _builtin_horizon(k, o) <= 5]


@pytest.mark.parametrize("k,objective", BUILTIN_CASES, ids=[f"{IDS[k]}-{o}" for k, o in BUILTIN_CASES])
def test_objective_horizon_search_builtin(k, objective):
    """The pure-Python backend on the instances with short horizons."""
    test_objective_horizon_search(k, objective, "builtin")


SINGLE_KS = [k for k in KS if k % 2 == 0]


@pytest.mark.parametrize("objective", sorted(OBJ))
@pytest.mark.parametrize("k", SINGLE_KS, ids=[IDS[k] for k in SINGLE_KS])
def test_objective_single_horizon(k, objective):
    """``horizon="single"`` at T*, T*+1 and T*+2 gives the oracle's fixed-horizon optimum;
    below T* nothing is returned."""
    name, nbrs, s, g = INSTANCES[k]
    r = oracle(k)
    if not r["feasible"]:
        return
    G = mpp_graph(nbrs)
    f = OBJ[objective]
    for T, at in sorted(r["at"].items()):
        res = f(G, s, g, T=T, horizon="single", backend="highs")
        assert res.paths is not None, (T, res.status)
        assert res.objective == at[objective], f"T={T}: mppflow {res.objective}, oracle {at[objective]}"
        m = check_plan(nbrs, s, g, res.paths, objective, at[objective])
        assert m["makespan"] == T
    if r["makespan"] > 0:
        try:
            res = f(G, s, g, T=r["makespan"] - 1, horizon="single", backend="highs")
        except Infeasible:
            pass
        else:
            assert res.paths is None


@pytest.mark.parametrize("objective", ["makespan", "total_time", "total_distance", "max_distance"])
@pytest.mark.parametrize("k", KS, ids=IDS)
def test_plan_api(k, objective):
    """:func:`mppflow.planner.plan`: optimal value and status, valid plan, metrics, lower
    bounds; with ``horizon="single"`` the oracle's optimum at the optimal makespan, reported
    "optimal" only when it meets the lower bound (and so is optimal over all horizons)."""
    name, nbrs, s, g = INSTANCES[k]
    r = oracle(k)
    G = mpp_graph(nbrs)
    if not r["feasible"]:
        with pytest.raises(Infeasible):
            plan((G, s, g), objective)
        return
    res = plan((G, s, g), objective)
    assert res.paths is not None, res.status
    assert res.value == r[objective], f"plan value {res.value}, oracle {r[objective]}"
    assert res.status == "optimal", res.status
    if objective != "makespan":
        # one horizon (the optimal makespan): "optimal" only if the plan is optimal overall
        one = plan((G, s, g), objective, horizon="single")
        assert one.paths is not None and one.extra["horizon"] == r["makespan"]
        assert one.value == r["at"][r["makespan"]][objective] >= r[objective]
        lb = one.lower_bounds[objective]
        assert one.status == ("optimal" if one.value == lb else "feasible"), (one.status, one.value, lb)
        if one.status == "optimal":
            assert one.value == r[objective]
    m = check_plan(nbrs, s, g, res.paths, objective, r[objective])
    for key in ("makespan", "total_time", "total_distance", "max_distance"):
        assert getattr(res, key) == m[key]
        assert m[key] >= r[key]
        lb = res.lower_bounds[key]
        assert lb is not None and lb <= r[key], (key, lb, r[key])


@pytest.mark.parametrize("k", KS, ids=IDS)
def test_lower_bounds(k):
    name, nbrs, s, g = INSTANCES[k]
    r = oracle(k)
    G = mpp_graph(nbrs)
    if not r["feasible"]:
        if any(O.bfs_distances(nbrs, a)[b] == O.INF for a, b in zip(s, g)):
            with pytest.raises(Infeasible):
                get_makespan_lower_bound(G, s, g, MathRandom(k))
        return
    lb_v = get_makespan_lower_bound(G, s, g, MathRandom(k))
    assert lb_v - 1 <= r["makespan"] if s else lb_v == 0
    tt = get_total_time_lower_bound(G, s, g, MathRandom(k))
    assert tt <= r["total_time"] and tt <= r["total_distance"]
    assert tt == sum(O.bfs_distances(nbrs, a)[b] for a, b in zip(s, g))


@pytest.mark.parametrize("k", KS, ids=IDS)
def test_joint_feasibility(k):
    """mppflow's own feasibility search (used to stop the makespan loop) agrees."""
    name, nbrs, s, g = INSTANCES[k]
    assert joint_feasibility(mpp_graph(nbrs), s, g) is oracle(k)["feasible"]


SPLITS = [
    ("plan-makespan-split2", lambda G, s, g: plan((G, s, g), "makespan", split=2).paths, "makespan"),
    ("plan-makespan-split4", lambda G, s, g: plan((G, s, g), "makespan", split=4).paths, "makespan"),
    ("plan-distance-split2", lambda G, s, g: plan((G, s, g), "total_distance", split=2).paths, "total_distance"),
    ("advanced-split1", lambda G, s, g: PathPlanner(rng=MathRandom(1)).plan_paths_advanced_split(G, s, g, True, -1.0, 1),
     "makespan"),
    ("advanced-split-no-cycle1",
     lambda G, s, g: PathPlanner(rng=MathRandom(1)).plan_paths_advanced_split_no_cycle(G, s, g, True, -1.0, 1),
     "makespan"),
]
SPLIT_KS = [k for k in KS if k % 2 == 1]


@pytest.mark.parametrize("variant", SPLITS, ids=[v[0] for v in SPLITS])
@pytest.mark.parametrize("k", SPLIT_KS, ids=[IDS[k] for k in SPLIT_KS])
def test_split_planners(k, variant):
    """Split heuristics: no plan for infeasible instances; otherwise ``None`` (a piece can
    be infeasible) or a valid plan whose metric is >= the optimum."""
    _, f, metric = variant
    name, nbrs, s, g = INSTANCES[k]
    r = oracle(k)
    G = mpp_graph(nbrs)
    try:
        paths = f(G, s, g)
    except Infeasible:
        paths = None
    if paths is None:
        return
    assert r["feasible"], f"plan for an infeasible instance: {paths}"
    m = check_plan(nbrs, s, g, paths)
    assert m[metric] >= r[metric]
    assert m["makespan"] >= r["makespan"]


# ------------------------------------------- goals without wait self-loops
# Graphs where a robot may stay only on some vertices (``oracles``, keyword ``loops``) and some
# goal has no wait loop: the horizon search then scans every horizon from the optimal makespan
# up to T_hi (``mppflow.solvers._search_every``), since a plan for T does not extend to T + 1.
NOWAIT_SEED = SEED + 7
NOWAIT_N = 40
NOWAIT_STATE_LIMIT = 150
NOWAIT_FIXED = [
    # (name, nbrs, loops, starts, goals): the optimum lies above the optimal makespan (rare in
    # random draws: about 3 in 1000), two per objective, found by an oracle search
    ("grid3x2-h[0]", [[2], [2, 3], [0, 1, 4], [1, 4], [2, 3]], [2, 3, 4], [4, 2], [0, 4]),  # TT 6 -> 5
    ("grid3x2-h[5]", [[1, 2], [0, 3], [0, 3, 4], [1, 2], [2]], [0, 3], [2, 0], [0, 4]),  # TT 6 -> 5
    ("cycle6", [[1, 5], [0, 2], [1, 3], [2, 4], [3, 5], [0, 4]], [0, 3, 5], [5, 2, 4, 0], [5, 3, 4, 0]),  # DIST 7 -> 5
    ("grid3x4-h[0, 3, 11]", [[1, 3], [0, 4], [3, 6], [0, 2, 4, 7], [1, 3, 5, 8], [4], [2, 7], [3, 6, 8], [4, 7]],
     [0, 3, 5, 7, 8], [1, 5], [8, 6]),  # DIST 8 -> 6
    ("tree7", [[1, 4], [0, 2, 3], [1, 5, 6], [1], [0], [2], [2]], [1, 2, 4, 5, 6], [6, 4], [0, 5]),  # MD 5 -> 4
    ("tree7", [[1, 3], [0, 2, 6], [1, 4, 5], [0], [2], [2], [1]], [1, 2, 3, 5, 6], [0, 2, 3], [2, 6, 4]),  # MD 6 -> 4 = lb
]


def _nowait_oracle(nbrs, loops, s, g):
    ms = O.min_makespan(nbrs, s, g, loops=loops)
    r = {"feasible": ms is not None}
    if ms is not None:
        r["makespan"] = ms[0]
        for obj, f, fh in (("total_time", O.min_total_time, O.min_total_time_at_horizon),
                           ("total_distance", O.min_total_distance, O.min_total_distance_at_horizon),
                           ("max_distance", O.min_max_distance, O.min_max_distance_at_horizon)):
            r[obj] = f(nbrs, s, g, loops=loops)[0]
            r["at_T_opt", obj] = fh(nbrs, s, g, ms[0], loops=loops)[0]
    return r


def _nowait_instance(rnd: random.Random):
    """``(name, nbrs, loops, starts, goals, oracle)``: a wait loop on each vertex with a
    probability drawn from 0.2-0.8, at least one goal without one, and a short scan (robots
    times the optimal makespan at most 12, which bounds T_hi)."""
    while True:
        name, nbrs = random_graph(rnd)
        n = len(nbrs)
        p = rnd.choice([0.2, 0.4, 0.6, 0.8])
        loops = frozenset(v for v in range(n) if rnd.random() < p)
        A = rnd.randint(1, min(n, 4))
        s, g = rnd.sample(range(n), A), rnd.sample(range(n), A)
        if all(v in loops for v in g):
            continue
        if O.reachable_configs(nbrs, s, NOWAIT_STATE_LIMIT, loops=loops) > NOWAIT_STATE_LIMIT:
            continue
        ms = O.min_makespan(nbrs, s, g, loops=loops)
        if ms is not None and A * ms[0] > 12:
            continue
        return f"{name}-loops{sorted(loops)}", nbrs, loops, s, g, _nowait_oracle(nbrs, loops, s, g)


_nowait_rnd = random.Random(NOWAIT_SEED)
NOWAIT = ([_nowait_instance(_nowait_rnd) for _ in range(NOWAIT_N)]
          + [(f"{n}-loops{lp}", nb, frozenset(lp), s, g, _nowait_oracle(nb, frozenset(lp), s, g))
             for n, nb, lp, s, g in NOWAIT_FIXED])
NOWAIT_IDS = [f"{k}-{x[0]}-A{len(x[3])}" for k, x in enumerate(NOWAIT)]


def nowait_graph(nbrs, loops) -> Graph:
    """``mpp_graph`` with the wait loops of the vertices outside ``loops`` removed."""
    G = mpp_graph(nbrs)
    for v in range(len(nbrs)):
        if v not in loops:
            G.adj[v].remove(v)
    return G


def test_nowait_instance_mix():
    """The set has infeasible instances, instances whose optimum lies above the optimal
    makespan (the scan must find it) and instances already optimal there."""
    rs = [x[5] for x in NOWAIT]
    assert sum(not r["feasible"] for r in rs) >= 2
    better = {o for r in rs if r["feasible"] for o in OBJ if r[o] < r["at_T_opt", o]}
    assert better == set(OBJ), better
    assert sum(r["feasible"] and r["total_time"] == r["at_T_opt", "total_time"] for r in rs) >= 5
    # the oracle respects the missing loops: the instance of test_solvers._no_wait_grid
    g5 = O.grid_graph(2, 5)
    assert [O.min_total_time_at_horizon(g5, [6, 8], [8, 5], T, loops={2, 8})[0] for T in (4, 5, 6, 7)] == [8, 7, 8, 9]
    # with every loop (the default) a plan extends by waiting at the goals: 7 at every horizon
    assert [O.min_total_time_at_horizon(g5, [6, 8], [8, 5], T)[0] for T in (4, 5, 6, 7)] == [7, 7, 7, 7]
    assert O.validate_plan([[1], [0]], [0], [0], [[0, 0]], loops=set()) is not None


@pytest.mark.parametrize("search", ["scan", "joint"])
@pytest.mark.parametrize("objective", sorted(OBJ))
@pytest.mark.parametrize("k", range(len(NOWAIT)), ids=NOWAIT_IDS)
def test_objective_horizon_search_without_goal_wait_loops(k, objective, search, monkeypatch):
    """The optimum over all horizons on graphs whose goals lack wait loops, proved
    ("optimal"), by the every-horizon scan (joint-state search off) and by the joint-state
    search (HiGHS)."""
    if search == "scan":
        monkeypatch.setattr(solvers, "JOINT_SEARCH_MAX_CONFIGS", 0)
    name, nbrs, loops, s, g, r = NOWAIT[k]
    G = nowait_graph(nbrs, loops)
    f = OBJ[objective]
    if not r["feasible"]:
        with pytest.raises(Infeasible):
            f(G, s, g, backend="highs")
        return
    res = f(G, s, g, backend="highs")
    hs = res.extra["horizon_search"]
    assert hs["every_horizon"] and hs["T_opt"] == r["makespan"]
    assert (res.status, res.objective) == ("optimal", r[objective]), (res.status, res.objective, r[objective])
    check_plan(nbrs, s, g, res.paths, objective, r[objective], loops=loops)
    assert res.lower_bound is None or res.lower_bound <= r[objective]
    if search == "scan" and r[objective] < r["at_T_opt", objective]:
        assert "proof" not in hs and res.T > r["makespan"]  # found by the scan above T_opt


# ------------------------------------------------------------- slow sweep
HAVE_FORK = "fork" in multiprocessing.get_all_start_methods()
"""Whether :func:`_forked` can run (no ``fork`` start method on Windows)."""
needs_fork = pytest.mark.skipif(not HAVE_FORK, reason="needs the 'fork' start method (POSIX)")
SWEEP_N = int(os.environ.get("MPPFLOW_ORACLE_SWEEP", "0") or 0)
SWEEP_TIMEOUT = float(os.environ.get("MPPFLOW_ORACLE_SWEEP_TIMEOUT", "60"))


def _sweep_calls(inst, r, backend):
    """``(tag, thunk, checker)`` for every mppflow entry point checked by the sweep on
    ``backend``; the entry points that take no backend (default HiGHS) with ``"highs"``."""
    name, nbrs, s, g = inst
    G = mpp_graph(nbrs)

    def opt_check(objective, T=None):
        def c(out):
            if isinstance(out, Infeasible):
                return None if not r["feasible"] else "Infeasible raised on a feasible instance"
            if not r["feasible"]:
                return f"no Infeasible on an infeasible instance (status {getattr(out, 'status', None)})"
            want = r[objective] if T is None else r["at"][T][objective]
            val = out.value if hasattr(out, "value") else out.objective
            if out.paths is None:
                return f"no plan (status {out.status})"
            err = O.validate_plan(nbrs, s, g, out.paths)
            if err:
                return f"invalid plan: {err}"
            m = O.plan_metrics(out.paths, g)
            if val != want or m[objective] != want:
                return f"value {val}, plan {objective} {m[objective]}, oracle {want}"
            if T is not None and m["makespan"] != T:
                return f"plan makespan {m['makespan']} != T = {T}"
            return None
        return c

    def split_check(metric):
        def c(out):
            if out is None or isinstance(out, Infeasible):
                return None
            if not r["feasible"]:
                return "plan for an infeasible instance"
            err = O.validate_plan(nbrs, s, g, out)
            if err:
                return f"invalid plan: {err}"
            m = O.plan_metrics(out, g)
            if m[metric] < r[metric]:
                return f"{metric} {m[metric]} below the optimum {r[metric]}"
            return None
        return c

    b = backend
    yield (f"min_makespan/{b}", lambda: min_makespan(G, s, g, backend=b, params=_params(b)),
           opt_check("makespan"))
    for objective, f in sorted(OBJ.items()):
        if b in OBJ_BACKENDS:
            yield (f"{objective}/{b}", lambda f=f: f(G, s, g, backend=b, params=_params(b)),
                   opt_check(objective))
        if b == "highs" and r["feasible"]:
            for T in sorted(r["at"]):
                yield (f"{objective}/single T={T}", lambda f=f, T=T: f(G, s, g, T=T, horizon="single"),
                       opt_check(objective, T))
    if b != "highs":
        return
    for objective in ("makespan", "total_time", "total_distance", "max_distance"):
        yield (f"plan/{objective}", lambda o=objective: plan((G, s, g), o), opt_check(objective))
    for vname, fn, metric in SPLITS:
        yield (f"split/{vname}", lambda fn=fn: fn(G, s, g), split_check(metric))


def _highs_used_here() -> bool:
    """Whether this process has run a HiGHS solve through mppflow (``highs._check_scipy`` runs
    first in every solve). Such a process must not fork a child that solves with HiGHS: the
    child inherits HiGHS's thread pool without its threads and deadlocks (see
    ``mppflow/lp/highs.py``, "Crashes")."""
    h = sys.modules.get("mppflow.lp.highs")
    return bool(h is not None and h._scipy_checked)


def _forked(thunk, check, timeout):
    """Run ``check(thunk())`` in a forked child (solvers in C cannot be interrupted):
    the checker's message, ``None``, or ``"TIMEOUT"``.

    Only call this in a process that has not solved with HiGHS (:func:`_fresh_python`);
    otherwise every child that reaches HiGHS hangs until ``timeout`` and the call would be
    reported as slow instead of checked. A :class:`RuntimeError` says so."""
    import multiprocessing as mp

    if _highs_used_here():
        raise RuntimeError("_forked after a HiGHS solve in this process: a forked HiGHS solve "
                           "deadlocks; run the caller through _fresh_python")
    ctx = mp.get_context("fork")
    q = ctx.Queue()

    def child():
        try:
            try:
                out = thunk()
            except Infeasible as e:
                out = e
            q.put(check(out))
        except Exception as e:  # noqa: BLE001
            q.put(f"exception {e!r}")

    p = ctx.Process(target=child)
    p.start()
    p.join(timeout)
    if p.is_alive():
        p.kill()
        p.join()
        return "TIMEOUT"
    try:
        return q.get(timeout=5)
    except Exception:  # noqa: BLE001
        return f"child died (exit code {p.exitcode})"


def _fresh_python(func, *args, timeout=None):
    """``test_oracles.<func>(*args)`` run in a new interpreter (``python -c``; the pytest
    process has usually solved with HiGHS already, so it cannot fork solves itself): its
    JSON-encoded return value. ``timeout`` (seconds) kills the interpreter
    (``subprocess.TimeoutExpired``)."""
    import json
    import subprocess

    import mppflow

    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.dirname(os.path.dirname(os.path.abspath(mppflow.__file__)))
    code = ("import json, sys; sys.path[:0] = [sys.argv[1], sys.argv[2]]; import test_oracles as T; "
            f"r = T.{func}(*json.loads(sys.argv[3])); print(); print(json.dumps(r))")
    p = subprocess.run([sys.executable, "-c", code, root, here, json.dumps(args)],
                       capture_output=True, text=True, timeout=timeout)
    assert p.returncode == 0, f"{func} failed (exit code {p.returncode}):\n{p.stderr[-3000:]}"
    return json.loads(p.stdout.strip().splitlines()[-1])


def _sweep_run(n, timeout, backend):
    """The sweep on ``backend`` (run by :func:`test_sweep` through :func:`_fresh_python`):
    ``[bad, slow]``, the wrong answers and the calls that exceeded ``timeout``."""
    insts = make_instances(SEED + 1, n, state_limit=1500)
    bad, slow = [], []
    for k, inst in enumerate(insts):
        r = oracle(None, inst)
        tag0 = f"[{k}] nbrs={inst[1]} s={inst[2]} g={inst[3]}"
        for tag, thunk, check in _sweep_calls(inst, r, backend):
            msg = _forked(thunk, check, timeout)
            if msg == "TIMEOUT":
                slow.append(f"{tag0} {tag}")
            elif msg is not None:
                bad.append(f"{tag0} {tag}: {msg}")
    return [bad, slow]


@pytest.mark.slow
@needs_fork
@pytest.mark.skipif(SWEEP_N <= 0, reason="set MPPFLOW_ORACLE_SWEEP=<n> to run the large sweep")
@pytest.mark.parametrize("backend", BACKENDS)  # the gurobi case is a Gurobi test (tests/conftest.py)
def test_sweep(backend):
    """``MPPFLOW_ORACLE_SWEEP`` more instances (up to 1500 joint configurations), every entry
    point of the default tests except the builtin objectives, one backend per case (the
    entry points without a backend argument in the ``highs`` case); each call runs in a forked
    child with a ``MPPFLOW_ORACLE_SWEEP_TIMEOUT`` (60 s) limit, forked from a new interpreter
    that never solves itself. Wrong answers fail the test; time-outs are reported as warnings
    (the horizon search can be very slow on dense instances, see the module docstring)."""
    import warnings

    bad, slow = _fresh_python("_sweep_run", SWEEP_N, SWEEP_TIMEOUT, backend)
    if slow:
        warnings.warn(f"{len(slow)} calls exceeded {SWEEP_TIMEOUT} s:\n" + "\n".join(slow))
    assert not bad, "\n".join(bad)


def _forked_highs_probe():
    """A forked HiGHS solve (3-vertex path, total time at ``T = 3``; milliseconds): the
    checker's message, ``None`` when the optimum 2 is found."""
    G = mpp_graph([[1], [0, 2], [1]])

    def check(out):
        return None if getattr(out, "objective", None) == 2 else f"returned {out!r}"

    return _forked(lambda: min_total_time(G, [0, 1], [1, 2], T=3, horizon="single", backend="highs"),
                   check, 30.0)


@needs_fork
def test_forked_solves_do_not_inherit_highs():
    """Regression: ``_forked`` used to fork the pytest process; after any earlier HiGHS solve
    there (test_cli, test_lp, ...) every forked HiGHS solve deadlocked, and the sweep reported
    those calls as slow instead of checking them. Now the sweep forks from a new interpreter,
    and ``_forked`` refuses to run in a process that has solved with HiGHS."""
    G = mpp_graph([[1], [0, 2], [1]])
    assert min_total_time(G, [0, 1], [1, 2], T=3, horizon="single", backend="highs").objective == 2
    assert _highs_used_here()
    with pytest.raises(RuntimeError, match="deadlocks"):
        _forked(lambda: None, lambda out: None, 1.0)
    assert _fresh_python("_forked_highs_probe", timeout=120) is None


# ------------------------------------------------------------ known issues
# Disagreements found by these oracles, kept as regression tests after their fixes.
FEAS_BUDGET_CASE = ([[1, 2], [0, 3], [0, 3, 5], [1, 2, 4], [3, 6], [2], [4]],  # 3x3 grid minus 2 cells
                    [1, 0, 2, 5, 3, 4], [6, 3, 0, 2, 4, 1])


@pytest.mark.slow
def test_known_infeasible_dense_instance_terminates():
    """joint_feasibility's default budget (50000) is too small for 6 robots on 7 vertices (840
    reachable configurations); the fixed-mode makespan loop used to keep that inconclusive
    answer and try T = 7, 8, ... forever. Fixed: the check is retried with a growing budget.
    The call runs in a new interpreter, killed after 60 s."""
    nbrs, s, g = FEAS_BUDGET_CASE
    assert O.min_makespan(nbrs, s, g) is None
    assert O.reachable_configs(nbrs, s) == 840
    G = mpp_graph(nbrs)
    assert joint_feasibility(G, s, g, budget=10**6) is False  # decided in ~0.03 s

    assert _fresh_python("_feas_budget_run", timeout=60.0) == "Infeasible"


def _feas_budget_run():
    """``min_makespan`` on :data:`FEAS_BUDGET_CASE` (run by the test above in a new
    interpreter, which the time-out kills): the exception's class name, or the result."""
    nbrs, s, g = FEAS_BUDGET_CASE
    try:
        return repr(min_makespan(mpp_graph(nbrs), s, g))
    except Infeasible as e:
        return type(e).__name__


BUILTIN_LP_CASES = [
    # (grid rows, cols, starts, goals, T, highs LP value): builtin's dense simplex on the TT
    # model's LP relaxation (713 / 532 columns). Before the fix (tolerant Bland ties,
    # periodic refactorization, final primal check) the first returned "optimal" 4.3e10
    # after ~300 s, the second hit the iteration cap after ~80 s; now ~280 s and ~60 s.
    pytest.param((3, 3, [8, 1, 2, 0, 3, 5], [6, 2, 8, 1, 7, 0], 7, 30.0), id="tt713",
                 marks=pytest.mark.skipif(SWEEP_N <= 0, reason="minutes; set MPPFLOW_ORACLE_SWEEP=<n>")),
    pytest.param((3, 3, [8, 2, 0, 7, 6, 1], [8, 7, 2, 5, 0, 4], 6, 23.333333333333336), id="tt532"),
]


@pytest.mark.slow
@pytest.mark.parametrize("case", BUILTIN_LP_CASES)
def test_known_builtin_lp_accuracy(case):
    from mppflow.model import build_total_time_model

    R, C, s, g, T, want = case
    G = mpp_graph(O.grid_graph(R, C))
    M = build_total_time_model(G, s, g, T).model
    assert lp.solve(M, backend="highs", relax=True).objective == pytest.approx(want)
    sol = lp.solve(M, backend="builtin", relax=True)
    assert sol.status == "optimal" and sol.objective == pytest.approx(want)
