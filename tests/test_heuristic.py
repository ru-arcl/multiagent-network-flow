"""Tests of mppflow.heuristic: the golden records of PathPlanner's and HeuristicPathPlanner's
local-repair heuristics under Gurobi (Threads=1: plans, printed text, random draws, mutated
arguments, crashes), the fixed-mode corrections, the deterministic budgets (virtual clock,
max_subproblems), the bHeuristic emulation, plan validity under HiGHS on random instances, and
the papers' garage example."""
from __future__ import annotations

import os
import re

import pytest

from mppflow import heuristic as H
from mppflow._java import MathRandom, java_double_str
from mppflow.errors import (
    Infeasible,
    JavaArrayIndexOutOfBoundsException,
    JavaException,
    JavaGRBException,
    JavaNullPointerException,
    OriginalWouldHang,
)
from mppflow.graph import Graph, create_2d_grid_graph
from mppflow.pathfinder import B_HEURISTIC
from mppflow.problem import create_grid_problem, get_long_straight_with_one_garage_problem
from helpers import GRB, graph_from_json, load_golden, needs_gurobi  # noqa: F401


HERE = os.path.dirname(__file__)
GOLDEN = os.path.join(HERE, "golden")


def _j(x):
    return x


HEUR = load_golden("heuristic.jsonl")
PLANNER_HEUR = [r for r in load_golden("planner.jsonl")
                if r["fn"] in ("PathPlanner.createSubProblem", "PathPlanner.moveAgentsIndependentlyWhenCan",
                               "PathPlanner.moveAgentsIndependently")]
BY_ID = {r["id"]: r for r in HEUR + PLANNER_HEUR}


def norm(s):
    return re.sub(r"(max time limit )(-?\d+\.\d+(E-?\d+)?)", r"\1<t>", s)


def collision_free(g, paths, starts):
    """Valid moves, no meets, no head-on swaps (rotations allowed), correct starts."""
    if not paths:
        return True
    L = len(paths[0])
    assert all(len(p) == L for p in paths)
    assert [p[0] for p in paths] == list(starts)
    for t in range(L):
        assert len({p[t] for p in paths}) == len(paths), f"meet at t={t}"
        if t:
            for p in paths:
                assert p[t] in g.adj[p[t - 1]], f"invalid move at t={t - 1}"
            mv = {(p[t - 1], p[t]) for p in paths if p[t - 1] != p[t]}
            assert not any((b, a) in mv for a, b in mv), f"head-on at t={t}"
    return True


def run_record(r, compat, backend="gurobi", params=GRB):
    """Run one golden record. Returns (value, exception, log, rng, mutated)."""
    fn, a = r["fn"], _j(r["args"])
    rng = MathRandom(r["seed"])
    cls = H.HeuristicPathPlanner if fn.startswith("HeuristicPathPlanner") else H.LocalRepairPlanner
    pl = cls(compat=compat, backend=backend, params=params, rng=rng)
    m = fn.split(".")[-1]
    mutated = {}
    log = ""
    try:
        if m == "planHeuristicPaths":
            res = pl.plan_heuristic_paths(graph_from_json(a["graph"]), a["start"], a["goal"])
            return res.paths, None, res.log, rng, mutated
        if m == "planHeuristicPath":
            g = graph_from_json(a["g"])
            if a.get("distanceToGoals") == "fromGoals":
                pl._set_distances(g, a["goal"])
            if "attempt" in a:
                pl.attempt = a["attempt"]
            s = list(a["start"] if "start" in a else a["s"])
            res = pl.plan_heuristic_path(g, s, a["goal"], a["timeLimit"])
            if s != list(a["start"] if "start" in a else a["s"]):
                mutated["start"] = s
            return res.paths, None, res.log, rng, mutated
        if m == "createSubProblem":
            g = graph_from_json(a["g"])
            if a.get("distanceToGoals") == "fromGoals":
                pl._set_distances(g, a["goal"])
            if "attempt" in a:
                pl.attempt = a["attempt"]
            if cls is H.HeuristicPathPlanner:
                p = pl.create_sub_problem(g, a["paths"], a["ct"], a["svs"], a["goal"], a["distance"])
            else:
                p = pl.create_sub_problem(g, a["paths"], a["svs"], a["goal"], a["distance"])
            val = {"adj": [[v, p.graph.neighbors(v)] for v in sorted(p.graph.adj)],
                   "vertices": p.graph.vertices, "xy": [p.graph.xy[v] for v in p.graph.vertices],
                   "sg": [p.starts, p.goals], "vidNewIdMap": dict(p.vid_new_id_map),
                   "newIdVidMap": dict(p.new_id_vid_map)}
            return val, None, "", rng, mutated
        if m == "moveAgentsIndependentlyWhenCan":
            g = graph_from_json(a["g"])
            s = list(a["start"])
            pa = [list(x) for x in a["pathArray"]]
            val = pl.move_agents_independently_when_can(g, s, a["goal"], pa)
            if s != a["start"]:
                mutated["start"] = s
            if pa != a["pathArray"]:
                mutated["pathArray"] = pa
            return val, None, "", rng, mutated
        if m == "moveAgentsIndependently":
            return H.move_agents_independently(a["paths"], compat), None, "", rng, mutated
        if m == "existsEdgeCollision":
            return H.exists_edge_collision(a["paths"], a["t"], compat), None, "", rng, mutated
        if m == "existsVertexCollision":
            return H.exists_vertex_collision(a["paths"], a["t"], compat), None, "", rng, mutated
        if m == "moveAgentsIndependentlyOneStep":
            pa = [list(x) for x in a["pathArray"]]
            H.move_agents_independently_one_step(pa, a["paths"], a["t"], compat)
            return None, None, "", rng, {"pathArray": pa}
        if m == "numberOfGoalsReached":
            return H.number_of_goals_reached(a["paths"], a["goal"], compat), None, "", rng, mutated
        if m == "printPaths":
            return None, None, H.print_paths(graph_from_json(a["g"]), a["paths"], compat), rng, mutated
        if m == "main":
            f = H.heuristic_path_planner_main if cls is H.HeuristicPathPlanner else H.path_planner_main
            return None, None, f(compat=compat, backend=backend, params=params, rng=rng), rng, mutated
    except JavaException as e:
        return None, e, getattr(e, "output", "") or "", rng, mutated
    raise AssertionError(f"unhandled record {r['id']}")


def expected_sub_problem(res):
    res = _j(res)
    g = res["graph"]
    return {"adj": g["adj"], "vertices": [v[0] for v in g["vertices"]], "xy": [None] * len(g["vertices"]),
            "sg": res["sg"], "vidNewIdMap": {k: v for k, v in res["vidNewIdMap"]},
            "newIdVidMap": {k: v for k, v in res["newIdVidMap"]}}


GOLDEN_IDS = [r["id"] for r in HEUR + PLANNER_HEUR if r["fn"] != "Sequence"]
SLOW_IDS: set[str] = set()


def _params(ids):
    return [pytest.param(i, marks=[pytest.mark.slow] if i in SLOW_IDS else []) for i in ids]


def check_against_golden(r, compat):
    val, exc, log, rng, mutated = run_record(r, compat)
    if "exception" in r:
        assert exc is not None, f"expected {r['exception']['class']}"
        assert exc.java_class == r["exception"]["class"]
        assert str(exc) == r["exception"]["message"]
    else:
        assert exc is None, exc
        exp = _j(r["result"])
        if r["fn"].endswith("createSubProblem"):
            exp = expected_sub_problem(exp)
        assert val == exp
    assert norm(log) == _j(r.get("stdout", ""))
    assert mutated == _j(r.get("mutated", {}))
    assert rng.draws == r["draws"]
    assert rng.random() == r["rng_next"]


@needs_gurobi
@pytest.mark.gurobi
@pytest.mark.parametrize("rid", _params(GOLDEN_IDS))
def test_golden_original(rid):
    """compat='original' reproduces every record exactly: result or exception (class and
    message), printed text, mutated arguments, number of draws and the next draw."""
    check_against_golden(BY_ID[rid], "original")


# records on which fixed mode deliberately differs (module docstring), with the reason
FIXED_DIFFERS = {
    "heuristic/pp/planHeuristicPaths/zero_agents": "zero robots: empty plan",
    "heuristic/hpp/planHeuristicPaths/zero_agents": "zero robots: empty plan",
    "heuristic/pp/planHeuristicPath/garage_npe": "missing distance table is computed",
    "heuristic/hpp/planHeuristicPath/garage_npe": "missing distance table is computed",
    "planner/createSubProblem/no_distance_table_npe": "missing distance table is computed",
    "heuristic/pp/planHeuristicPath/garage_table": "the caller's start is not mutated",
    "heuristic/pp/planHeuristicPath/no_conflict": "the caller's start is not mutated",
}


@needs_gurobi
@pytest.mark.gurobi
@pytest.mark.parametrize("rid", _params([i for i in GOLDEN_IDS if i not in FIXED_DIFFERS]))
def test_golden_fixed_equal(rid):
    """compat='fixed' equals the original wherever the original is correct."""
    check_against_golden(BY_ID[rid], "fixed")


@needs_gurobi
@pytest.mark.gurobi
def test_fixed_zero_agents():
    for rid in ("heuristic/pp/planHeuristicPaths/zero_agents", "heuristic/hpp/planHeuristicPaths/zero_agents"):
        r = BY_ID[rid]
        val, exc, log, rng, _ = run_record(r, "fixed")
        assert exc is None and val == []
        assert norm(log) == r["stdout"]  # the "Attempting" line is still printed
        assert rng.draws == 0


@needs_gurobi
@pytest.mark.gurobi
def test_fixed_missing_distance_table():
    """The original throws NullPointerException at the first sub-problem; fixed computes the
    goal distances itself and solves the instance (same draws up to that point)."""
    for rid in ("heuristic/pp/planHeuristicPath/garage_npe", "heuristic/hpp/planHeuristicPath/garage_npe"):
        r = BY_ID[rid]
        a = r["args"]
        val, exc, log, rng, mutated = run_record(r, "fixed")
        assert exc is None
        g = graph_from_json(a["g"])
        s = a["start"] if "start" in a else a["s"]
        collision_free(g, val, s)
        assert [p[-1] for p in val] == a["goal"]
        assert mutated == {}
        # = the original with the table supplied (same seed): plan, text and draws
        r2 = dict(r, args=dict(a, distanceToGoals="fromGoals"))
        val2, exc2, log2, rng2, _ = run_record(r2, "original")
        assert exc2 is None and val == val2 and log == log2 and rng.draws == rng2.draws
    r = BY_ID["planner/createSubProblem/no_distance_table_npe"]
    val, exc, _, rng, _ = run_record(r, "fixed")
    assert exc is None and rng.draws == r["draws"]
    # = the original with the table supplied (same seed, same draws)
    r2 = dict(r, args=dict(r["args"], distanceToGoals="fromGoals"))
    val2, exc2, _, rng2, _ = run_record(r2, "original")
    assert exc2 is None and val == val2 and rng2.draws == r["draws"]


@needs_gurobi
@pytest.mark.gurobi
def test_fixed_does_not_mutate_start():
    for rid in ("heuristic/pp/planHeuristicPath/garage_table", "heuristic/pp/planHeuristicPath/no_conflict"):
        r = BY_ID[rid]
        val, exc, log, rng, mutated = run_record(r, "fixed")
        assert val == r["result"] and norm(log) == r["stdout"] and rng.draws == r["draws"]
        assert mutated == {}
        val, exc, log, rng, mutated = run_record(r, "original")
        assert mutated == r["mutated"]


@needs_gurobi
@pytest.mark.gurobi
def test_golden_sequence_stale_distance_table():
    """One HeuristicPathPlanner reused: the second call (other goals, no table) silently uses
    the first call's distance table in the original and oscillates for 277 steps; fixed
    recomputes the table and finds a short plan."""
    r = BY_ID.get("heuristic/Sequence/stale_distanceToGoals") or next(x for x in HEUR if x["fn"] == "Sequence")
    calls = _j(r["args"])["calls"]
    expected = _j(r["result"])
    for compat in ("original", "fixed"):
        rng = MathRandom(r["seed"])
        pl = H.HeuristicPathPlanner(compat=compat, backend="gurobi", params=GRB, rng=rng)
        a1, a2 = calls[0]["args"], calls[1]["args"]
        r1 = pl.plan_heuristic_paths(graph_from_json(a1["graph"]), a1["start"], a1["goal"])
        assert r1.paths == expected[0]["result"]
        g2 = graph_from_json(a2["g"])
        r2 = pl.plan_heuristic_path(g2, a2["s"], a2["goal"], a2["timeLimit"])
        if compat == "original":
            assert r2.paths == expected[1]["result"]
            assert len(r2.paths[0]) == 278 and r2.extra["stop"] == "done"
            assert rng.draws == r["draws"]
            assert rng.random() == r["rng_next"]
            stdout = _j(r["stdout"])
            assert norm(r1.log + r2.log)[:20000] == stdout
        else:
            collision_free(g2, r2.paths, a2["s"])
            assert [p[-1] for p in r2.paths] == a2["goal"]
            assert len(r2.paths[0]) - 1 < 20


# ----------------------------------------------------------------- unit tests
def test_edge_id_collision_above_10000():
    """{20001, 10001} and {20002, 1} share the original's id 200020001."""
    paths = [[20001, 10001], [20002, 1]]
    assert H.exists_edge_collision(paths, 0, "original") == [0, 1]
    assert H.exists_edge_collision(paths, 0, "fixed") is None
    assert H.move_agents_independently(paths, "original") == 0
    assert H.move_agents_independently(paths, "fixed") == 1


def test_collision_helpers():
    paths = [[0, 1, 2], [2, 1, 0], [5, 4, 3]]
    for c in ("original", "fixed"):
        assert H.exists_vertex_collision(paths, 0, c) == [0, 1]
        assert H.exists_edge_collision(paths, 0, c) is None
        assert H.exists_edge_collision([[0, 1], [1, 0]], 0, c) == [0, 1]
        # waiting robots on distinct vertices never collide; two robots on one self-loop do
        assert H.move_agents_independently([[0, 0, 1], [3, 3, 3]], c) == 2
    pa = [[0], [8]]
    H.move_agents_independently_one_step(pa, [[0, 1, 2], [8, 7, 6]], 2)
    assert pa == [[0, 2], [8, 6]]
    assert H.number_of_goals_reached([[0, 1], [3, 3]], [1, 0]) == 1


def test_print_helpers():
    p = get_long_straight_with_one_garage_problem()
    assert H.print_paths(p.graph, [[0, 1, 2], [8, 7, 6]]) == "Agent 0: 0:0 1:1 2:2\nAgent 1: 0:8 1:7 2:6\n"
    assert H.print_start_and_goals(p.graph, p.starts, p.goals) == "Start and goal vertices: 2 pairs\n0 8  \n8 0  \n"
    g = create_2d_grid_graph(2, 2)
    assert H.print_paths(g, [[0, 1]]) == "Agent 0: 0:0(0,0) 1:1(1,0)\n"
    with pytest.raises(JavaNullPointerException):
        H.print_paths(g, None, "original")
    with pytest.raises(ValueError):
        H.print_paths(g, None)
    with pytest.raises(JavaArrayIndexOutOfBoundsException):
        H.print_paths(g, [[0, 1], [2]], "original")


def test_attempt_schedules():
    lr = H.LocalRepairPlanner
    assert lr.attempt_schedule(25) == (8, 1.0, 0.1)
    assert lr.attempt_schedule(150) == (4, 8.0, 0.0)
    assert lr.attempt_schedule(7) == (10, 3.0, 0.0)
    limits = [java_double_str(t) for _, t in lr()._attempt_limits(25)]
    assert limits == ["1.1", "1.2", "1.3", "1.4", "1.5", "1.6", "1.7000000000000002", "1.8"]
    hp = [java_double_str(t) for _, t in H.HeuristicPathPlanner()._attempt_limits(2)]
    assert hp == ["1.5833333333333333", "1.65", "1.7166666666666668"]
    hp6 = [java_double_str(t) for _, t in H.HeuristicPathPlanner()._attempt_limits(6)]
    assert hp6 == ["1.75", "1.95", "2.15"]


def _garage():
    return get_long_straight_with_one_garage_problem()


@pytest.mark.parametrize("compat", ["original", "fixed"])
def test_zero_agents_direct(compat):
    p = _garage()
    for cls in (H.LocalRepairPlanner, H.HeuristicPathPlanner):
        pl = cls(compat=compat)
        if compat == "original":
            with pytest.raises(JavaArrayIndexOutOfBoundsException):
                pl.plan_heuristic_path(p.graph, [], [], 1.0)
        else:
            r = pl.plan_heuristic_path(p.graph, [], [], 1.0)
            assert r.paths == [] and r.status == "feasible"


def test_fixed_input_checks():
    p = _garage()
    for cls in (H.LocalRepairPlanner, H.HeuristicPathPlanner):
        with pytest.raises(ValueError):
            cls().plan_heuristic_paths(p.graph, [0, 0], [8, 1])
        with pytest.raises(ValueError):
            cls().plan_heuristic_paths(p.graph, [0, 99], [8, 1])
        with pytest.raises(Infeasible):
            cls().plan_heuristic_paths(p.graph, [0, 1], [8, 8])
    # one-way: 1 cannot reach 0
    g = Graph.from_adjacency([[1], [2], [1]])
    rng = MathRandom(1)
    with pytest.raises(Infeasible):
        H.LocalRepairPlanner(rng=rng).plan_heuristic_paths(g, [1], [0])
    assert rng.draws == 0
    with pytest.raises(JavaNullPointerException):
        H.LocalRepairPlanner(compat="original", rng=MathRandom(1)).plan_heuristic_paths(g, [1], [0])


def test_goal_distances_one_way():
    """Fixed: dist(v, goal) over the reversed arcs, and a large value when v cannot reach the
    goal; original: the forward BFS from the goal (0 when unreachable)."""
    g = Graph.from_adjacency([[1], [2], [3], [0], [4]])  # one-way 4-cycle plus an isolated 4
    assert H._goal_distances(g, [3], True) == [[1, 2, 3, 0, 0]]
    assert H._goal_distances(g, [3], False) == [[3, 2, 1, 0, H._UNREACHABLE]]
    grid = create_2d_grid_graph(3, 3)
    assert H._goal_distances(grid, [8, 0], True) == H._goal_distances(grid, [8, 0], False)


@pytest.mark.parametrize("cls", [H.LocalRepairPlanner, H.HeuristicPathPlanner])
def test_budget_and_clock(cls):
    """max_subproblems and the clock end an attempt at the original's time test, before the
    first sub-problem, returning the partial paths."""
    p = _garage()
    full = cls(rng=MathRandom(1)).plan_heuristic_path(p.graph, p.starts, p.goals, 30.0)
    assert full.status == "feasible" and full.extra["solved"] >= 1
    r0 = cls(rng=MathRandom(1), max_subproblems=0).plan_heuristic_path(p.graph, p.starts, p.goals, 30.0)
    assert r0.extra["stop"] == "budget" and r0.status == "partial"
    assert r0.extra["examined"] == 1 and r0.extra["solved"] == 0
    # a negative limit expires at the first test, as in the original
    rn = cls(rng=MathRandom(1)).plan_heuristic_path(p.graph, p.starts, p.goals, -1.0)
    assert rn.extra["stop"] == "time_limit" and rn.paths == r0.paths
    # a clock that jumps 100 s per reading: the time runs out at the first test
    ticks = iter(range(0, 10 ** 6, 100))
    rc = cls(rng=MathRandom(1), clock=lambda: float(next(ticks))).plan_heuristic_path(
        p.graph, p.starts, p.goals, 30.0)
    assert rc.extra["stop"] == "time_limit" and rc.paths == r0.paths
    collision_free(p.graph, r0.paths, p.starts)


@pytest.mark.parametrize("compat", ["original", "fixed"])
def test_infeasible_corridor_partial(compat):
    """A corridor swap is infeasible: every sub-problem fails, the attempts stall, and the
    partial paths of the last attempt are returned (verified on the original:
    [[0,1,2],[6,5,4]], 0/2 goals)."""
    g = Graph.from_adjacency([[1], [0, 2], [1, 3], [2, 4], [3, 5], [4, 6], [5]])
    pl = H.HeuristicPathPlanner(compat=compat, rng=MathRandom(3))
    r = pl.plan_heuristic_paths(g, [0, 6], [6, 0])
    assert r.paths == [[0, 1, 2], [6, 5, 4]]
    assert r.status == "partial" and r.extra["goals_reached"] == 0 and r.extra["stop"] == "stalled"
    assert len(r.attempts) == 3 and all(a["examined"] == 6 for a in r.attempts)


def test_bheuristic_emulation():
    """original: heuristic=None reads PathFinder.bHeuristic; an explicit value acts as the
    global flag during the call (the sub-solvers see it) and is restored. fixed ignores the
    global flag."""
    p = create_grid_problem(6, 6, 0.1, 6, MathRandom(5))
    old = B_HEURISTIC.value

    def draws(compat, heuristic=None, flag=False):
        B_HEURISTIC.set(flag)
        try:
            rng = MathRandom(11)
            H.LocalRepairPlanner(compat=compat, rng=rng, heuristic=heuristic).plan_heuristic_paths(
                p.graph, p.starts, p.goals)
            assert B_HEURISTIC.value == flag
            return rng.draws
        finally:
            B_HEURISTIC.set(old)

    plain = draws("original")
    assert draws("original", flag=True) != plain
    assert draws("original", heuristic=True) == draws("original", flag=True)
    assert draws("fixed", flag=True) == plain == draws("fixed")
    assert draws("fixed", heuristic=True) != plain


def test_statics_after_reported():
    p = _garage()
    r = H.HeuristicPathPlanner(compat="original", rng=MathRandom(1)).plan_heuristic_paths(p.graph, p.starts, p.goals)
    assert r.extra["statics_after"] == {"MultiagentGraphSolverGurobiTime.bPrintPaths": False,
                                        "MultiagentGraphSolverGurobiTime.bDebugInfo": False}
    r = H.HeuristicPathPlanner(rng=MathRandom(1)).plan_heuristic_paths(p.graph, p.starts, p.goals)
    assert "statics_after" not in r.extra


def test_attempt_field_persists():
    """PathPlanner's loop variable is an instance field: after planHeuristicPaths it keeps the
    last attempt number and selects the strategy of later direct calls."""
    p = _garage()
    pl = H.LocalRepairPlanner(rng=MathRandom(1))
    r = pl.plan_heuristic_paths(p.graph, p.starts, p.goals)
    assert r.status == "feasible" and pl.attempt == r.attempts[-1]["attempt"] == 1
    g = create_2d_grid_graph(3, 3)
    paths = [[1, 4, 7], [3, 4, 5], [8, 8, 8]]
    pl.attempt = 1  # odd: rotate
    sp = pl.create_sub_problem(g, paths, [1, 3], [7, 5, 8])
    assert [sp.new_id_vid_map[v] for v in sp.goals] == [3, 1, 8]
    pl.attempt = 2  # even: greedy
    sp = pl.create_sub_problem(g, paths, [1, 3], [7, 5, 8])
    assert [sp.new_id_vid_map[v] for v in sp.goals] == [7, 5, 8]


@pytest.mark.parametrize("cls", [H.LocalRepairPlanner, H.HeuristicPathPlanner])
def test_random_instances_highs(cls):
    """Fixed mode under HiGHS: every plan is collision-free from the right starts, and these
    instances are all solved completely."""
    for seed in range(12):
        p = create_grid_problem(5, 5, 0.0, 12, MathRandom(seed))
        r = cls(rng=MathRandom(seed + 100)).plan_heuristic_paths(p.graph, p.starts, p.goals)
        collision_free(p.graph, r.paths, p.starts)
        assert r.status == "feasible"
        assert [q[-1] for q in r.paths] == p.goals
        assert r.objective == len(r.paths[0]) - 1


def test_two_by_two_rotation_no_ilp():
    """HeuristicPathPlanner allows a synchronous rotation (verified on the original: one
    step, no sub-ILP)."""
    g = create_2d_grid_graph(2, 2)
    r = H.HeuristicPathPlanner(rng=MathRandom(1)).plan_heuristic_paths(g, [0, 1, 3, 2], [1, 3, 2, 0])
    assert r.paths == [[0, 1], [1, 3], [3, 2], [2, 0]]
    assert r.extra["solved"] == 0 and "Solving" not in r.log


@needs_gurobi
@pytest.mark.gurobi
@pytest.mark.parametrize("cls", [H.LocalRepairPlanner, H.HeuristicPathPlanner])
def test_paper_garage_example(cls):
    """T-RO 2016 / ICRA 2013 garage example (path 0..8, garage 9 on 4, robots swap ends):
    the optimal makespan is 10 (8 moves plus entering and leaving the garage);
    HeuristicPathPlanner finds it, PathPlanner's variant needs one more step."""
    p = _garage()
    r = cls(backend="gurobi", params=GRB, rng=MathRandom(1)).plan_heuristic_paths(p.graph, p.starts, p.goals)
    collision_free(p.graph, r.paths, p.starts)
    assert r.status == "feasible"
    assert r.objective == (10 if cls is H.HeuristicPathPlanner else 11)


# ------------------------------------------------------------ regressions
_LINE4 = {"vertices": [[0, 0, 0], [1, 1, 0], [2, 2, 0], [3, 3, 0]],
          "adj": [[0, [0, 1]], [1, [0, 1, 2]], [2, [1, 2, 3]], [3, [2, 3]]], "adj_type": "TreeSet"}


@pytest.mark.parametrize("compat", ["original", "fixed"])
def test_attempt_field_after_all_attempts_fail_highs(compat):
    """Java's ``for (attempt = 1; attempt <= totalAttempts; attempt++)`` on the field ends at
    totalAttempts + 1 when every attempt fails (11 by default: odd, rotation sub-goals)."""
    pl = H.LocalRepairPlanner(compat=compat, rng=MathRandom(7))
    r = pl.plan_heuristic_paths(graph_from_json(_LINE4), [0, 3], [3, 0])  # a swap on a path: impossible
    assert r.status == "partial" and len(r.attempts) == 10
    assert pl.attempt == 11
    sp = pl.create_sub_problem(graph_from_json(_LINE4), [[1, 2, 3], [2, 1, 0]], [1, 2], [3, 0], 2)
    assert [sp.starts, sp.goals] == [[1, 2], [2, 1]]  # rotation
    # a planner whose run succeeds keeps the successful attempt's number
    q = _garage()
    pl2 = H.LocalRepairPlanner(compat=compat, rng=MathRandom(1))
    assert pl2.plan_heuristic_paths(q.graph, q.starts, q.goals).status == "feasible" and pl2.attempt == 1


@needs_gurobi
@pytest.mark.gurobi
@pytest.mark.parametrize("compat", ["original", "fixed"])
def test_attempt_field_after_all_attempts_fail_live_java(compat):
    """Live Java Sequence on one PathPlanner (probe case seq_attempt, seed 7): a fully failed
    planHeuristicPaths, then createSubProblem without setting attempt."""
    rng = MathRandom(7)
    pl = H.LocalRepairPlanner(compat=compat, backend="gurobi", params=GRB, rng=rng)
    r = pl.plan_heuristic_paths(graph_from_json(_LINE4), [0, 3], [3, 0])
    assert r.paths == [[0, 1], [3, 2]]
    sp = pl.create_sub_problem(graph_from_json(_LINE4), [[1, 2, 3], [2, 1, 0]], [1, 2], [3, 0], 2)
    assert [sp.starts, sp.goals] == [[1, 2], [2, 1]]
    assert rng.draws == 2365 and rng.random() == 0.9987057440465635


def test_distance_table_staleness_not_keyed_by_id(monkeypatch):
    """The fixed-mode stale-table check must not rely on id(): CPython reuses the id of a
    freed graph. Shadowing id() with a constant in the module makes every graph collide; the
    table must still be recomputed for the new graph."""
    monkeypatch.setattr(H, "id", lambda o: 42, raising=False)
    pl = H.LocalRepairPlanner(rng=MathRandom(1))
    pl.plan_heuristic_paths(create_2d_grid_graph(3, 3), [0, 2], [8, 6])
    assert len(pl.distance_to_goals[0]) == 9
    g2 = create_2d_grid_graph(5, 5)
    r = pl.plan_heuristic_path(g2, [5, 9], [8, 6], 100.0)
    assert r.status == "feasible" and len(pl.distance_to_goals[0]) == 25
    collision_free(g2, r.paths, [5, 9])
    # another graph object with the same vertex count and goals is also recomputed
    g3 = create_2d_grid_graph(5, 5)
    pl.plan_heuristic_path(g3, [5, 9], [8, 6], 100.0)
    assert pl._dist_key[0] is g3


@pytest.mark.parametrize("compat", ["original", "fixed"])
def test_module_level_plan_heuristic_paths_and_planner_main(compat):
    """planner.path_planner_main() calls mppflow.heuristic.plan_heuristic_paths by default."""
    from mppflow import planner

    p = _garage()
    r1 = H.plan_heuristic_paths(p.graph, p.starts, p.goals, compat=compat, rng=MathRandom(5))
    r2 = H.LocalRepairPlanner(compat=compat, rng=MathRandom(5)).plan_heuristic_paths(p.graph, p.starts, p.goals)
    assert r1.paths == r2.paths and r1.log == r2.log and r1.status == "feasible"
    a = planner.path_planner_main(compat=compat, rng=MathRandom(5))
    b = H.path_planner_main(compat=compat, rng=MathRandom(5))
    assert a == b and a.endswith("Path set is valid.\n")


@pytest.mark.parametrize("call, msg", [
    (lambda c: H.move_agents_independently([], c), "Index 0 out of bounds for length 0"),
    (lambda c: H.move_agents_independently([[0, 1], [2]], c), "Index 1 out of bounds for length 1"),
    (lambda c: H.exists_edge_collision([[0, 1]], 1, c), "Index 2 out of bounds for length 2"),
    (lambda c: H.exists_edge_collision([[0, 1]], -1, c), "Index -1 out of bounds for length 2"),
    (lambda c: H.exists_vertex_collision([[0, 1]], 1, c), "Index 2 out of bounds for length 2"),
    (lambda c: H.move_agents_independently_one_step([[0]], [[0, 1]], 2, c), "Index 2 out of bounds for length 2"),
    (lambda c: H.move_agents_independently_one_step([], [[0, 1]], 1, c), "Index 0 out of bounds for length 0"),
    (lambda c: H.number_of_goals_reached([[]], [1], c), "Index -1 out of bounds for length 0"),
])
def test_helpers_java_index_errors(call, msg):
    """original: Java's ArrayIndexOutOfBoundsException (no Python IndexError, no negative-index
    wrap-around); fixed: ValueError, or a sensible value for no robots / an empty path."""
    with pytest.raises(JavaArrayIndexOutOfBoundsException) as ei:
        call("original")
    assert str(ei.value) == msg
    try:
        v = call("fixed")
    except ValueError:
        return
    assert v == 0  # no robots / an empty path counts as not arrived


def test_helpers_take_compat():
    p = _garage()
    for c in ("original", "fixed"):
        assert H.number_of_goals_reached([[0, 1], [3, 3]], [1, 0], c) == 1
        assert H.print_start_and_goals(p.graph, p.starts, p.goals, c) == \
            "Start and goal vertices: 2 pairs\n0 8  \n8 0  \n"
        pa = [[0], [8]]
        H.move_agents_independently_one_step(pa, [[0, 1], [8, 7]], 1, c)
        assert pa == [[0, 1], [8, 7]]
    with pytest.raises(JavaNullPointerException):
        H.print_start_and_goals(p.graph, [99], [0], "original")
    with pytest.raises(ValueError):
        H.print_start_and_goals(p.graph, [99], [0], "fixed")


@pytest.mark.parametrize("cls", [H.LocalRepairPlanner, H.HeuristicPathPlanner])
def test_exactly_spent_budget(cls, monkeypatch):
    """Regression (O1, an original bug): the original's guard ``if (timeLeft < 0) return``
    lets ``timeLeft == 0.0`` through (millisecond clock, budgets such as 1.75 s are exact),
    and planPathsAdvanced(..., timeLeft) then solves the sub-problem with hasTimeLimit =
    false, i.e. without any limit: on an infeasible sub-problem the original never returns.
    ``original`` reproduces this (OriginalWouldHang); ``fixed`` treats 0 as spent."""
    g = Graph.from_adjacency([[1], [0, 2], [1, 3], [2, 4], [3, 5], [4, 6], [5]])  # corridor swap

    def clock():  # 0 at the attempt's start, then exactly 1.75 s later at every reading
        clock.n += 1
        return 0.0 if clock.n == 1 else 1.75

    def planner(compat):
        clock.n = 0
        pl = cls(compat=compat, rng=MathRandom(3), clock=clock)
        pl._set_distances(g, [6, 0])  # what plan_heuristic_paths does before its attempts
        return pl

    with pytest.raises(OriginalWouldHang):
        planner("original").plan_heuristic_path(g, [0, 6], [6, 0], 1.75)
    r = planner("fixed").plan_heuristic_path(g, [0, 6], [6, 0], 1.75)
    assert r.extra["stop"] == "time_limit" and r.extra["solved"] == 0
    # 1 ms more budget: the sub-solver gets a (tiny) wall-clock limit. Whether a backend
    # finishes anything in 1 ms depends on the machine, so the backend is stubbed: a model
    # with a limit under 10 ms times out without a solution (the LP relaxation solves only
    # when ``lp_in_time``).
    from mppflow import lp, solvers as S
    run = S._run
    lp_in_time = True

    def tiny_limit(m, backend, params, time_limit, mip_gap):
        if time_limit is not None and 0 < time_limit < 0.01:
            if m.model.is_mip or not lp_in_time:
                return lp.Solution(status="time_limit", backend=backend)
            time_limit = None
        return run(m, backend, params, time_limit, mip_gap)

    monkeypatch.setattr(S, "_run", tiny_limit)
    # the integer models time out: every sub-problem fails, and the attempt ends
    r = planner("original").plan_heuristic_path(g, [0, 6], [6, 0], 1.751)
    assert r.paths is not None and r.extra["solved"] == 0
    # the LP relaxation times out too: the original reads ObjVal of a model without a
    # solution (GRBException, as Gurobi raises it)
    lp_in_time = False
    with pytest.raises(JavaGRBException, match="ObjVal"):
        planner("original").plan_heuristic_path(g, [0, 6], [6, 0], 1.751)
