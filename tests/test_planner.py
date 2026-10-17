"""Tests of mppflow.planner: golden PathPlanner records (utilities, split goals, the split
planners under Gurobi with Threads=1: plans, printed text, random draws, crashes and hangs),
the fixed-mode corrections, the original's endless retry loops, the paper's worked examples
(T-RO Fig. 3, Fig. 8, Fig. 14/15) and the high-level plan() API."""
from __future__ import annotations

import json
import os
import re
import warnings

import pytest

from mppflow import planner as P
from mppflow._java import MathRandom
from mppflow.errors import (
    InexactSolutionWarning,
    Infeasible,
    JavaArrayIndexOutOfBoundsException,
    JavaException,
    JavaNullPointerException,
    JavaStackOverflowError,
    NotInOriginal,
    OriginalWouldHang,
)
from mppflow.graph import Graph, create_2d_grid_graph
from mppflow.problem import Problem, create_grid_problem
from helpers import (GRB, from_edges, graph_from_json, java_class,  # noqa: F401
                     load_golden, needs_gurobi, ring)


HERE = os.path.dirname(__file__)
GOLDEN = os.path.join(HERE, "golden")
FIXTURES = os.path.join(HERE, "data", "paper")  # the published paper fixtures


def norm(s):
    return re.sub(r"(found in |max time limit )(-?\d+\.\d+(E-?\d+)?|-?Infinity|NaN)", r"\1<t>", s)


RECS = load_golden("planner")

HEURISTIC_FNS = ("PathPlanner.moveAgentsIndependently", "PathPlanner.moveAgentsIndependentlyWhenCan",
                 "PathPlanner.createSubProblem")
UTIL_FNS = ("PathPlanner.printPaths", "PathPlanner.printPathToString", "PathPlanner.printStartAndGoals",
            "PathPlanner.numberOfGoalsReached", "PathPlanner.parsePaths", "PathPlanner.hasCycles",
            "PathPlanner.isPathSetValid", "PathPlanner.mergePaths")
SPLIT_FNS = ("PathPlanner.splitPaths", "PathPlanner.splitPathBySize", "PathPlanner.getLongestPathLength")
UTIL_RECS = [r for r in RECS if r["fn"] in UTIL_FNS]
SPLIT_RECS = [r for r in RECS if r["fn"] in SPLIT_FNS]
SOLVE_RECS = [r for r in RECS if r["fn"] not in UTIL_FNS + SPLIT_FNS + HEURISTIC_FNS]


def make_planner(rec, compat, backend="gurobi", params=GRB):
    flags = P.SolverFlags.java_defaults().apply_java(rec.get("statics") or {})
    return P.PathPlanner(backend=backend, params=params, rng=MathRandom(rec["seed"]), compat=compat, flags=flags)


def call(pl: P.PathPlanner, fn: str, a: dict):
    """Run one Java call on planner ``pl``; returns (result, extra)."""
    name = fn.split(".", 1)[1]
    g = graph_from_json(a["g"]) if "g" in a else None
    if name == "splitPaths":
        return pl.split_paths(g, a["start"], a["goal"], a["random"]), None
    if name == "splitPathBySize":
        return pl.split_path_by_size(g, a["start"], a["goal"], a["random"], a["firstSize"]), None
    if name == "getLongestPathLength":
        return pl.get_longest_path_length(g, a["start"], a["goal"]), None
    if name == "planPathsAdvanced":
        return pl.plan_paths_advanced(g, a["start"], a["goal"], a["solveRelaxed"], a["extraSteps"],
                                      a["setOptimal"], a["timeLimit"]), None
    if name == "planPathsAdvancedPMG":
        return pl.plan_paths_advanced_pmg(g, a["start"], a["goal"], a["solveRelaxed"], a["extraSteps"],
                                          a["setOptimal"], a["timeLimit"]), None
    if name == "planPathsAdvancedTT":
        opt = list(a["opt"])
        r = pl.plan_paths_advanced_tt(g, a["start"], a["goal"], opt, a["gap"], a["timeSteps"], a["timeLimit"],
                                      horizon="single")
        return r, {"opt": opt}
    if name == "planPathsAdvancedTD":
        return pl.plan_paths_advanced_td(g, a["start"], a["goal"], a["gap"], a["timeSteps"], a["timeLimit"],
                                         horizon="single"), None
    if name == "planPathsAdvancedSplit":
        return pl.plan_paths_advanced_split(g, a["start"], a["goal"], a["setOptimal"], a["timeLimit"],
                                            a["splits"]), None
    if name == "planPathsAdvancedSplitDist":
        return pl.plan_paths_advanced_split_dist(g, a["start"], a["goal"], a["gap"], a["timeLimit"],
                                                 a["splits"]), None
    if name == "planPathsAdvancedSplitNoCycle":
        return pl.plan_paths_advanced_split_no_cycle(g, a["start"], a["goal"], a["setOptimal"], a["timeLimit"],
                                                     a["splits"], a["top"]), None
    if name == "planPathsAdvancedSplitBySize":
        return pl.plan_paths_advanced_split_by_size(g, a["start"], a["goal"], a["setOptimal"], a["timeLimit"],
                                                    a["size"]), None
    if name == "planPathsAdvancedSplitBySizeNoCycle":
        return pl.plan_paths_advanced_split_by_size_no_cycle(g, a["start"], a["goal"], a["setOptimal"],
                                                             a["timeLimit"], a["size"], a["top"]), None
    if name == "multiThreadedSplitPlanning":
        r = pl.multi_threaded_split_planning(g, a["start"], a["goal"], a["timeLimit"], a["splits"])
        return r, {"msg": pl.msg, "mpath_keys": sorted(pl.mpath)}
    raise AssertionError(fn)


def run_util(rec, compat):
    """Run a utility record; returns (result, stdout)."""
    a, name = rec["args"], rec["fn"].split(".", 1)[1]
    if name == "printPaths":
        return None, P.print_paths(graph_from_json(a["g"]), a["paths"], compat)
    if name == "printPathToString":
        return P.print_path_to_string(graph_from_json(a["g"]), a["paths"], compat), ""
    if name == "printStartAndGoals":
        return None, P.print_start_and_goals(graph_from_json(a["g"]), a["start"], a["goal"], compat)
    if name == "numberOfGoalsReached":
        return P.number_of_goals_reached(a["paths"], a["goal"], compat), ""
    if name == "parsePaths":
        return P.parse_paths(a["paths"], compat), ""
    if name == "hasCycles":
        return P.has_cycles(a["paths"], compat), ""
    if name == "isPathSetValid":
        msg = P.path_set_error(a["paths"], graph_from_json(a["graph"]), a["starts"], a["goals"], compat)
        return msg is None, "" if msg is None else msg + "\n"
    if name == "mergePaths":
        return P.merge_paths(a["pathFromStart"], a["pathFromMiddle"], compat), ""
    raise AssertionError(name)


# ------------------------------------------------------------- golden utilities
@pytest.mark.parametrize("rec", UTIL_RECS, ids=lambda r: r["id"])
def test_golden_utilities_original(rec):
    if "exception" in rec:
        with pytest.raises(JavaException) as ei:
            run_util(rec, "original")
        assert java_class(ei.value) == rec["exception"]["class"]
        out = getattr(ei.value, "output", None)
        if out is not None:
            assert out == rec["stdout"]
        return
    res, out = run_util(rec, "original")
    assert res == rec["result"]
    assert out == rec["stdout"]


@pytest.mark.parametrize("rec", UTIL_RECS, ids=lambda r: r["id"])
def test_golden_utilities_fixed(rec):
    """``fixed`` equals the original on valid input and fails cleanly elsewhere."""
    rid = rec["id"]
    special = {
        "planner/printPaths/null_npe": ValueError,
        "planner/parsePaths/null_npe": ValueError,
        "planner/hasCycles/null_npe": ValueError,
        "planner/isPathSetValid/null_npe": ValueError,
        "planner/mergePaths/discontinuous": ValueError,
    }
    if rid in special:
        with pytest.raises(special[rid]):
            run_util(rec, "fixed")
        return
    res, out = run_util(rec, "fixed")
    if rid == "planner/printPaths/ragged_aioobe":
        assert out == "Agent 0: 0:0(0,0) 1:1(1,0) 2:2(2,0)\nAgent 1: 0:3(0,1) 1:4(1,1)\n"
        return
    if rid == "planner/hasCycles/empty_aioobe":
        assert res == 0
        return
    if rid.startswith("planner/isPathSetValid/") and rid.split("/")[-1] in ("rows0", "ragged_short", "ragged_long"):
        assert res is False and out
        return
    if rid == "planner/isPathSetValid/empty_aioobe":
        assert res is True and out == ""  # zero robots: the empty path set is valid
        return
    assert res == rec["result"]
    assert out == rec["stdout"]


def test_print_paths_indexes_vertices_array_in_original():
    """``printPaths`` uses ``vertices[id]``: on a graph with ids 1..n the original prints
    the next vertex (fixed: the vertex itself)."""
    g = Graph()
    for v, nb in [(1, [2]), (2, [1, 3]), (3, [2])]:
        g.add_vertex(v, nb)
        g.xy[v] = (v, 0)
    g.finish_building_graph()
    assert P.print_paths(g, [[1, 2]], "fixed") == "Agent 0: 0:1(1,0) 1:2(2,0)\n"
    assert P.print_paths(g, [[0, 1]], "original") == "Agent 0: 0:1(1,0) 1:2(2,0)\n"
    with pytest.raises(JavaArrayIndexOutOfBoundsException):
        P.print_paths(g, [[3]], "original")


def test_path_set_error_order_and_fast_path():
    g = create_2d_grid_graph(3, 3, True)
    # (1, 2) meet at t=1 and (0, 2) at t=2: pairs are checked first, so (0, 2) is reported
    paths = [[0, 1, 1], [3, 4, 4], [5, 4, 1]]
    for compat in ("original", "fixed"):
        assert P.path_set_error(paths, g, [0, 3, 5], [1, 4, 1], compat) == \
            "Path 0 and 2 meet on vertex 1 at time step 2"
        assert P.path_set_error(paths, g, [0, 3, 5], [1, 4, 1], compat) == \
            P._path_set_error_literal(paths, g, [0, 3, 5], [1, 4, 1])
    swap = [[0, 1, 2], [3, 4, 1]]
    assert P.path_set_error(swap, g, [0, 3], [2, 1]) is None
    swap2 = [[0, 1, 4], [3, 4, 1]]
    assert P.path_set_error(swap2, g, [0, 3], [4, 1]) == "Path 0 and 1 cross the same edge at time step 2"
    assert P.path_set_error([[0, 1], [1, 2]], g, [0, 1], [1, 2]) is None  # a train is allowed
    assert P.path_set_error([[0, 1]], g, [0, 1], [1, 2], "fixed").startswith("The path set has 1 paths")
    assert P.is_path_set_valid([[0, 1]], g, [0], [1])


def test_has_cycles_semantics():
    assert P.has_cycles([[0, 1], [1, 3], [3, 4], [4, 0]]) == 1
    assert P.has_cycles([[0, 1, 0], [1, 0, 1]]) == 2
    assert P.has_cycles([[0, 1], [1, 2]]) == 0
    with pytest.raises(JavaNullPointerException):
        P.has_cycles(None, "original")
    with pytest.raises(ValueError):
        P.has_cycles([[0, 1], [2]])


def test_merge_paths_and_parse_paths():
    assert P.merge_paths([[0, 1]], [[1, 2]]) == [[0, 1, 2]]
    assert P.merge_paths([], []) == []
    with pytest.raises(JavaArrayIndexOutOfBoundsException):
        P.merge_paths([], [], "original")
    assert P.parse_paths([[1], [2, 3]]) == [[1, 1], [2, 3]]
    with pytest.raises(JavaArrayIndexOutOfBoundsException):
        P.parse_paths([[], [2, 3]], "original")
    with pytest.raises(ValueError):
        P.parse_paths([[], [2, 3]])


def test_plan_metrics():
    m = P.plan_metrics([[0, 1, 2, 2], [5, 5, 4, 5]], [2, 5])
    assert m == {"makespan": 3, "total_time": 2 + 3, "total_distance": 4, "max_distance": 2}
    # arrival 0 (start == goal, also after a detour: the robot leaves and comes back) and 1
    assert P.plan_metrics([[2]], [2]) == {"makespan": 0, "total_time": 0, "total_distance": 0, "max_distance": 0}
    assert P.plan_metrics([[2, 2, 2], [1, 3, 3]], [2, 3])["total_time"] == 0 + 1
    assert P.plan_metrics([[2, 1, 2], [3, 3, 3]], [2, 3])["total_time"] == 2 + 0
    assert P.plan_metrics([[1, 2, 2, 2]], [2]) == {"makespan": 3, "total_time": 1, "total_distance": 1,
                                                   "max_distance": 1}
    assert P.plan_metrics([[1, 2, 1]], [2])["total_time"] is None  # does not end at its goal


# ------------------------------------------------------------- golden split goals
@pytest.mark.parametrize("rec", SPLIT_RECS, ids=lambda r: r["id"])
def test_golden_split_original(rec):
    pl = make_planner(rec, "original")
    if rec.get("hang"):
        with pytest.raises(OriginalWouldHang):
            call(pl, rec["fn"], rec["args"])
        return
    res, _ = call(pl, rec["fn"], rec["args"])
    assert res == rec["result"]
    assert pl.rng.draws == rec["draws"]
    assert pl.rng.next_double() == rec["rng_next"]


@pytest.mark.parametrize("rec", SPLIT_RECS, ids=lambda r: r["id"])
def test_golden_split_fixed(rec):
    """``fixed``: same goals and draws where the original terminates (and its
    ``getLongestPathLength`` is right); a valid fallback where it hangs."""
    pl = make_planner(rec, "fixed")
    a = rec["args"]
    res, _ = call(pl, rec["fn"], a)
    name = rec["fn"].split(".")[1]
    if name == "getLongestPathLength":
        g = graph_from_json(a["g"])
        from mppflow.pathfinder import get_distances_to_all_vertices
        want = max(get_distances_to_all_vertices(g, s)[t] + 1 for s, t in zip(a["start"], a["goal"]))
        assert res == want
        return
    assert len(set(res)) == len(res)  # distinct intermediate goals
    if rec["id"] in SPLIT_ENDPOINT_DIFFERS:
        pl = make_planner(rec, "fixed")
        pl._split_endpoints = False
        res, _ = call(pl, rec["fn"], a)
    if name == "splitPaths" and not rec.get("hang"):
        assert res == rec["result"]
        assert pl.rng.draws == rec["draws"]
    if name == "splitPathBySize" and not rec.get("hang"):
        assert pl.rng.draws == rec["draws"]


def test_split_fallback_fixed():
    """2x2 grid: the third robot (3 -> 2) finds {0, 1} taken; the original widens its
    window forever, fixed gives it its goal (an endpoint, adjacent to its start)."""
    g = create_2d_grid_graph(2, 2, True)
    assert P.split_paths(g, [0, 2, 3], [3, 1, 2], rng=MathRandom(1)) == [1, 0, 2]
    with pytest.raises(OriginalWouldHang):
        P.split_paths(g, [0, 2, 3], [3, 1, 2], rng=MathRandom(1), compat="original")
    # with the original's rule (endpoints excluded) fixed falls back to the start
    pl = P.PathPlanner(rng=MathRandom(1))
    pl._split_endpoints = False
    assert pl.split_paths(g, [0, 2, 3], [3, 1, 2]) == [1, 0, 3]


def test_split_fallback_bfs_needs_a_vertex_that_reaches_the_goal():
    """fixed fallback with both endpoints taken: the nearest free vertex (BFS from the start)
    from which the goal can be reached. On this one-way graph 1 is nearer but a dead end."""
    g = Graph.from_adjacency([[1, 2], [], [3], [4], []])  # 0 -> 1, 0 -> 2 -> 3 -> 4
    assert P._split_fallback(g, 0, 4, {0}, 0) == 4 and P._split_fallback(g, 0, 4, {4}, 0) == 0
    assert P._split_fallback(g, 0, 4, {0, 4}, 0) == 2
    assert P._split_fallback(g, 0, 4, {0, 4, 2}, 0) == 3
    with pytest.raises(Infeasible, match="intermediate goal of robot 7"):
        P._split_fallback(g, 0, 4, {0, 2, 3, 4}, 7)


def test_split_paths_fig8_and_endpoints():
    g = create_2d_grid_graph(4, 4, True)
    for compat in ("fixed", "original"):
        assert P.split_paths(g, [13, 8], [0, 7], rng=MathRandom(1), compat=compat) == [5, 10]
    g3 = create_2d_grid_graph(3, 3, True)
    # original: a stationary robot and an adjacent pair are sent on a detour
    # (original-bugs B11); fixed lets them use their endpoints
    assert P.split_paths(g3, [0, 4], [1, 4], rng=MathRandom(1), compat="original") == [2, 1]
    assert P.split_paths(g3, [0, 4], [1, 4], rng=MathRandom(1)) == [0, 4]
    assert P.split_paths(g3, [4], [4], rng=MathRandom(1), compat="original") == [1]
    assert P.split_paths(g3, [4], [4], rng=MathRandom(1)) == [4]
    # same draws in both modes
    a, b = MathRandom(3), MathRandom(3)
    P.split_paths(g3, [0, 4, 8], [1, 4, 5], True, rng=a)
    P.split_paths(g3, [0, 4, 8], [1, 4, 5], True, rng=b, compat="original")
    assert a.draws == b.draws


@pytest.mark.parametrize("backend", ["highs", "builtin"])
def test_split_no_detour_for_stationary_or_adjacent_robots(backend):
    """Regression: ``plan(split=k)`` keeps the unsplit makespan when robots are
    at, or one step from, their goals."""
    g = create_2d_grid_graph(3, 3, True)
    r = P.plan((g, [0, 2], [1, 2]), split=2, backend=backend)
    assert r.makespan == 1 and r.intermediate_goals == [[0, 2]]
    assert P.path_set_error(r.paths, g, [0, 2], [1, 2]) is None
    r = P.plan((g, [4], [4]), split=2, backend=backend)
    assert r.makespan == 0 and r.paths == [[4]]
    r = P.plan((g, [0], [0]), split=4, backend=backend)
    assert r.makespan == 0 and r.paths == [[0]]
    for fn in ("plan_paths_advanced_split", "plan_paths_advanced_split_no_cycle"):
        res = getattr(P, fn)(g, [0, 4], [1, 4], True, -1, 1, rng=MathRandom(1), backend=backend)
        assert len(res[0]) - 1 == 1 and P.path_set_error(res, g, [0, 4], [1, 4]) is None
    res = P.plan_paths_advanced_split_dist(g, [0, 4], [1, 4], 0.0, -1, 1, rng=MathRandom(1), backend=backend)
    assert P.plan_metrics(res, [1, 4])["total_distance"] == 1
    if backend == "highs":
        # the original's rule still gives the detour (makespan 6)
        pl = P.PathPlanner(rng=MathRandom(1))
        pl._split_endpoints = False
        res = pl.multi_threaded_split_planning(g, [0, 2], [1, 2], -1, 1)
        assert len(res[0]) - 1 == 6


def test_split_path_by_size_fixed_terminates():
    g3 = create_2d_grid_graph(3, 3, True)
    with pytest.raises(OriginalWouldHang):
        P.split_path_by_size(g3, [0], [8], False, 2, rng=MathRandom(1), compat="original")
    m = P.split_path_by_size(g3, [0], [8], False, 2, rng=MathRandom(1))
    assert m[0] in (1, 3)  # one step from the start, on a shortest path
    m = P.split_path_by_size(g3, [0, 2, 6], [8, 6, 1], False, 9, rng=MathRandom(1))
    assert m == [8, 6, 1]  # longer than every path: the goals


# ------------------------------------------------------------- golden solve records
def check_record(rec, compat, split_endpoints=None, planner=None):
    pl = planner or make_planner(rec, compat)
    if split_endpoints is not None:
        pl._split_endpoints = split_endpoints
    nondet = set(rec.get("nondet") or ())
    if rec["fn"] == "Sequence":
        outs = []
        for c in rec["args"]["calls"]:
            res, extra = call(pl, c["fn"], c["args"])
            outs.append((res, extra))
        for (res, extra), want in zip(outs, rec["result"]):
            assert res == want["result"]
            if "extra" in want:
                assert extra == want["extra"]
        return pl
    if rec.get("hang"):
        with pytest.raises(OriginalWouldHang):
            call(pl, rec["fn"], rec["args"])
        return pl
    if "exception" in rec:
        with pytest.raises(JavaException) as ei:
            call(pl, rec["fn"], rec["args"])
        assert java_class(ei.value) == rec["exception"]["class"]
        return pl
    res, extra = call(pl, rec["fn"], rec["args"])
    if "result" not in nondet:
        assert res == rec["result"]
    if "stdout" not in nondet:
        assert norm(pl.log) == rec["stdout"]
    if "draws" not in nondet:
        assert pl.rng.draws == rec["draws"]
    if "rng_next" not in nondet:
        assert pl.rng.next_double() == rec["rng_next"]
    if "mutated" in rec:
        assert extra == {k: v for k, v in rec["mutated"].items()}
    if "extra" in rec and extra is not None:
        assert extra["msg"] == rec["extra"]["msg"]
        if "result" not in nondet:
            assert extra["mpath_keys"] == rec["extra"]["mpath_keys"]
    return pl


@pytest.mark.gurobi
@needs_gurobi
@pytest.mark.parametrize("rec", SOLVE_RECS, ids=lambda r: r["id"])
def test_golden_planner_original_gurobi(rec):
    check_record(rec, "original")


# Records whose outcome the solver does not decide: a hang, a Java exception, or null (an
# invalid path set, a failed piece). They run under HiGHS too, so that an install without a
# Gurobi license still checks the original-mode code they reach (the hang analysis of the
# split planners, the input checks, the validation of the returned path set).
SOLVER_FREE_RECS = [r for r in SOLVE_RECS if r["fn"] != "Sequence"
                    and (r.get("hang") or "exception" in r or r.get("result", 1) is None)]


@pytest.mark.parametrize("rec", SOLVER_FREE_RECS, ids=lambda r: r["id"])
def test_golden_planner_original_highs_solver_free(rec):
    """The original reads a plan with exact tests (X == 1.0); a HiGHS release that returns
    1 + 6e-15 changes the outcome of a record that solves models before it hangs (the
    InexactSolutionWarning names it), and only then is the record skipped."""
    pl = make_planner(rec, "original", backend="highs", params=None)
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always", InexactSolutionWarning)
        try:
            check_record(rec, "original", planner=pl)
        except BaseException:
            if not any(issubclass(w.category, InexactSolutionWarning) for w in caught):
                raise
            pytest.skip("this HiGHS returned inexact 0/1 values; the original's exact path extraction needs Gurobi")


# Records on which fixed mode deliberately differs (see the module docstring of planner).
FIXED_DIFFERS = {
    "planner/planPathsAdvanced/dupstarts": "ValueError",
    "planner/planPathsAdvancedTT/line3_T2_nan": "accepts the optimum 0",
    "planner/planPathsAdvancedTD/stay_T0_nan": "accepts the optimum 0",
    "planner/planPathsAdvancedSplitDist/stay_k0_nan": "accepts the optimum 0",
    "planner/planPathsAdvancedSplitDist/3x3_k1_nolimit_null": "no limit -> both halves",
    "planner/planPathsAdvancedSplitDist/3x3_k2_zero_null": "no limit -> both halves",
    "planner/planPathsAdvancedSplitDist/line2_swap_k0_npe": "Infeasible",
    "planner/multiThreadedSplitPlanning/3x3_k1_nolimit_null": "no limit",
    "planner/multiThreadedSplitPlanning/3x3_k2_zero_null": "no limit",
    "planner/multiThreadedSplitPlanning/worker_npe_hang": "Infeasible",
    "planner/Sequence/mt_stale_mpath": "no stale pieces",
    "planner/planPathsAdvancedSplitBySize/3x3_size3": "corrected by-size split",
    "planner/planPathsAdvancedSplitBySize/3x3_size4": "corrected by-size split",
    "planner/planPathsAdvancedSplitBySize/3x3_size6": "corrected longest path",
    "planner/planPathsAdvancedSplitBySize/line8_size5": "corrected longest path",
    "planner/planPathsAdvancedSplitBySizeNoCycle/3x3_size3": "corrected by-size split",
    "planner/planPathsAdvancedSplitBySizeNoCycle/3x3_size4": "corrected by-size split",
}


# Records whose split sends a robot at (or next to) its goal on a detour in the original
# (original-bugs B11). Fixed may use the endpoints as intermediate goals;
# with that rule switched off it reproduces the record exactly, and with it the makespan
# is never worse.
SPLIT_ENDPOINT_DIFFERS = {
    "planner/splitPaths/square_random_ok",
    "planner/planPathsAdvancedSplit/3x3_k2",
    "planner/planPathsAdvancedSplit/8x8_k2",
    "planner/planPathsAdvancedSplitDist/3x3_k2",
    "planner/planPathsAdvancedSplitDist/stay_k1_detour",
    "planner/planPathsAdvancedSplitNoCycle/3x3_k2",
    "planner/multiThreadedSplitPlanning/3x3_k2",
    "planner/multiThreadedSplitPlanning/8x8_20_k2",
}


@pytest.mark.gurobi
@needs_gurobi
@pytest.mark.parametrize("rec", SOLVE_RECS, ids=lambda r: r["id"])
def test_golden_planner_fixed_gurobi(rec):
    """``fixed`` reproduces every record on which the original is correct; elsewhere it
    returns a valid plan (or a clean failure)."""
    rid = rec["id"]
    if rid in SPLIT_ENDPOINT_DIFFERS:
        check_record(rec, "fixed", split_endpoints=False)
        pl = make_planner(rec, "fixed")
        res, _ = call(pl, rec["fn"], rec["args"])
        a = rec["args"]
        assert P.path_set_error(res, graph_from_json(a["g"]), a["start"], a["goal"]) is None
        assert len(res[0]) <= len(rec["result"][0])
        return
    if rid not in FIXED_DIFFERS and not rec.get("hang") and "exception" not in rec:
        check_record(rec, "fixed")
        return
    pl = make_planner(rec, "fixed")
    calls = rec["args"]["calls"] if rec["fn"] == "Sequence" else [{"fn": rec["fn"], "args": rec["args"]}]
    for c in calls:
        a = c["args"]
        g = graph_from_json(a["g"])
        try:
            res, _ = call(pl, c["fn"], a)
        except (ValueError, Infeasible):
            assert rid in ("planner/planPathsAdvanced/dupstarts", "planner/multiThreadedSplitPlanning/worker_npe_hang",
                           "planner/planPathsAdvancedSplitDist/line2_swap_k0_npe")
            continue
        if res is None:
            assert rid in ("planner/planPathsAdvancedSplitDist/line2_swap_k0_npe",), rid
            continue
        assert P.path_set_error(res, g, a["start"], a["goal"]) is None


HIGHS_FNS = ("PathPlanner.planPathsAdvanced", "PathPlanner.planPathsAdvancedPMG", "PathPlanner.planPathsAdvancedSplit",
             "PathPlanner.multiThreadedSplitPlanning", "PathPlanner.planPathsAdvancedTT",
             "PathPlanner.planPathsAdvancedTD", "PathPlanner.planPathsAdvancedSplitDist")
HIGHS_RECS = [r for r in SOLVE_RECS if r["fn"] in HIGHS_FNS and r.get("result") is not None
              and r["id"] not in FIXED_DIFFERS and "result" not in (r.get("nondet") or ())]


@pytest.mark.parametrize("rec", HIGHS_RECS, ids=lambda r: r["id"])
def test_golden_planner_fixed_highs_objectives(rec):
    """Under HiGHS: the same makespan / total time / distance as the Gurobi records, the
    same random draws, and valid plans."""
    pl = make_planner(rec, "fixed", backend="highs", params=None)
    if rec["id"] in SPLIT_ENDPOINT_DIFFERS:
        pl._split_endpoints = False  # the original's split goals (see test_golden_planner_fixed_gurobi)
    a = rec["args"]
    res, extra = call(pl, rec["fn"], a)
    g = graph_from_json(a["g"])
    want = rec["result"]
    assert P.path_set_error(res, g, a["start"], a["goal"]) is None
    assert len(res[0]) == len(want[0])
    m, mw = P.plan_metrics(res, a["goal"]), P.plan_metrics(want, a["goal"])
    if rec["fn"].endswith("TT"):
        assert extra == rec["mutated"] and m["total_time"] == mw["total_time"]
    if rec["fn"].endswith(("TD", "SplitDist")):
        assert m["total_distance"] == mw["total_distance"]
    if "draws" not in (rec.get("nondet") or ()):
        assert pl.rng.draws == rec["draws"]


# ------------------------------------------------------------- fixed corrections
@needs_gurobi
def test_mt_nonpositive_time_limit_fixed():
    g = create_2d_grid_graph(3, 3, True)
    for compat, want in (("original", None), ("fixed", 6)):
        pl = P.PathPlanner(backend="gurobi", params=GRB, rng=MathRandom(1), compat=compat)
        r = pl.multi_threaded_split_planning(g, [0, 2, 6], [8, 6, 1], -1, 1)
        assert (r if r is None else len(r[0]) - 1) == want


@needs_gurobi
def test_mt_stale_mpath_original_only():
    g = create_2d_grid_graph(3, 3, True)
    pl = P.PathPlanner(backend="gurobi", params=GRB, rng=MathRandom(1), compat="original")
    r1 = pl.multi_threaded_split_planning(g, [0, 2, 6], [8, 6, 1], 100, 1)
    r2 = pl.multi_threaded_split_planning(g, [0, 2, 6], [8, 6, 1], -1, 1)
    assert r2 == r1  # piece 0 failed (first horizon infeasible) and the stale one is reused
    pf = P.PathPlanner(backend="gurobi", params=GRB, rng=MathRandom(1), compat="fixed")
    f1 = pf.multi_threaded_split_planning(g, [0, 2, 6], [8, 6, 1], 100, 1)
    assert f1 == r1 and sorted(pf.mpath) == [0, 1]
    g2 = create_2d_grid_graph(2, 2, True)
    assert pf.multi_threaded_split_planning(g2, [0, 3], [3, 0], 100, 1) is not None
    assert pf.mpath[0][0][0] == 0  # this call's pieces only


def test_split_dist_no_limit_fixed():
    g = create_2d_grid_graph(3, 3, True)
    r = P.plan_paths_advanced_split_dist(g, [0, 2, 6], [8, 6, 1], 0.0, -1, 1, rng=MathRandom(1))
    assert r is not None and len(r[0]) - 1 == 6
    assert P.path_set_error(r, g, [0, 2, 6], [8, 6, 1]) is None
    ro = P.plan_paths_advanced_split_dist(g, [0, 2, 6], [8, 6, 1], 0.0, -1, 1, rng=MathRandom(1),
                                          compat="original")
    assert ro is None


def test_split_infeasible_piece_fixed_returns_none():
    """2x2 grid, 3 robots: the fallback middle makes a piece infeasible? Either way the
    fixed split never hangs and returns a valid plan or None."""
    g = create_2d_grid_graph(2, 2, True)
    pl = P.PathPlanner(rng=MathRandom(1))
    r = pl.plan_paths_advanced_split(g, [0, 2, 3], [3, 1, 2], False, -1, 1)
    if r is None:
        assert pl.failures
    else:
        assert P.path_set_error(r, g, [0, 2, 3], [3, 1, 2]) is None
    with pytest.raises(OriginalWouldHang):
        P.plan_paths_advanced_split(g, [0, 2, 3], [3, 1, 2], False, -1, 1, rng=MathRandom(1), compat="original")


def test_no_cycle_unbounded_retry_hang_detected():
    """Full 6-ring, robot i -> i+2: every plan is a rotation; the original's top-level
    retries never end (original-bugs B12)."""
    g = ring(6)
    s = list(range(6))
    t = [(i + 2) % 6 for i in s]
    pl = P.PathPlanner(rng=MathRandom(1), compat="original", retry_analysis_after=3)
    with pytest.raises(OriginalWouldHang):
        pl.plan_paths_advanced_split_no_cycle(g, s, t, True, -1, 1, True)
    # not at top level: bounded (27 retries), returns the rotation
    pn = P.PathPlanner(rng=MathRandom(1), compat="original")
    r = pn.plan_paths_advanced_split_no_cycle(g, s, t, True, -1, 1, False)
    assert r is not None and P.has_cycles(r) > 0
    # fixed: the same analysis stops the top-level loop (at retry 20), keeping the rotation
    pf = P.PathPlanner(rng=MathRandom(1))
    r = pf.plan_paths_advanced_split_no_cycle(g, s, t, True, -1, 1, True)
    assert P.path_set_error(r, g, s, t) is None and P.has_cycles(r) > 0
    assert len(pf.solves) == 2 + 2 * 19 and "no retry can give" in pf.failures[-1]
    # splits = 2: the top-level loop re-solves a half whose every outcome rotates
    pl2 = P.PathPlanner(rng=MathRandom(1), compat="original", retry_analysis_after=2)
    with pytest.raises(OriginalWouldHang):
        pl2.plan_paths_advanced_split_no_cycle(g, s, t, True, -1, 2, True)


def test_no_cycle_fixed_top_level_retries_beyond_original_tries():
    """Regression: at top level fixed retries past the original's ``3^(4-splits)``
    tries, as the original does; without a verdict of the endless-loop analysis
    (``analysis_budget=0`` disables it) it stops once the retries have spent
    ``max_retry_solves`` solves, or once a positive time limit has elapsed."""
    g = ring(6)
    s = list(range(6))
    t = [(i + 2) % 6 for i in s]
    pf = P.PathPlanner(rng=MathRandom(1), analysis_budget=0, max_retry_solves=80)
    r = pf.plan_paths_advanced_split_no_cycle(g, s, t, True, -1, 1, True)
    assert P.has_cycles(r) > 0 and "stopped after 80 solves" in pf.failures[-1]
    assert len(pf.solves) == 2 + 80
    # splits = 2 (robot i -> i+1): the top-level loop re-solves a half past the 9 tries
    t1 = [(i + 1) % 6 for i in s]
    pf = P.PathPlanner(rng=MathRandom(1), analysis_budget=0, max_retry_solves=600)
    r = pf.plan_paths_advanced_split_no_cycle(g, s, t1, True, -1, 2, True)
    assert P.path_set_error(r, g, s, t1) is None and P.has_cycles(r) > 0
    assert any("stopped after 600 solves" in f for f in pf.failures)
    assert 600 <= len(pf.solves) < 600 + 60
    # not at top level the original's bound applies
    pn = P.PathPlanner(rng=MathRandom(1), analysis_budget=0, max_retry_solves=10)
    pn.plan_paths_advanced_split_no_cycle(g, s, t, True, -1, 1, False)
    assert len(pn.solves) == 2 + 2 * 27
    # a positive time limit ends the retries once it has elapsed (after the 27 tries)
    tick = iter(range(0, 10**6, 100))
    pt = P.PathPlanner(rng=MathRandom(1), analysis_budget=0, max_retry_solves=None)
    pt._now = lambda: float(next(tick))
    pt.plan_paths_advanced_split_no_cycle(g, s, t, True, 10**4, 1, True)
    assert "time limit" in pt.failures[-1] and len(pt.solves) >= 2 + 2 * 28
    # split by size (no cycle): beyond its 5 tries at top level too
    pb = P.PathPlanner(rng=MathRandom(1), analysis_budget=0, max_retry_solves=30)
    r = pb.plan_paths_advanced_split_by_size_no_cycle(g, s, t, True, -1, 2, True)
    assert P.has_cycles(r) > 0 and "stopped after" in pb.failures[-1]


def test_no_cycle_fixed_stops_rotating_loops_quickly():
    """Regression ("fixed top-level loop has no practical budget"): the
    6-ring with robot i -> i+1 rotates in every plan. Fixed used to run 500 top-level
    retries (20856 solves at splits = 3, 1992 at splits = 2); the analysis, which starts
    at the first retry beyond the original's bounded tries (9 and 3), now shows at once
    that no retry gives a plan without rotations: 44 and 144 solves."""
    g = ring(6)
    s = list(range(6))
    t1 = [(i + 1) % 6 for i in s]
    for k, solves in ((2, 44), (3, 144)):
        pf = P.PathPlanner(rng=MathRandom(1))
        r = pf.plan_paths_advanced_split_no_cycle(g, s, t1, True, -1, k, True)
        assert P.path_set_error(r, g, s, t1) is None and P.has_cycles(r) > 0
        assert "no retry can give" in pf.failures[-1]
        assert len(pf.solves) == solves
    # the analysis's own solves (one per retry here) count toward max_retry_solves
    seen = []

    class Spy(P.PathPlanner):
        def _top_retry_stop(self, it, ok, run, watch):
            stop = super()._top_retry_stop(it, ok, run, watch)
            seen.append((ok, run.analysis, run.work(), len(self.solves) - run.solves0, stop))
            return stop

    pf = Spy(rng=MathRandom(1), analysis_budget=1, max_retry_solves=None)
    pf.plan_paths_advanced_split_no_cycle(g, s, t1, True, -1, 2, True)
    assert "no retry can give" in pf.failures[-1] and seen[-1][1] > 1
    assert all(w == a + n for _, a, w, n, _ in seen)
    ok, a, w, n, _ = next(x for x in seen if not x[0] and x[1] > 0)
    pc = Spy(rng=MathRandom(1), analysis_budget=1, max_retry_solves=w)
    pc.plan_paths_advanced_split_no_cycle(g, s, t1, True, -1, 2, True)
    assert f"stopped after {w} solves" in pc.failures[-1] and len(pc.solves) - 2 < w + 2


@pytest.mark.slow  # about 40 s (many builtin-backend solves)
def test_no_cycle_original_analysis_in_installments():
    """Regression ("hang detector gives up too early"): the analysis used
    to run once and give up when it needed more than ``analysis_budget`` leaf solves
    (the loop then spun forever, as Java does). It now continues at the following
    retries, keeping its leaves, until it decides. 2x3 grid, 5 robots: the random
    re-split has several outcomes, and every one rotates."""
    g = create_2d_grid_graph(2, 3, True)
    s, t = [4, 0, 5, 2, 3], [2, 1, 4, 3, 0]
    calls = []

    class Counting(P.PathPlanner):
        def _retry_check(self, watch):
            calls.append(watch.solves)
            super()._retry_check(watch)

    pl = Counting(rng=MathRandom(1), compat="original", backend="builtin", retry_analysis_after=1, analysis_budget=1)
    with pytest.raises(OriginalWouldHang, match="top-level retry loop"):
        pl.plan_paths_advanced_split_no_cycle(g, s, t, True, -1, 1, True)
    assert calls == [0, 1, 2, 3, 4, 5] and len(pl.solves) == 12
    # one installment is enough with the default budget
    calls.clear()
    p1 = Counting(rng=MathRandom(1), compat="original", backend="builtin", retry_analysis_after=1)
    with pytest.raises(OriginalWouldHang, match="top-level retry loop"):
        p1.plan_paths_advanced_split_no_cycle(g, s, t, True, -1, 1, True)
    assert calls == [0] and len(p1.solves) == 2
    # without any budget the analysis cannot progress and gives up for good (the loop
    # then continues, as in Java; here capped by the counting hook)
    class Capped(Counting):
        def _retry_check(self, watch):
            super()._retry_check(watch)
            if len(calls) >= 50:
                assert watch.gave_up and watch.verdict is None
                raise _Capped()

    calls.clear()
    p0 = Capped(rng=MathRandom(1), compat="original", backend="builtin", retry_analysis_after=1, analysis_budget=0)
    with pytest.raises(_Capped):
        p0.plan_paths_advanced_split_no_cycle(g, s, t, True, -1, 1, True)
    # fixed: the same verdict ends its retries with the rotating plan
    pf = P.PathPlanner(rng=MathRandom(1), backend="builtin", retry_analysis_after=1, analysis_budget=1)
    r = pf.plan_paths_advanced_split_no_cycle(g, s, t, True, -1, 1, True)
    assert P.path_set_error(r, g, s, t) is None and P.has_cycles(r) > 0
    assert "no retry can give" in pf.failures[-1]


@pytest.mark.gurobi
@pytest.mark.slow
@needs_gurobi
@pytest.mark.parametrize("seed, starts, goals", [
    (9736, [4, 0, 1, 6, 3, 7, 8, 2], [3, 7, 2, 0, 4, 6, 1, 8]),
    (27536, [2, 4, 5, 0, 7, 6, 3, 1], [1, 8, 2, 3, 5, 4, 6, 7]),
])
def test_no_cycle_dense_8_puzzle_hang_detected(seed, starts, goals):
    """Regression: 3x3 grid, 8 robots, ``splits = 1``, top level, seeds 9736 and
    27536: live Java (Threads 1) is still retrying after 300 s. The random re-split has
    182 outcomes (364 leaves), more than one installment of 200: the original used to
    give up and spin forever; now it raises at retry 21. Fixed stops at the same point
    with the rotating plan (it used to run 500 retries, 1002 solves)."""
    g = create_2d_grid_graph(3, 3, True, compat="original")
    pl = P.PathPlanner(backend="gurobi", params=GRB, rng=MathRandom(seed), compat="original")
    with pytest.raises(OriginalWouldHang):
        pl.plan_paths_advanced_split_no_cycle(g, starts, goals, True, -1, 1, True)
    assert len(pl.solves) == 2 + 2 * 20
    for ep in (False, True):
        pf = P.PathPlanner(backend="gurobi", params=GRB, rng=MathRandom(seed))
        pf._split_endpoints = ep
        r = pf.plan_paths_advanced_split_no_cycle(g, starts, goals, True, -1, 1, True)
        assert P.path_set_error(r, g, starts, goals) is None and P.has_cycles(r) > 0
        assert "no retry can give" in pf.failures[-1] and len(pf.solves) <= 2 + 2 * 21


@pytest.mark.gurobi
@needs_gurobi
def test_no_cycle_fixed_equals_original_when_its_retry_loop_ends():
    """Regression: the 8-puzzle of ``create_n2m1_puzzle(3, MathRandom(1))``; the
    original (and live Java, ``D5 1 1 -1`` with Threads 1) needs 39 top-level retries:
    78 leaf solves, makespan 6, no rotations. Fixed, with the original's split rule,
    gives the same plan and draws; with endpoint goals it needs no retry loop that long."""
    from mppflow.problem import create_n2m1_puzzle

    out = {}
    for compat, ep in (("original", False), ("fixed", False), ("fixed", True)):
        rng = MathRandom(1)
        p = create_n2m1_puzzle(3, rng, compat="original")
        pl = P.PathPlanner(backend="gurobi", params=GRB, rng=rng, compat=compat)
        pl._split_endpoints = ep
        r = pl.plan_paths_advanced_split_no_cycle(p.graph, list(p.starts), list(p.goals), True, -1, 1, True)
        assert P.path_set_error(r, p.graph, p.starts, p.goals) is None and P.has_cycles(r) == 0
        out[compat, ep] = (r, len(pl.solves), rng.draws)
    assert out["original", False][1] == 78 and len(out["original", False][0][0]) - 1 == 6
    assert out["fixed", False] == out["original", False]
    assert len(out["fixed", True][0][0]) - 1 <= 6


def test_no_cycle_retry_cache_keeps_draws():
    """The leaf cache of the original retry loop consumes the same draws as solving."""
    g = ring(6)
    s = list(range(6))
    t = [(i + 2) % 6 for i in s]
    a = P.PathPlanner(rng=MathRandom(5), compat="original")
    ra = a.plan_paths_advanced_split_no_cycle(g, s, t, True, -1, 1, False)
    ref = P.PathPlanner(rng=MathRandom(5), compat="original")
    # the same calls without cache: a non-top call outside a scope with leaf cache
    ref._depth = 1  # disable the scope's cache creation
    ref._wcache = {}
    rb = ref._nc_orig(g, s, t, True, -1.0, 1, False)
    assert ra == rb and a.rng.draws == ref.rng.draws


def test_no_cycle_printpaths_leak():
    g = create_2d_grid_graph(3, 3, True)
    for compat, after in (("original", False), ("fixed", True)):
        pl = P.PathPlanner(rng=MathRandom(1), compat=compat, flags=P.SolverFlags(time_print_paths=True))
        pl.plan_paths_advanced_split_no_cycle(g, [0, 2], [8, 6], True, -1, 0, True)
        assert pl.flags.time_print_paths is after


def test_split_by_size_window_hang_original():
    """size 1: the original splitPathBySize(firstSize=1) never finds a vertex (its window
    [0, 0] is empty and never changes), so it loops forever; fixed solves the instance."""
    g = create_2d_grid_graph(3, 3, True)
    with pytest.raises(OriginalWouldHang, match="no free vertex"):
        P.plan_paths_advanced_split_by_size(g, [0], [2], True, -1, 1, rng=MathRandom(1), compat="original")
    r = P.plan_paths_advanced_split_by_size(g, [0], [2], True, -1, 1, rng=MathRandom(1))
    assert r is not None and P.path_set_error(r, g, [0], [2]) is None


@pytest.mark.parametrize("no_cycle", [False, True], ids=["by_size", "by_size_no_cycle"])
def test_split_by_size_stack_overflow_original(no_cycle):
    """size 3: a middle steps sideways and a later middle steps back to an earlier start, so
    a sub-instance repeats on the recursion stack; the split is deterministic (``random =
    false``), so the original recurses until the stack overflows. fixed solves the repeated
    sub-instance directly."""
    rows, cols, s, t, seed = (4, 4, [5], [14], 13) if no_cycle else (5, 4, [0], [15], 8)
    g = create_2d_grid_graph(rows, cols, True)
    f = P.plan_paths_advanced_split_by_size_no_cycle if no_cycle else P.plan_paths_advanced_split_by_size
    with pytest.raises(JavaStackOverflowError, match="recurses forever on the same sub-instance"):
        f(g, s, t, True, -1, 3, rng=MathRandom(seed), compat="original")
    r = f(g, s, t, True, -1, 3, rng=MathRandom(seed))
    assert r is not None and P.path_set_error(r, g, s, t) is None
    assert len(r[0]) - 1 == P.get_longest_path_length(g, s, t) - 1  # one robot: a shortest path



def test_split_by_size_no_cycle_retries_shrink():
    """Regression (C1B3-BYSIZE-NOCYCLE-RECURSIONERROR): each retry re-splits with a random
    split_paths, whose halves need not be shorter than their parent; fixed only stopped exact
    (s, goal) repeats, so new middles kept the descent going (depth 100 after 90 s) until a
    plain RecursionError. A half must now shorten the longest path to split again: the depth
    is at most L, and the call returns a valid plan."""
    p = create_grid_problem(3, 5, 0.2, 7, MathRandom(842709))
    pl = P.PathPlanner(rng=MathRandom(842709))
    L = pl.get_longest_path_length(p.graph, p.starts, p.goals)
    inner, depth = pl._bsnc, [0, 0]

    def counted(*a):
        depth[0] += 1
        depth[1] = max(depth)
        try:
            return inner(*a)
        finally:
            depth[0] -= 1
    pl._bsnc = counted
    r = pl.plan_paths_advanced_split_by_size_no_cycle(p.graph, p.starts, p.goals, True, -1.0, 2)
    assert r is not None and P.path_set_error(r, p.graph, p.starts, p.goals) is None
    assert depth[1] <= L

def test_longest_path_off_by_one():
    g = from_edges(8, [(i, i + 1) for i in range(7)])
    assert P.get_longest_path_length(g, [0, 1], [4, 6], rng=MathRandom(1), compat="original") == 5
    assert P.get_longest_path_length(g, [0, 1], [4, 6], rng=MathRandom(1)) == 6


# ------------------------------------------------------------- paper examples
def _fixture(name):
    # published with the repo (tests/data/paper); a missing file is an error, not a skip
    with open(os.path.join(FIXTURES, name), encoding="utf-8") as f:
        return json.load(f)


PAPER_BACKENDS = ["highs", "gurobi"]
"""The backends of the paper-example tests; the ``gurobi`` cases get the ``gurobi`` marker and
skip without a full-size license (tests/conftest.py)."""


@pytest.mark.parametrize("backend", PAPER_BACKENDS)
@pytest.mark.parametrize("compat", ["fixed", "original"])
def test_fig8_split(compat, backend):
    """T-RO Fig. 8: intermediate goals (1,1), (2,2) = [5, 10]; the halves need 3 + 2 steps
    (the paper claims 2 + 2), the unsplit optimum is 4."""
    fx = _fixture("tor16_fig8_split.json")
    g = create_2d_grid_graph(4, 4, True)
    s, t = fx["starts"], fx["goals"]
    params = GRB if backend == "gurobi" else None
    pl = P.PathPlanner(backend=backend, params=params, rng=MathRandom(1), compat=compat)
    r = pl.multi_threaded_split_planning(g, s, t, 1e6, 1)
    assert pl.msg[1] == fx["intermediate_goals"] == [5, 10]
    assert [len(pl.mpath[i][0]) - 1 for i in (0, 1)] == [fx["expected"]["child1_optimal_makespan"],
                                                          fx["expected"]["child2_optimal_makespan"]]
    assert len(r[0]) - 1 == fx["expected"]["two_way_split_total_makespan"] == 5
    assert P.path_set_error(r, g, s, t) is None
    res = P.plan(Problem(g, s, t), compat=compat, backend=backend, params=params, rng=MathRandom(1))
    assert res.makespan == fx["expected"]["optimal_makespan_unsplit"] == 4
    res2 = P.plan(Problem(g, s, t), split=2, compat=compat, backend=backend, params=params, rng=MathRandom(1),
                  time_limit=1e6)
    assert res2.makespan == 5 and res2.intermediate_goals == [[5, 10]] and res2.status == "feasible"


def test_plan_original_split_without_limit_fails():
    """The original's k-way split always passes hasTimeLimit=true: without a limit every
    piece that is infeasible at its first horizon fails (original-bugs B14)."""
    g = create_2d_grid_graph(4, 4, True)
    r = P.plan((g, [13, 8], [0, 7]), split=2, compat="original", rng=MathRandom(1))
    assert r.paths is None and r.status == "time_limit"
    assert P.plan((g, [13, 8], [0, 7]), split=2, rng=MathRandom(1)).makespan == 5


@pytest.mark.parametrize("backend", PAPER_BACKENDS)
def test_fig3_9puzzle(backend):
    """T-RO Fig. 1/3: the 9-puzzle has optimal makespan 4; under Gurobi the original's plan
    is exactly Fig. 3 (4 rotations)."""
    fx = _fixture("tor16_fig1_fig3_9puzzle.json")
    g = create_2d_grid_graph(3, 3, True)
    s, t = fx["starts"], fx["goals"]
    params = GRB if backend == "gurobi" else None
    res = P.plan(Problem(g, s, t), backend=backend, params=params, rng=MathRandom(1))
    assert res.makespan == fx["expected"]["optimal_makespan"] == 4
    assert res.status == "optimal" and res.lower_bounds["makespan"] == 4
    assert P.path_set_error(res.paths, g, s, t) is None
    if backend == "gurobi":
        java = _fixture("tor16_java_original_outputs.json")["fig1_fig3_9puzzle"]
        assert res.paths == java["paths"]
        assert P.has_cycles(res.paths) == java["hasCycles"] == 4
        assert res.total_distance == java["total_distance"] == 24


def test_fig14_25puzzle_plan_checks():
    """T-RO Fig. 14/15: the original's 7-step 25-puzzle plan is valid, has 19 rotations and
    distance 144 (the paper's Fig. 15 plan: 134); 7 is also the lower bound."""
    java = _fixture("tor16_java_original_outputs.json")["fig14_fig15_25puzzle"]
    fx = _fixture("tor16_fig14_fig15_25puzzle.json")
    g = create_2d_grid_graph(5, 5, True)
    paths = java["paths"]
    assert P.is_path_set_valid(paths, g, java["starts"], java["goals"])
    assert P.is_path_set_valid(paths, g, java["starts"], java["goals"], "original")
    assert len(paths[0]) - 1 == java["makespan"] == fx["expected"]["optimal_makespan"] == 7
    assert P.has_cycles(paths) == java["hasCycles"] == 19
    assert P.plan_metrics(paths, java["goals"])["total_distance"] == java["total_distance"] == 144
    assert P._lower_bounds(g, fx["starts"], fx["goals"])["makespan"] == 7


@needs_gurobi
def test_fig14_25puzzle_split():
    fx = _fixture("tor16_fig14_fig15_25puzzle.json")
    g = create_2d_grid_graph(5, 5, True)
    res = P.plan(Problem(g, fx["starts"], fx["goals"]), split=2, backend="gurobi", params={"Threads": 4},
                 rng=MathRandom(1))
    assert res.paths is not None and res.makespan >= 7 and res.status == "feasible"
    assert P.path_set_error(res.paths, g, fx["starts"], fx["goals"]) is None


@pytest.mark.slow
@needs_gurobi
def test_fig14_25puzzle_exact():
    """T-RO Sec. VI-B: the Fig. 14 instance needs 7 steps (about 12 s with 8 threads)."""
    fx = _fixture("tor16_fig14_fig15_25puzzle.json")
    g = create_2d_grid_graph(5, 5, True)
    res = P.plan(Problem(g, fx["starts"], fx["goals"]), backend="gurobi", params={"Threads": 8},
                 rng=MathRandom(1), solve_relaxed_first=False)
    assert res.makespan == fx["expected"]["optimal_makespan"] == 7 and res.status == "optimal"
    assert res.stats["horizons"] == [7]
    assert P.path_set_error(res.paths, g, fx["starts"], fx["goals"]) is None


# ------------------------------------------------------------- high-level API
@pytest.mark.parametrize("backend", ["highs", "builtin"])
def test_plan_objectives_small(backend):
    g = from_edges(4, [(0, 1), (1, 2), (1, 3)])  # Main case 3's star
    prob = Problem(g, [0, 3], [3, 0])
    r = P.plan(prob, backend=backend, rng=MathRandom(1))
    assert r.makespan == 4 and r.status == "optimal" and r.lower_bounds["makespan"] == 2
    tt = P.plan(prob, "total_time", backend=backend, rng=MathRandom(1))
    # arrivals 3 + 4 (one robot waits while the other dodges into the leaf 2)
    assert tt.status == "optimal" and tt.value == tt.total_time == 7
    td = P.plan(prob, "total_distance", backend=backend, rng=MathRandom(1))
    assert td.value == 6 and td.lower_bounds["total_distance"] == 4
    md = P.plan(prob, "max_distance", backend=backend, rng=MathRandom(1))
    assert md.value == 4  # one robot must dodge into the leaf 2: 4 moves
    for res in (r, tt, td, md):
        assert P.path_set_error(res.paths, g, [0, 3], [3, 0]) is None
        assert res.stats["solves"] >= 1


def test_plan_split_and_errors():
    g = create_2d_grid_graph(3, 3, True)
    prob = (g, [0, 2, 6], [8, 6, 1])
    r = P.plan(prob, split=2, rng=MathRandom(1))
    assert r.makespan == 6 and r.status == "feasible" and r.intermediate_goals == [[2, 0, 3]]
    r4 = P.plan(prob, split=4, rng=MathRandom(1))
    assert r4.makespan == 8 and len(r4.intermediate_goals) == 3
    d = P.plan(prob, "total_distance", split=2, rng=MathRandom(1))
    assert d.paths is not None and d.makespan == 6
    # a total-distance split reports its intermediate configurations too, in time order, and the
    # plan passes through each one (where its pieces meet)
    d4 = P.plan(prob, "total_distance", split=4, rng=MathRandom(1))
    assert len(d.intermediate_goals) == 1 and len(d4.intermediate_goals) == 3
    for res in (d, d4):
        t = 0
        for cfg in res.intermediate_goals:
            while [p[min(t, len(p) - 1)] for p in res.paths] != cfg:
                t += 1
                assert t < len(res.paths[0]), (cfg, res.paths)
    with pytest.raises(ValueError):
        P.plan(prob, split=3)
    with pytest.raises(ValueError):
        P.plan(prob, "total_time", split=2)
    with pytest.raises(NotInOriginal):
        P.plan(prob, "max_distance", compat="original")


@pytest.mark.parametrize("compat", ["fixed", "original"])
def test_plan_rejects_a_huge_split_at_once(compat, monkeypatch):
    """Regression (TS1A-HUGE-SPLIT-OOM): split = 2**40 (a typo such as 1 << 40) allocated 2**40
    configurations until the OOM killer stopped the process; above MAX_SPLIT it raises at once,
    before any solve, in both modes (and so does the split method itself)."""
    prob = create_grid_problem(4, 4, 0.0, 4, MathRandom(2))
    monkeypatch.setattr(P.PathPlanner, "_mt", lambda *a, **k: pytest.fail("split started"))
    for k in (2 * P.MAX_SPLIT, 2 ** 40):
        with pytest.raises(ValueError, match="at most 1024"):
            P.plan(prob, split=k, compat=compat)
        with pytest.raises(ValueError, match="at most 1024"):
            P.plan(prob, "total_distance", split=k, compat=compat)
    monkeypatch.undo()
    pl = P.PathPlanner(rng=MathRandom(1), compat=compat)
    # past the guard the split computes 2**splits and allocates that many configurations: a
    # guard that no longer raises must fail this test, not run into the OOM killer
    with monkeypatch.context() as m:
        m.setattr(P, "_jpow_int", lambda *a: pytest.fail("split started"))
        for splits in (P.MAX_SPLIT.bit_length(), 40):  # 11, the smallest rejected value
            with pytest.raises(ValueError, match="at most 1024 pieces"):
                pl.multi_threaded_split_planning(prob.graph, prob.starts, prob.goals, -1.0, splits)
    assert P.plan(prob, split=P.MAX_SPLIT, rng=MathRandom(2)).makespan is not None  # the cap itself runs


@needs_gurobi
def test_plan_original_total_time_uses_split_horizon():
    g = create_2d_grid_graph(3, 3, True)
    prob = (g, [0, 2, 6], [8, 6, 1])
    r = P.plan(prob, "total_time", compat="original", backend="gurobi", params=GRB, rng=MathRandom(1))
    # the value meets the lower bound, so the single horizon is optimal (it said "feasible")
    assert r.extra["horizon"] == 8 and r.total_time == 11 == r.lower_bounds["total_time"]
    assert r.status == "optimal" and r.makespan == 4
    rf = P.plan(prob, "total_time", backend="gurobi", params=GRB, rng=MathRandom(1))
    assert rf.total_time == 11 and rf.status == "optimal"


def test_plan_infeasible_fixed():
    g = from_edges(2, [(0, 1)])
    with pytest.raises(Infeasible):
        P.plan((g, [0, 1], [1, 0]))


def test_path_planner_main_with_stub():
    def stub(g, s, t):
        return [[0, 1, 2, 3, 4, 9, 4, 5, 6, 7, 8, 8], [8, 7, 6, 5, 5, 4, 3, 4, 3, 2, 1, 0]]

    out = P.path_planner_main(heuristic=stub)
    assert out == ("Start and goal vertices: 2 pairs\n0 8  \n8 0  \n"
                   "Agent 0: 0:0 1:1 2:2 3:3 4:4 5:9 6:4 7:5 8:6 9:7 10:8 11:8\n"
                   "Agent 1: 0:8 1:7 2:6 3:5 4:5 5:4 6:3 7:4 8:3 9:2 10:1 11:0\n"
                   "Path set is valid.\n")


def test_path_planner_main_prints_invalid_message():
    """Regression: an invalid plan prints isPathSetValid's message, as Java does."""
    for compat in ("fixed", "original"):
        out = P.path_planner_main(heuristic=lambda g, s, t: [[0, 1], [8, 7]], compat=compat)
        assert out == ("Start and goal vertices: 2 pairs\n0 8  \n8 0  \nAgent 0: 0:0 1:1\nAgent 1: 0:8 1:7\n"
                       "Goal location for agent 0 is incorrect in the path set.\n")


def test_path_planner_main_default_heuristic_fixed():
    """Regression: without ``heuristic=`` the local-repair heuristic of
    :mod:`mppflow.heuristic` runs (it used to raise NotImplementedError)."""
    out = P.path_planner_main(rng=MathRandom(9))
    assert out.startswith("Start and goal vertices: 2 pairs\n0 8  \n8 0  \nAttempting with 3.0 seconds...\n")
    assert out.endswith("Path set is valid.\n")


@pytest.mark.gurobi
@needs_gurobi
def test_path_planner_main_default_heuristic_golden():
    """``PathPlanner.main`` against the live Java record (heuristic.jsonl, seed 9)."""
    with open(os.path.join(GOLDEN, "heuristic.jsonl"), encoding="utf-8") as f:
        rec = next(r for r in map(json.loads, f) if r["id"] == "heuristic/pp/main")
    rng = MathRandom(rec["seed"])
    out = P.path_planner_main(rng=rng, compat="original", backend="gurobi", params=GRB)
    assert norm(out) == rec["stdout"]
    assert rng.draws == rec["draws"] and rng.next_double() == rec["rng_next"]


def test_module_wrappers():
    g = create_2d_grid_graph(3, 3, True)
    r = P.plan_paths_advanced(g, [0, 2, 6], [8, 6, 1], rng=MathRandom(1))
    assert len(r[0]) - 1 == 4
    r = P.multi_threaded_split_planning(g, [0, 2, 6], [8, 6, 1], 100.0, 1, rng=MathRandom(1))
    assert len(r[0]) - 1 == 6


class _Capped(Exception):
    pass


class _CapPlanner(P.PathPlanner):
    """Analyse from the first top-level retry on (if ``analyse``) and stop at the 60th
    analysed retry."""

    analyse = True

    def _retry_check(self, watch):
        self.checks = getattr(self, "checks", 0) + 1
        if self.analyse:
            super()._retry_check(watch)
        if self.checks >= 60:
            raise _Capped()


def _capped_run(g, s, t, seed, analyse, kind, k):
    pl = _CapPlanner(rng=MathRandom(seed), compat="original", retry_analysis_after=1, backend="gurobi", params=GRB)
    pl.analyse = analyse
    try:
        if kind == "nc":
            r = pl.plan_paths_advanced_split_no_cycle(g, s, t, True, -1, k, True)
        else:
            r = pl.plan_paths_advanced_split_by_size_no_cycle(g, s, t, True, -1, k, True)
        return ("ok", r, pl.rng.draws)
    except _Capped:
        return ("capped", None, None)
    except OriginalWouldHang as e:
        return ("retry-hang" if "top-level retry" in str(e) else "hang", None, pl.rng.draws)
    except JavaException as e:
        return ("exc", type(e).__name__, pl.rng.draws)


@pytest.mark.gurobi
@needs_gurobi
def test_retry_analysis_is_sound_on_random_instances():
    """The endless-retry verdict is only given where 60 plain retries do not finish, and
    the analysis (with its leaf cache) never changes a result or the random draws."""
    import random

    rnd = random.Random(1)
    verdicts = 0
    for _ in range(120):
        R, C = rnd.choice([2, 3]), rnd.choice([2, 3])
        g = create_2d_grid_graph(R, C, True)
        n = rnd.randint(2, R * C - 1)
        s, t = rnd.sample(range(R * C), n), rnd.sample(range(R * C), n)
        kind = rnd.choice(["nc", "nc", "bsnc"])
        k = rnd.choice([1, 2]) if kind == "nc" else rnd.choice([2, 3])
        seed = rnd.randint(0, 1000)
        a = _capped_run(g, s, t, seed, True, kind, k)
        b = _capped_run(g, s, t, seed, False, kind, k)
        if a[0] == "retry-hang":
            verdicts += 1
            assert b[0] == "capped"
        elif not (a[0] == b[0] == "capped"):
            assert a == b
    assert verdicts > 0


# --------------------------------------------- plan(): horizon option and statuses
def test_plan_single_horizon_status_and_max_distance():
    """horizon="single" solves at the optimal makespan; the plan is optimal there only, so the
    status is "feasible" unless it meets the lower bound (it used to say "optimal": 14 here,
    while the horizon search finds 13). max_distance accepts horizon="single" too (it used to
    raise "horizon='single' needs time_steps")."""
    from mppflow.problem import create_grid_problem
    p = create_grid_problem(3, 4, 0.0, 6, MathRandom(35))
    a = P.plan(p, "total_time", horizon="single")
    b = P.plan(p, "total_time")
    assert (a.status, a.value, a.extra["horizon"]) == ("feasible", 14, 3)
    assert (b.status, b.value) == ("optimal", 13)
    # the horizon search solves several horizons and records none (API.md, PlanResult)
    assert "horizon" not in b.extra and b.makespan == 4 and 4 not in b.stats["horizons"]
    m = P.plan(p, "max_distance", horizon="single")
    assert m.paths is not None and m.extra["horizon"] == 3 and m.status in ("optimal", "feasible")
    assert m.value >= P.plan(p, "max_distance").value
    for obj in ("makespan", "total_time", "max_distance"):
        with pytest.raises(ValueError, match="horizon"):
            P.plan(p, obj, horizon="bogus")
    with pytest.raises(ValueError, match="horizon"):
        P.plan(p, "total_distance", split=2, horizon="bogus")


def test_plan_without_a_plan_reports_the_time_limit():
    """A budget that runs out before (or during) the first solve: no plan and status
    "time_limit" for every objective and both horizon options (the horizon search used to
    say "failed" when the budget was gone before its first model)."""
    from mppflow.problem import create_grid_problem
    p = create_grid_problem(3, 4, 0.0, 6, MathRandom(35))
    for obj in ("makespan", "total_time", "total_distance", "max_distance"):
        for hz in ("single", None):
            r = P.plan(p, obj, horizon=hz, time_limit=1e-9)
            assert (r.paths, r.status, r.value, r.makespan) == (None, "time_limit", None, None), (obj, hz)
            assert r.lower_bounds[obj] is not None


def test_lower_bounds_with_an_unreachable_goal():
    g = from_edges(4, [(0, 1), (2, 3)])
    assert P._lower_bounds(g, [0, 2], [1, 0]) == dict.fromkeys(P._OBJECTIVES)
    assert P._lower_bounds(g, [0, 2], [1, 3]) == {"makespan": 1, "total_time": 2, "total_distance": 2,
                                                  "max_distance": 1}
    assert P._lower_bounds(g, [], []) == dict.fromkeys(P._OBJECTIVES, 0)


def test_plan_metrics_rejects_malformed_plans():
    assert P.plan_metrics([], [])["makespan"] == 0
    for paths, goals in ((None, [3]), ([[], [1, 2]], [0, 2]), ([[0, 3]], [3, 2]), ([[0, 3], [1, 2, 2]], [3, 2])):
        with pytest.raises(ValueError):
            P.plan_metrics(paths, goals)


def test_plan_stats_phases():
    """stats["phases"] labels every horizon of stats["horizons"]: the LP relaxations that the
    makespan solve tries first, then the ILPs (the demo marks the LP horizons with them), and
    ("joint") a makespan plan that the builtin loop took from its joint-state search, at a
    horizon where no model was solved (not counted in "solves")."""
    g = from_edges(4, [(0, 1), (1, 2), (1, 3)])  # Main case 3's star
    prob = Problem(g, [0, 3], [3, 0])
    for objective, split in (("makespan", 1), ("total_time", 1), ("makespan", 2)):
        st = P.plan(prob, objective, split, backend="builtin", rng=MathRandom(1)).stats
        assert len(st["phases"]) == len(st["horizons"])
        assert set(st["phases"]) <= {"lp", "ilp", "joint"}
        assert st["phases"].count("lp") == st["lp_solves"] and st["phases"].count("ilp") == st["ilp_solves"]
        assert st["solves"] == len(st["horizons"]) - st["phases"].count("joint")
        if objective == "makespan" and split == 1:
            assert st["phases"][0] == "lp"   # solve_relaxed_first
        if split > 1:
            assert "lp" not in st["phases"]
    for backend in ("highs", "builtin"):
        st = P.plan(prob, "makespan", backend=backend, rng=MathRandom(1)).stats
        if backend == "highs":  # HiGHS (and Gurobi) solve every horizon: no joint-state plan
            assert st["horizons"] == [2, 3, 3, 4] and "joint" not in st["phases"]
        else:  # the LP at T = 2 fails, and the joint-state search's plan at T = 4 is returned
            assert (st["horizons"], st["phases"], st["solves"]) == ([2, 4], ["lp", "joint"], 1)


def test_plan_stats_record_joint_state_pieces():
    """A split piece taken from the joint-state search is listed at its own horizon, so the
    horizons account for the plan: 3x3 grid, [2, 0] -> [3, 5], 2-way split (DEMO5 finding:
    stats["horizons"] used to be [2, 2] for pieces of 3 + 2 steps)."""
    r = P.plan(Problem(create_2d_grid_graph(3, 3), [2, 0], [3, 5]), "makespan", split=2, backend="builtin",
               rng=MathRandom(1))
    st = r.stats
    assert r.value == 5 and st["solves"] == 2
    assert list(zip(st["horizons"], st["phases"])) == [(2, "ilp"), (3, "joint"), (2, "ilp")]


def test_split_goal_side_measured_to_the_goal_on_one_way_graphs():
    """fixed: the goal side of the split window is dist(v, goal) (reversed arcs). The forward
    BFS from the goal offered the sink 3 (dist(2, 3) = 1 forward, but 3 reaches nothing), so
    the split failed on a solvable instance, and with k = 4 plan() raised Infeasible."""
    g = Graph.from_adjacency([[2, 3], [0, 2], [1, 3], []])
    for k in (2, 4):
        r = P.plan((g, [0], [2]), "makespan", k)
        assert r.status == "feasible" and r.paths == [[0, 2]]
    g2 = Graph.from_adjacency([[1, 3], [0, 2, 3], [], [2]])
    s, t = [0, 1, 3], [1, 3, 2]
    assert P.plan((g2, s, t), "makespan").makespan == 1
    for obj in ("makespan", "total_distance"):
        for k in (2, 4):
            r = P.plan((g2, s, t), obj, k)
            assert r.status == "feasible", (obj, k)
            assert P.path_set_error(r.paths, g2, s, t) is None


def test_split_heuristic_failure_is_failed_not_infeasible():
    """fixed: when the split finds no intermediate goal (_split_fallback) the split variants
    record a failure and return None (status "failed"); only a top-level unreachable goal raises."""
    g = Graph.from_adjacency([[1], [0, 2], [1]])
    pl = P.PathPlanner()

    def stuck(*a, **k):
        raise P._SplitStuck("no free vertex can serve as the intermediate goal of robot 1")

    pl.split_paths = stuck
    assert pl.multi_threaded_split_planning(g, [0], [2], -1.0, 2) is None
    assert pl.msg is None and pl.failures == ["split [0] -> [2]: no free vertex can serve as the intermediate "
                                              "goal of robot 1"]
    assert pl.plan_paths_advanced_split(g, [0], [2], True, -1.0, 1) is None
    assert pl.plan_paths_advanced_split_dist(g, [0], [2], 0.0, -1.0, 1) is None
    with pytest.raises(Infeasible, match="unreachable"):
        P.PathPlanner().multi_threaded_split_planning(Graph.from_adjacency([[1], [], [1]]), [0], [2], -1.0, 1)


def test_split_takes_the_middle_of_an_odd_shortest_path():
    """T-RO Sec. V-B: the middle node of the shortest path. On a non-bipartite graph the
    original's window d = ceil(D/2) from both ends holds only vertices off every shortest
    path, so a lone robot got a one-step detour (8-connected 3x4 grid, 0 -> 3: 4 instead of 3)."""
    from mppflow.graph import create_2d_grid_graph_8connected
    g = create_2d_grid_graph_8connected(3, 4)
    r = P.plan((g, [0], [3]), "makespan", 2)
    assert r.makespan == 3 and r.intermediate_goals in ([[1]], [[2]])
    assert P.plan((g, [0], [3]), "total_distance", 2).value == 3
    pl = P.PathPlanner(rng=MathRandom(1))
    pl._split_endpoints = False  # the original's rule
    assert pl.split_paths(g, [0], [3]) not in ([1], [2])


def test_split_runs_the_instance_checks_first():
    """fixed: plan() with split > 1 raises Infeasible for an infeasible instance, as split = 1
    does, so "failed" means a failure of the heuristic."""
    from mppflow.graph import convert_graph
    g = convert_graph(create_2d_grid_graph(2, 3), [], [])
    p3 = Graph.from_adjacency([[1], [0, 2], [1]])
    with pytest.raises(Infeasible, match="share a goal"):
        P.plan((g, [0, 1], [2, 2]), "makespan", 2)
    for obj, k in (("makespan", 4), ("total_distance", 2)):
        with pytest.raises(Infeasible, match="no solution for any horizon"):
            P.plan((p3, [0, 2], [2, 0]), obj, k)
    with pytest.raises(ValueError, match="share a start"):
        P.plan((p3, [0, 0], [1, 2]), "makespan", 2)


def test_plan_original_rejects_horizon_search():
    """plan(compat="original", horizon="search") raises NotInOriginal, as the solvers do."""
    g = create_2d_grid_graph(2, 2, True)
    for obj in ("total_time", "total_distance"):
        with pytest.raises(NotInOriginal):
            P.plan((g, [0], [3]), obj, compat="original", horizon="search")


# ------------------------------------------- failure branches (stubbed solver / clock)
def test_original_grb_exception_is_printed_and_returns_none(monkeypatch):
    """original planPathsAdvanced* catch the solver's GRBException (here: the LP relaxation
    of solve_time has no solution, so ObjVal cannot be read), print it to stderr and return
    null; fixed never raises it, and a GRBException reaching a fixed planner propagates."""
    from mppflow import lp, solvers as S
    from mppflow.errors import JavaGRBException

    monkeypatch.setattr(S, "_run", lambda m, backend, params, tl, gap: lp.Solution(status="time_limit",
                                                                                   backend=backend))
    g = create_2d_grid_graph(3, 3)
    pl = P.PathPlanner(compat="original")
    assert pl.plan_paths_advanced(g, [0, 1], [1, 0], True, 0, True, 1.0) is None
    assert pl.stderr == "com.gurobi.gurobi.GRBException: Unable to retrieve attribute 'ObjVal'\n"
    assert P.PathPlanner().plan_paths_advanced(g, [0, 1], [1, 0], True, 0, True, 1.0) is None

    def grb(*a, **k):
        raise JavaGRBException("Unable to retrieve attribute 'ObjVal'")

    monkeypatch.setattr(P, "solve_total_time", grb)  # the Java catch blocks of TT and TD
    monkeypatch.setattr(P, "solve_distance", grb)
    pl = P.PathPlanner(compat="original")
    assert pl.plan_paths_advanced_tt(g, [0, 1], [1, 0], None, 0.0, 3, 1.0) is None
    assert pl.plan_paths_advanced_td(g, [0, 1], [1, 0], 0.0, 3, 1.0) is None
    assert pl.stderr.count("GRBException: Unable to retrieve attribute 'ObjVal'") == 2
    with pytest.raises(JavaGRBException):
        P.PathPlanner().plan_paths_advanced_tt(g, [0, 1], [1, 0], None, 0.0, 3, 1.0)
    with pytest.raises(JavaGRBException):
        P.PathPlanner().plan_paths_advanced_td(g, [0, 1], [1, 0], 0.0, 3, 1.0)


def test_split_dist_failure_reasons():
    """fixed planPathsAdvancedSplitDist: each piece gets the whole time limit (a slow first
    half does not starve the second); the limit spent by a piece's makespan solve, and an
    infeasible piece, give None with the reason in failures (and time_limit_hit for the
    former)."""
    g = create_2d_grid_graph(3, 3, True)
    s, t = [0, 2], [8, 6]
    ok = P.PathPlanner(rng=MathRandom(1)).plan_paths_advanced_split_dist(g, s, t, 0.0, 10.0, 1)
    assert ok is not None and P.path_set_error(ok, g, s, t) is None
    now = [0.0]

    def planner_after(method):  # the injected clock jumps by 100 s after each call of method
        pl = P.PathPlanner(rng=MathRandom(1), clock=lambda: now[0])
        f = getattr(pl, method)

        def slow(*a, **k):
            r = f(*a, **k)
            now[0] += 100.0
            return r

        setattr(pl, method, slow)
        return pl

    pl = planner_after("plan_paths_advanced_td")  # the first half finishes after the limit
    r = pl.plan_paths_advanced_split_dist(g, s, t, 0.0, 10.0, 1)
    assert r is not None and P.path_set_error(r, g, s, t) is None  # the second half has its own limit
    assert pl.failures == [] and not pl.time_limit_hit
    pl = planner_after("_leaf")  # each piece's makespan solve uses up the limit
    assert pl.plan_paths_advanced_split_dist(g, s, t, 0.0, 10.0, 1) is None
    assert pl.failures == ["time limit reached after a piece's makespan solve"] * 2
    assert pl.time_limit_hit

    def infeasible(*a, **k):
        raise Infeasible("no plan (stub)")

    pl = P.PathPlanner(rng=MathRandom(1))
    pl.plan_paths_advanced_td = infeasible
    assert pl.plan_paths_advanced_split_dist(g, s, t, 0.0, -1, 1) is None
    mid = pl.failures[0].split(" -> ")[1].split(":")[0]
    assert pl.failures == [f"piece {s} -> {mid}: no plan (stub)", f"piece {mid} -> {t}: no plan (stub)"]
    pl = P.PathPlanner(rng=MathRandom(1))
    pl.plan_paths_advanced_td = infeasible
    with pytest.raises(Infeasible, match="stub"):  # no split: the top-level piece raises
        pl.plan_paths_advanced_split_dist(g, s, t, 0.0, -1, 0)


@pytest.mark.parametrize("objective", ["total_time", "total_distance"])
def test_plan_original_single_horizon_trims_and_proves(objective):
    """Regression (C-ORIG-TT-PADDED-MAKESPAN-STATUS): compat="original" solves TT/DIST at the
    4-way split's makespan (8 here) and kept the idle steps at the end, so PlanResult.makespan
    and the CLI said 8 where every robot arrives by t = 3; and the status stayed "feasible"
    although the value meets the lower bound (7 = 7)."""
    from mppflow.problem import create_grid_problem
    p = create_grid_problem(3, 3, 0.0, 3, MathRandom(1))
    r = P.plan(p, objective, compat="original", backend="builtin", rng=MathRandom(1))
    assert r.extra["horizon"] == 8 and r.makespan == len(r.paths[0]) - 1
    assert any(q[-1] != q[-2] for q in r.paths)  # no trailing step in which every robot waits
    if objective == "total_time":  # every robot arrives by t = 3 (waits are free for DIST)
        assert r.makespan == 3
    assert r.value == 7 == r.lower_bounds[objective] and r.status == "optimal"
    assert P.validate(p, r.paths) is None
