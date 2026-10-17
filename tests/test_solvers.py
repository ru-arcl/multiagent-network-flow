"""Tests of mppflow.solvers: golden solve records under Gurobi (Threads=1; plans, stdout,
random draws, mutated arguments, crashes and hangs), objective equality under HiGHS and the
builtin backend, the fixed-mode corrections, the horizon search (Algorithm 1), max
distance, and the paper's worked examples."""
from __future__ import annotations

import os
import re

import pytest

from mppflow import planner as P
from mppflow import solvers as S
from mppflow._java import MathRandom
from mppflow.errors import Infeasible, JavaNullPointerException, NotInOriginal, OriginalWouldHang
from mppflow.graph import Graph, create_2d_grid_graph, create_main_case3_graph
from mppflow.problem import get_a9_puzzle, get_long_straight_with_one_garage_problem
from helpers import GRB, from_edges, graph_from_json, load_golden, needs_gurobi  # noqa: F401


HERE = os.path.dirname(__file__)
GOLDEN = os.path.join(HERE, "golden")


RECS = load_golden("solvers")
SOLVE_RECS = [r for r in RECS if not r["fn"].startswith("PuzzleSolver")]
TIME_FNS = ("MultiagentGraphSolverGurobiTime.solve", "MultiagentGraphSolverGurobiTimeR.solve")


def norm(s):
    return re.sub(r"(found in |max time limit )(-?\d+\.\d+(E-?\d+)?|-?Infinity|NaN)", r"\1<t>", s)


def run_record(r, compat, backend, params=None):
    """Run a golden record; returns (result, rng, mutated)."""
    a, st, fn = r["args"], r.get("statics") or {}, r["fn"]
    rng = MathRandom(r["seed"])
    g = graph_from_json(a["graph"])
    cls = fn.split(".")[0]
    if fn in TIME_FNS:
        f = S.solve_time_r if "TimeR" in fn else S.solve_time
        res = f(g, a["starts"], a["goals"], a["solveRelaxedFirst"], a["setOptimal"], a["extraSteps"],
                a["hasTimeLimit"], a["timeLimit"], rng=rng, backend=backend, params=params, compat=compat,
                debug=st.get(cls + ".bDebugInfo", True), print_paths=st.get(cls + ".bPrintPaths", True))
        return res, rng, None
    if fn.endswith("TotalTime.solve"):
        opt = list(a["opt"])
        res = S.solve_total_time(g, a["starts"], a["goals"], opt, a["gap"], a["timeSteps"], a["hasTimeLimit"],
                                 a["timeLimit"], return_any_solution=st.get(cls + ".bReturnAnySolution", False),
                                 horizon="single", rng=rng, backend=backend, params=params, compat=compat,
                                 debug=st.get(cls + ".bDebugInfo", True),
                                 print_paths=st.get(cls + ".bPrintPaths", True))
        return res, rng, {"opt": opt}
    if fn.endswith("Distance.solve"):
        res = S.solve_distance(g, a["starts"], a["goals"], a["gap"], a["expansion"], a["timeLimit"],
                               return_any_solution=st.get(cls + ".bReturnAnySolution", False), horizon="single",
                               backend=backend, params=params, compat=compat,
                               debug=st.get(cls + ".bDebugInfo", True),
                               print_paths=st.get(cls + ".bPrintPaths", False))
        return res, rng, None
    raise AssertionError(fn)


def check_plan(g, starts, goals, paths, one_move=False):
    """The paper's motion model: arcs of the graph, no meets (t >= 1), no head-on swaps."""
    assert [p[0] for p in paths] == list(starts)
    assert [p[-1] for p in paths] == list(goals)
    L = len(paths[0])
    assert all(len(p) == L for p in paths)
    for t in range(L - 1):
        for p in paths:
            assert p[t + 1] in g.adj[p[t]]
        nxt = [p[t + 1] for p in paths]
        assert len(set(nxt)) == len(nxt), f"meet at t={t + 1}"
        moves = {(p[t], p[t + 1]) for p in paths if p[t] != p[t + 1]}
        assert not any((v, u) in moves for (u, v) in moves), f"head-on at t={t}"
        if one_move:
            assert len(moves) <= 1


def distance(paths):
    return sum(p[i] != p[i + 1] for p in paths for i in range(len(p) - 1))


def total_time(paths, goals):
    tot = 0
    for p, g in zip(paths, goals):
        t = len(p) - 1
        while t > 0 and p[t - 1] == g:
            t -= 1
        tot += t
    return tot


# ----------------------------------------------------- golden, Gurobi, original
@pytest.mark.gurobi
@needs_gurobi
@pytest.mark.parametrize("rec", SOLVE_RECS, ids=lambda r: r["id"])
def test_golden_original_gurobi(rec):
    if rec.get("hang"):
        with pytest.raises(OriginalWouldHang):
            run_record(rec, "original", "gurobi", GRB)
        return
    if "exception" in rec:
        assert rec["exception"]["class"] == "java.lang.NullPointerException"
        with pytest.raises(JavaNullPointerException) as ei:
            run_record(rec, "original", "gurobi", GRB)
        assert str(ei.value) == rec["exception"]["message"]
        return
    res, rng, mut = run_record(rec, "original", "gurobi", GRB)
    nondet = rec.get("nondet") or []
    if "result" not in nondet:
        assert res.paths == rec["result"]
    assert norm(res.log) == rec.get("stdout", "")
    assert rng.draws == rec["draws"]
    assert rng.random() == rec["rng_next"]
    if rec.get("mutated"):
        assert mut == rec["mutated"]
    elif mut is not None:
        assert mut == {"opt": rec["args"]["opt"]}


# fixed mode: same as the golden record except here
FIXED_EXPECT = {
    "solvers/Time/hasTL_nonpositive_star": ("paths", [[0, 0, 1, 3, 3], [3, 1, 2, 1, 0]]),
    "solvers/Time/hasTL_zero_star": ("paths", [[0, 0, 1, 3, 3], [3, 1, 2, 1, 0]]),
    "solvers/Time/hasTL_nonpositive_relaxed_3x3": ("paths", [[0, 1, 2, 5, 8], [2, 5, 8, 7, 6], [6, 3, 0, 1, 1]]),
    "solvers/Time/dupstarts": ("raises", ValueError),
    "solvers/Time/unreachable_npe": ("raises", Infeasible),
    "solvers/Time/swap_line2_hang": ("raises", Infeasible),
    "solvers/Time/dupgoals_hang": ("raises", Infeasible),
    "solvers/TimeR/6x6_swap_high_ids": ("makespan", 12),
    "solvers/TimeR/6x6_swap_low_ids": ("makespan", 12),
    "solvers/TotalTime/line3_T2_nan": ("opt", 2),
    "solvers/TotalTime/cycle3_rotation_T1_nan": ("opt", 3),
    "solvers/Distance/stay_T3_nan": ("paths", [[0, 0, 0, 0], [3, 3, 3, 3]]),
    "solvers/Distance/stay_T0_nan": ("paths", [[0], [3]]),
}
FIXED_LOG_DIFFERS = {"solvers/TotalTime/infeasible_T1", "solvers/Distance/infeasible_T1"}


@pytest.mark.gurobi
@needs_gurobi
@pytest.mark.parametrize("rec", SOLVE_RECS, ids=lambda r: r["id"])
def test_golden_fixed_gurobi(rec):
    exp = FIXED_EXPECT.get(rec["id"])
    if exp is not None and exp[0] == "raises":
        with pytest.raises(exp[1]):
            run_record(rec, "fixed", "gurobi", GRB)
        return
    res, rng, mut = run_record(rec, "fixed", "gurobi", GRB)
    a = rec["args"]
    g = graph_from_json(a["graph"])
    if exp is None:
        assert res.paths == rec["result"]
        if rec["id"] not in FIXED_LOG_DIFFERS:
            assert norm(res.log) == rec.get("stdout", "")
        assert rng.draws == rec["draws"]
        if rec.get("mutated"):
            assert mut == rec["mutated"]
        return
    kind, val = exp
    assert res.paths is not None and res.status == "optimal"
    check_plan(g, a["starts"], a["goals"], res.paths, one_move="TimeR" in rec["fn"])
    if kind == "paths":
        assert res.paths == val
    elif kind == "makespan":
        assert res.makespan == val
    else:
        assert mut["opt"] == [val] and res.objective == val


# ------------------------------------------------ objectives on other backends
def golden_objective(rec):
    res = rec["result"]
    if rec["fn"] in TIME_FNS:
        return len(res[0]) - 1
    if rec["fn"].endswith("TotalTime.solve"):
        return rec["mutated"]["opt"][0]
    return distance(res)


OBJ_RECS = [r for r in SOLVE_RECS if r.get("result") is not None and r["id"] not in FIXED_EXPECT
            and not (r.get("nondet"))]


@pytest.mark.parametrize("rec", OBJ_RECS, ids=lambda r: r["id"])
def test_objective_highs(rec):
    res, rng, mut = run_record(rec, "fixed", "highs")
    a = rec["args"]
    check_plan(graph_from_json(a["graph"]), a["starts"], a["goals"], res.paths, one_move="TimeR" in rec["fn"])
    assert res.objective == golden_objective(rec)
    assert rng.draws == rec["draws"]  # the draws do not depend on the backend
    if rec["fn"].endswith("TotalTime.solve"):
        assert total_time(res.paths, a["goals"]) == res.objective
    if rec["fn"].endswith("Distance.solve"):
        assert distance(res.paths) == res.objective


SMALL = [r for r in OBJ_RECS if graph_from_json(r["args"]["graph"]).n <= 9 and len(r["args"]["starts"]) <= 3
         and "TimeR" not in r["fn"]]


@pytest.mark.parametrize("rec", SMALL, ids=lambda r: r["id"])
def test_objective_builtin(rec):
    res, rng, mut = run_record(rec, "fixed", "builtin")
    a = rec["args"]
    check_plan(graph_from_json(a["graph"]), a["starts"], a["goals"], res.paths)
    assert res.objective == golden_objective(rec)


# ------------------------------------------------------- fixed-mode behaviour
def star():
    return create_main_case3_graph()


def test_infeasible_detection():
    line2 = Graph.from_adjacency([[1], [0]])
    with pytest.raises(OriginalWouldHang):
        S.solve_time(line2, [0, 1], [1, 0], compat="original")
    r = S.solve_time(line2, [0, 1], [1, 0], False, True, 0, True, 1e6, compat="original")
    assert r.paths is None and r.status == "time_limit" and r.extra["shortcut"]
    with pytest.raises(Infeasible):
        S.solve_time(line2, [0, 1], [1, 0])
    with pytest.raises(Infeasible):
        S.solve_time(line2, [0, 1], [1, 0], True)  # relaxed first: LP feasible, ILP never
    with pytest.raises(OriginalWouldHang):
        S.solve_time_r(line2, [0, 1], [1, 0], compat="original")
    # t_max
    with pytest.raises(Infeasible):
        S.solve_time(star(), [0, 3], [3, 0], t_max=3)
    assert S.solve_time(star(), [0, 3], [3, 0], t_max=4).makespan == 4


def test_joint_feasibility():
    line3 = from_edges(3, [(0, 1), (1, 2)])
    cyc3 = from_edges(3, [(0, 1), (1, 2), (2, 0)])
    assert S.joint_feasibility(line3, [0, 2], [2, 0]) is False
    assert S.joint_feasibility(cyc3, [0, 1, 2], [1, 2, 0]) is True  # rotation
    assert S.joint_feasibility(cyc3, [0, 1, 2], [1, 2, 0], one_move=True) is False
    assert S.joint_feasibility(star(), [0, 3], [3, 0]) is True
    assert S.joint_feasibility(star(), [0, 3], [3, 3]) is False
    assert S.joint_feasibility(star(), [0, 0], [3, 2]) is True  # duplicate start (t = 0 unconstrained)
    g = create_2d_grid_graph(6, 6, True)
    assert S.joint_feasibility(g, list(range(10)), list(range(26, 36)), budget=100) is None


def test_hastimelimit_nonpositive():
    """original-bugs B2: original returns null at the first failed T."""
    r = S.solve_time(star(), [0, 3], [3, 0], False, True, 0, True, -1.0, compat="original")
    assert r.paths is None and r.status == "time_limit"
    r = S.solve_time(star(), [0, 3], [3, 0], False, True, 0, True, -1.0)
    assert r.makespan == 4


class StepClock:
    def __init__(self, step):
        self.t = 1000.0
        self.step = step

    def __call__(self):
        self.t += self.step
        return self.t


def limits(log):
    return [float(x) for x in re.findall(r"max time limit (-?\d+\.\d+(?:E-?\d+)?)", log)]


def test_time_limit_accounting():
    """original-bugs B2: the original subtracts the cumulative
    elapsed time at every iteration; fixed prints the true remaining time."""
    kw = dict(debug=True)
    ro = S.solve_time(star(), [0, 3], [3, 0], True, True, 0, True, 1000.0, compat="original",
                      clock=StepClock(0.25), **kw)
    rf = S.solve_time(star(), [0, 3], [3, 0], True, True, 0, True, 1000.0, clock=StepClock(0.25), **kw)
    lo, lf = limits(ro.log), limits(rf.log)
    assert ro.makespan == rf.makespan == 4
    assert len(lo) == len(lf) == 4
    assert lo[0] == pytest.approx(lf[0], abs=1e-9)
    assert all(a < b for a, b in zip(lo[1:], lf[1:]))
    # cumulative: the decrement of the printed limit grows with the elapsed time
    dec = [lo[i] - lo[i + 1] for i in range(len(lo) - 1)]
    assert dec == sorted(dec) and dec[0] < dec[-1]
    # fixed: limit + elapsed is constant (1000)
    assert lf == sorted(lf, reverse=True) and 1000 - lf[-1] < 5


def test_time_limit_expires_fixed():
    clk = StepClock(10.0)
    r = S.solve_time(star(), [0, 3], [3, 0], False, True, 0, True, 15.0, clock=clk)
    assert r.paths is None and r.status == "time_limit"


def test_nan_gap_fixed_vs_original():
    line3 = from_edges(3, [(0, 1), (1, 2)])
    opt = [-777]
    r = S.solve_total_time(line3, [0], [2], opt, 0.0, 2, horizon="single", compat="original")
    assert r.paths is None and r.status == "rejected" and opt == [-777]
    r = S.solve_total_time(line3, [0], [2], opt, 0.0, 2, horizon="single")
    assert r.paths == [[0, 1, 2]] and opt == [2]
    r = S.solve_distance(star(), [0, 3], [0, 3], 0.0, 3, compat="original")
    assert r.paths is None and r.status == "rejected"
    r = S.solve_distance(star(), [0, 3], [0, 3], 0.0, 3, horizon="single")
    assert r.objective == 0 and r.paths == [[0, 0, 0, 0], [3, 3, 3, 3]]
    r = S.solve_distance(star(), [0, 3], [0, 3])  # search: already at the lower bound
    assert r.objective == 0


def test_original_rejects_search_and_maxdist():
    with pytest.raises(NotInOriginal):
        S.solve_total_time(star(), [0, 3], [3, 0], None, 0.0, 4, horizon="search", compat="original")
    with pytest.raises(NotInOriginal):
        S.solve_distance(star(), [0, 3], [3, 0], 0.0, 4, horizon="search", compat="original")
    with pytest.raises(NotInOriginal):
        S.solve_max_distance(star(), [0, 3], [3, 0], compat="original")
    with pytest.raises(ValueError):
        S.solve_total_time(star(), [0, 3], [3, 0], None, 0.0, None, horizon="single")


def test_unreachable_goal():
    g = Graph.from_adjacency([[1], [0], [3], [2]])
    with pytest.raises(JavaNullPointerException) as ei:
        S.solve_total_time(g, [0], [3], None, 0.0, 3, horizon="single", compat="original")
    assert "<local14>[<local15>]" in str(ei.value)
    for f in (S.min_total_time, S.min_total_distance, S.min_max_distance, S.min_makespan):
        with pytest.raises(Infeasible):
            f(g, [0], [3])
    r = S.solve_distance(g, [0], [3], 0.0, 3, compat="original")  # infeasible model, null
    assert r.paths is None


def test_zero_agents():
    g = star()
    assert S.solve_time(g, [], [], compat="original").paths is None
    assert S.solve_time(g, [], []).paths == []


def test_tt_search_consumes_draws_like_one_call():
    """In search mode the random draws are those of one original TT call."""
    g = create_2d_grid_graph(3, 3, True)
    s, t = [0, 2, 6], [8, 6, 1]
    r1, r2 = MathRandom(5), MathRandom(5)
    S.solve_total_time(g, s, t, None, 0.0, 4, horizon="single", rng=r1)
    S.solve_total_time(g, s, t, None, 0.0, None, rng=r2)
    assert r1.draws == r2.draws > 0
    from mppflow._java import default_rng
    before = default_rng().state()
    S.min_total_distance(g, s, t)  # DIST and max distance never draw
    S.min_max_distance(g, s, t)
    assert default_rng().state() == before


def test_log_echo(capsys):
    S.solve_time(star(), [0, 3], [3, 0], debug=True, print_paths=True, echo=True)
    out = capsys.readouterr().out
    assert "Solution with optimal makespan found in" in out and "Agent 1: 0:3" in out
    S.solve_time(star(), [0, 3], [3, 0])
    assert capsys.readouterr().out == ""


def test_time_puzzle_single_T():
    g = star()
    r = S.solve_time_puzzle(g, [0, 3], [3, 0], True, 4)
    check_plan(g, [0, 3], [3, 0], r.paths)
    assert r.extra == {"good_solution": 1, "failed_attempt": False}
    # one solve above the lower bound proves a plan, not the optimal makespan (fixed)
    assert (r.status, r.lower_bound) == ("feasible", 2)
    assert S.solve_time_puzzle(g, [0, 3], [3, 0], True, 4, compat="original").status == "optimal"
    path3 = Graph.from_adjacency([[1], [0, 2], [1]])
    assert S.solve_time_puzzle(path3, [0], [2], True, 6).status == "feasible"
    assert S.solve_time_puzzle(path3, [0], [2], True, 2).status == "optimal"
    r = S.solve_time_puzzle(g, [0, 3], [3, 0], False, 3, instance=2)
    assert r.paths is None and r.extra == {"good_solution": None, "failed_attempt": True}
    g3 = create_2d_grid_graph(3, 3, True)  # golden PuzzleSolver.solve(double)/3x3
    assert S.solve_time_puzzle(g3, [0, 6], [2, 8], True, 2).paths == [[0, 1, 2], [6, 7, 8]]


# ---------------------------------------------------------- paper examples
FIG4 = dict(edges=[(0, 2), (1, 2), (2, 4), (3, 4), (4, 5)], starts=[3, 5], goals=[0, 1])


@pytest.mark.parametrize("backend", ["highs", "builtin"])
def test_fig4a_all_objectives(backend):
    """tor16_fig4_fig7_network.json: optimal makespan 4; eq16 corrected min distance 6;
    min max distance 3; total time 7."""
    g = from_edges(6, FIG4["edges"])
    s, t = FIG4["starts"], FIG4["goals"]
    r = S.min_makespan(g, s, t, backend=backend)
    assert r.makespan == 4 and r.lower_bound == 3
    check_plan(g, s, t, r.paths)
    for f, val, metric in ((S.min_total_time, 7, lambda p: total_time(p, t)), (S.min_total_distance, 6, distance),
                           (S.min_max_distance, 3, lambda p: max(distance([q]) for q in p))):
        r = f(g, s, t, backend=backend)
        assert r.objective == val
        check_plan(g, s, t, r.paths)
        assert metric(r.paths) == val


def test_9puzzle_fig1_and_a9():
    """Fig. 1/3 9-puzzle: makespan 4 (= lower bound); getA9Puzzle: 5 (lower bound 3)."""
    g = create_2d_grid_graph(3, 3, True)
    s = [2, 4, 5, 1, 8, 6, 7, 3, 0]
    r = S.min_makespan(g, s, list(range(9)))
    assert r.makespan == 4
    check_plan(g, s, list(range(9)), r.paths)
    p = get_a9_puzzle()
    r = S.solve_time(p.graph, p.starts, p.goals, True)
    assert r.makespan == 5 and r.lower_bound == 3


def test_garage_corridor():
    """tor16_bounds_objectives.json garage_corridor: T_opt 10, min total distance 18, min
    total time 19."""
    p = get_long_straight_with_one_garage_problem()
    g = p.graph
    assert S.min_makespan(g, p.starts, p.goals).makespan == 10
    assert S.min_total_time(g, p.starts, p.goals).objective == 19
    assert S.min_total_distance(g, p.starts, p.goals).objective == 18


def test_eq15_counterexample_total_time():
    """3x4 grid minus vertex 6, T=8: the true optimum of the sum of arrival times is 21
    (the paper's eq. (15) would pick a plan with 22)."""
    ids = [v for v in range(12) if v != 6]
    mp = {v: i for i, v in enumerate(ids)}
    edges = []
    for v in ids:
        r_, c_ = divmod(v, 4)
        for w in ((v + 1) if c_ < 3 else None, (v + 4) if r_ < 2 else None):
            if w is not None and w in mp:
                edges.append((mp[v], mp[w]))
    g = from_edges(11, edges)
    s = [mp[v] for v in [1, 7, 8, 3, 10]]
    t = [mp[v] for v in [11, 1, 8, 3, 10]]
    r = S.solve_total_time(g, s, t, None, 0.0, 8, horizon="single")
    assert r.objective == 21 and total_time(r.paths, t) == 21
    assert S.min_makespan(g, s, t).makespan == 6
    assert S.min_total_time(g, s, t).objective == 21


def test_horizon_search_total_time_gadget():
    """TT gadget (paper-vs-code, "TT and DIST are solved at one horizon"), L=12: TT at
    T_opt=13 is 36, the optimum 32 needs T>=19."""
    L = 12
    l0, l1, l2, r1, r2 = L + 1, L + 2, L + 3, L + 4, L + 5
    g = from_edges(L + 6, [(i, i + 1) for i in range(L)] + [(l0, 0), (l1, 0), (l2, 0), (r1, 4), (r2, 4)])
    s, t = [l0, r1, r2], [L, l1, l2]
    single = S.solve_total_time(g, s, t, None, 0.0, 13, horizon="single")
    assert single.objective == 36
    opt = [0]
    r = S.solve_total_time(g, s, t, opt, 0.0, 13)
    assert r.objective == 32 == opt[0] == total_time(r.paths, t)
    check_plan(g, s, t, r.paths)
    hs = r.extra["horizon_search"]
    assert hs["T0"] == 13 and hs["T_hi"] >= 19
    assert r.makespan == 19  # trailing idle steps trimmed
    assert S.min_total_time(g, s, t).objective == 32


def test_horizon_search_distance_gadget():
    """DIST gadget (paper-vs-code, "TT and DIST are solved at one horizon"), N=10: DIST at
    T_opt=14 is 26, the optimum 24 needs T>=23."""
    n_ = 10
    idx = n_ + 1
    edges = [(i, i + 1) for i in range(n_)]
    sA, gB, sB, gA = idx, idx + 1, idx + 2, idx + 3
    idx += 4
    edges += [(sA, 0), (gB, 0), (sB, n_), (gA, n_)]
    prev = n_
    for _ in range(n_ + 1):
        edges.append((prev, idx))
        prev = idx
        idx += 1
    edges.append((prev, 0))
    g = from_edges(idx, edges)
    s, t = [sA, sB], [gA, gB]
    assert S.solve_distance(g, s, t, 0.0, 14, horizon="single").objective == 26
    r = S.solve_distance(g, s, t, 0.0, 14)
    assert r.objective == 24 == distance(r.paths)
    check_plan(g, s, t, r.paths)
    assert S.min_total_distance(g, s, t).objective == 24
    # max distance: both robots must cover 12 (around or straight plus the leaves)
    md = S.min_max_distance(g, s, t)
    check_plan(g, s, t, md.paths)
    assert md.objective == max(distance([p]) for p in md.paths) >= 12


@pytest.mark.gurobi
@needs_gurobi
def test_search_gurobi_matches_single_at_lower_bound():
    """When the single-horizon solution is already at the lower bound, search mode returns
    exactly the original's plan (golden solvers/TotalTime/3x3_T4)."""
    rec = next(r for r in SOLVE_RECS if r["id"] == "solvers/TotalTime/3x3_T4")
    a = rec["args"]
    g = graph_from_json(a["graph"])
    rng = MathRandom(rec["seed"])
    r = S.solve_total_time(g, a["starts"], a["goals"], None, 0.0, a["timeSteps"], rng=rng, backend="gurobi",
                           params=GRB)
    assert r.paths == rec["result"] and r.objective == rec["mutated"]["opt"][0]
    assert rng.draws == rec["draws"]


# ------------------------------------------------- regressions (models group)
def test_time_r_original_hang_uses_one_move_check():
    """TimeR cannot rotate a triangle (one move per step) at any T, Time can (T = 1).
    Java's TimeR.solve loops forever here (verified: 'Trying T = 249 ...' after 20 s);
    the original's one-move rows are exact because every vertex id is below 22."""
    tri = Graph.from_adjacency([[0, 1, 2]] * 3)
    assert S.joint_feasibility(tri, [0, 1, 2], [1, 2, 0]) is True
    assert S.joint_feasibility(tri, [0, 1, 2], [1, 2, 0], one_move=True) is False
    assert S._time_r_rows_exact(tri)
    with pytest.raises(OriginalWouldHang):
        S.solve_time_r(tri, [0, 1, 2], [1, 2, 0], compat="original", rng=MathRandom(1))
    with pytest.raises(Infeasible):
        S.solve_time_r(tri, [0, 1, 2], [1, 2, 0], rng=MathRandom(1))
    # with a time limit the original's null result comes back at once
    r = S.solve_time_r(tri, [0, 1, 2], [1, 2, 0], False, True, 0, True, 1e6, compat="original",
                       rng=MathRandom(1))
    assert r.paths is None and r.extra["shortcut"]
    # ids >= 22: the original rows miss arcs, only the unrestricted check is sound
    assert not S._time_r_rows_exact(create_2d_grid_graph(6, 6, True))


def test_distances_follow_one_way_adjacency():
    """The BFS goes from the start to the goal, not back (one-way adjacency)."""
    g = Graph.from_adjacency([[1], [2], []])  # 0 -> 1 -> 2 (self-loops added)
    assert S._distances(g, [0], [2]) == [2]
    assert S._distances(g, [2], [0]) == [None]
    assert S.joint_feasibility(g, [0], [2]) is True
    assert S.joint_feasibility(g, [2], [0]) is False
    for compat in ("original", "fixed"):
        r = S.solve_total_time(g, [0], [2], [-1], 0.0, 3, horizon="single", compat=compat, rng=MathRandom(1))
        assert r.objective == 2 and r.paths == [[0, 1, 2, 2]]
        r = S.solve_distance(g, [0], [2], 0.0, 3, horizon="single", compat=compat)
        assert r.objective == 2 and r.paths == [[0, 1, 2, 2]]
    assert S.min_total_time(g, [0], [2]).objective == 2
    assert S.min_total_distance(g, [0], [2]).objective == 2
    assert S.min_max_distance(g, [0], [2]).objective == 2
    with pytest.raises(Infeasible):
        S.min_total_time(g, [2], [0])


def exact_makespan(g, starts, goals, t_cap=40):
    """Oracle: the minimum makespan by layered BFS over joint configurations (the paper's
    motion model: arcs incl. self-loops, no meets, no head-on swaps); None if > t_cap."""
    import itertools

    goal = tuple(goals)
    layer = {tuple(starts)}
    seen = set(layer)
    for t in range(t_cap + 1):
        if goal in layer:
            return t
        nxt = set()
        for cfg in layer:
            for tgt in itertools.product(*(g.adj[v] for v in cfg)):
                if len(set(tgt)) < len(tgt):
                    continue
                moves = {(u, w) for u, w in zip(cfg, tgt) if u != w}
                if any((w, u) in moves for (u, w) in moves):
                    continue
                if tgt not in seen:
                    seen.add(tgt)
                    nxt.add(tgt)
        layer = nxt
        if not layer:
            return None
    return None


ONE_WAY_CYCLE = [[1], [2], [3], [0]]  # 0 -> 1 -> 2 -> 3 -> 0, plus self-loops


@needs_gurobi
def test_one_way_makespan_matches_java():
    """Live Java (Threads 1) MultiagentGraphSolverGurobiTime.solve: T = 2, 3, 4 infeasible,
    then makespan 5 with exactly this plan (draws 4) -- reproduced by ``original``. Its
    forward goal-side BFS prunes the optimal horizon; ``fixed`` (reversed BFS) finds the
    optimal makespan 4, which the exact joint-state BFS confirms."""
    g = Graph.from_adjacency([[1], [2, 0], [3], [1], [2, 3]])
    starts, goals = [4, 0, 3], [3, 1, 0]
    java = [[4, 4, 4, 4, 4, 3], [0, 1, 2, 3, 1, 1], [3, 3, 1, 0, 0, 0]]
    assert S.joint_feasibility(g, starts, goals) is True
    assert exact_makespan(g, starts, goals) == 4
    rng = MathRandom(1)
    r = S.solve_time(g, starts, goals, compat="original", backend="gurobi", params=GRB, rng=rng, debug=True)
    assert r.paths == java
    assert [a["T"] for a in r.attempts] == [2, 3, 4, 5]
    assert rng.draws == 4
    rng = MathRandom(1)
    r = S.solve_time(g, starts, goals, backend="gurobi", params=GRB, rng=rng, debug=True)
    assert r.paths == [[4, 4, 4, 2, 3], [0, 1, 2, 3, 1], [3, 3, 1, 0, 0]]
    check_plan(g, starts, goals, r.paths)
    assert [a["T"] for a in r.attempts] == [2, 3, 4]
    assert rng.draws == 4  # the same shortest-path draws as the original


def test_one_way_cycle_fixed_optimal_original_suboptimal():
    """Regression, one-way 4-cycle 0 -> 3: the plan 0,1,2,3 (makespan 3) is valid.
    ``original`` (live Java: makespan 5) prunes it at T = 3, 4; ``fixed`` does not."""
    g = Graph.from_adjacency(ONE_WAY_CYCLE)
    assert S.solve_time(g, [0], [3], compat="original").makespan == 5
    assert S.solve_time(g, [0], [3]).paths == [[0, 1, 2, 3]]
    assert S.solve_time_r(g, [0], [3]).paths == [[0, 1, 2, 3]]
    assert S.min_makespan(g, [0], [3], t_max=3).paths == [[0, 1, 2, 3]]
    for compat, status in (("fixed", "optimal"), ("original", "infeasible")):
        tt = S.solve_total_time(g, [0], [3], None, 0.0, 3, horizon="single", compat=compat)
        dd = S.solve_distance(g, [0], [3], 0.0, 3, horizon="single", compat=compat)
        assert (tt.status, dd.status) == (status, status)
    assert S.solve_total_time(g, [0], [3], None, 0.0, 3, horizon="single").paths == [[0, 1, 2, 3]]
    for f in (S.min_total_time, S.min_total_distance, S.min_max_distance):
        assert f(g, [0], [3]).paths == [[0, 1, 2, 3]]


def test_one_way_cycle_without_wait_loops():
    """Without self-loops every horizon of the original's pruned model is infeasible
    (live Java was still at 'Trying T = 302' after 20 s), so ``original`` raises
    OriginalWouldHang (or takes the time-limit shortcut); ``fixed`` solves it. Detected by
    the joint-state certificate ``_original_pruning_dead`` (its verdicts were checked on
    25 random instances: every original model for T = 1..30 is infeasible)."""
    g = Graph()
    g.vertices = [0, 1, 2, 3]
    g.adj = {0: [1], 1: [2], 2: [3], 3: [0]}
    assert S.joint_feasibility(g, [0], [3]) is True
    assert S._original_pruning_dead(g, [0], [3]) is True
    for f in (S.solve_time, S.solve_time_r):
        assert f(g, [0], [3]).paths == [[0, 1, 2, 3]]
        assert f(g, [0], [3], t_max=60).paths == [[0, 1, 2, 3]]
        with pytest.raises(OriginalWouldHang):
            f(g, [0], [3], compat="original")
        r = f(g, [0], [3], compat="original", has_time_limit=True, time_limit=5.0)
        assert (r.paths, r.status, r.extra.get("shortcut")) == (None, "time_limit", True)
    # two robots: each alone has a pruned path for some T, jointly they never do
    g2 = Graph()
    g2.vertices = list(range(6))
    g2.adj = {0: [0, 3, 4, 5], 1: [0], 2: [1, 4, 5], 3: [1, 4, 5], 4: [0, 4], 5: [3, 4, 5]}
    assert [S._original_pruning_dead(g2, [s], [t]) for s, t in ((4, 3), (2, 1))] == [False, False]
    assert S._original_pruning_dead(g2, [4, 2], [3, 1]) is True
    assert S._original_pruning_dead(g2, [4, 2], [3, 1], budget=10) is None
    assert exact_makespan(g2, [4, 2], [3, 1]) == 3
    r = S.solve_time(g2, [4, 2], [3, 1])
    assert r.makespan == 3
    check_plan(g2, [4, 2], [3, 1], r.paths)
    with pytest.raises(OriginalWouldHang):
        S.solve_time(g2, [4, 2], [3, 1], compat="original")
    # the backward sets B_k never empty but become periodic once the windows stop restricting:
    # the per-robot pass (one robot on a one-way 0 -> 4 -> 2 -> 0 cycle with side arcs) and the
    # joint pass (each robot alone has a pruned path) end by detecting the repeat
    g3 = _bare([[4], [0, 3], [0], [1], [2]])
    assert S.solve_time(g3, [4], [0]).paths == [[4, 2, 0]]
    for per_robot_only in (False, True):
        assert S._original_pruning_dead(g3, [4], [0], per_robot_only=per_robot_only) is True
    with pytest.raises(OriginalWouldHang):
        S.solve_time(g3, [4], [0], compat="original")
    g4 = _bare([[1, 4], [3, 5], [1, 4, 5, 2], [0, 4, 3], [0], [3]])
    assert [S._original_pruning_dead(g4, [s], [t]) for s, t in ((5, 0), (3, 4))] == [False, False]
    assert S._original_pruning_dead(g4, [5, 3], [0, 4]) is True
    r = S.solve_time(g4, [5, 3], [0, 4])
    assert r.paths == [[5, 3, 0], [3, 0, 4]]
    check_plan(g4, [5, 3], [0, 4], r.paths)
    with pytest.raises(OriginalWouldHang):
        S.solve_time(g4, [5, 3], [0, 4], compat="original")
    # the certificate is never raised where the original's pruning leaves a path
    assert S._original_pruning_dead(Graph.from_adjacency(ONE_WAY_CYCLE), [0], [3]) is False
    assert S._original_pruning_dead(create_2d_grid_graph(3, 3), [0, 8], [8, 0]) is False


def test_one_way_random_fixed_makespan_is_optimal():
    """Random small one-way graphs (self-loops added): ``fixed`` returns the exact optimal
    makespan with a valid plan; ``original`` is never better and stops (with wait loops
    the original's windows open up for large T)."""
    import random

    rnd = random.Random(2024)
    checked = 0
    while checked < 40:
        n = rnd.randint(3, 6)
        adj = [sorted(rnd.sample([w for w in range(n) if w != v], rnd.randint(1, min(2, n - 1))))
               for v in range(n)]
        g = Graph.from_adjacency(adj)
        A = rnd.randint(1, min(3, n - 1))
        verts = rnd.sample(range(n), 2 * A) if 2 * A <= n else None
        if verts is None:
            starts, goals = rnd.sample(range(n), A), rnd.sample(range(n), A)
        else:
            starts, goals = verts[:A], verts[A:]
        opt = exact_makespan(g, starts, goals, t_cap=25)
        if opt is None:
            continue
        r = S.solve_time(g, starts, goals, rng=MathRandom(checked))
        assert r.makespan == opt, (adj, starts, goals)
        check_plan(g, starts, goals, r.paths)
        ro = S.solve_time(g, starts, goals, rng=MathRandom(checked), compat="original")
        assert ro.makespan >= opt
        checked += 1


def test_t_max_checked_before_first_horizon():
    """t_max also bounds the first horizon T_min + extra_steps (no plan above t_max)."""
    path4 = Graph.from_adjacency([[1], [0, 2], [1, 3], [2]])  # T_min = 3 for 0 -> 3
    with pytest.raises(Infeasible):
        S.solve_time(path4, [0], [3], t_max=1)
    with pytest.raises(Infeasible):
        S.min_makespan(path4, [0], [3], t_max=2)
    with pytest.raises(Infeasible):
        S.solve_time(path4, [0], [3], extra_steps=2, t_max=3)
    with pytest.raises(Infeasible):
        S.solve_time_r(path4, [0], [3], t_max=2)
    assert S.solve_time(path4, [0], [3], t_max=3).makespan == 3
    assert S.solve_time(path4, [0], [3], extra_steps=2, t_max=5).makespan == 5
    # the shortest-path draws come first, exactly as in a run without t_max
    rng, rng2 = MathRandom(7), MathRandom(7)
    with pytest.raises(Infeasible):
        S.solve_time(path4, [0], [3], t_max=2, rng=rng)
    S.solve_time(path4, [0], [3], rng=rng2)
    assert rng.draws == rng2.draws
    # original mode ignores t_max (the Java has none)
    assert S.solve_time(path4, [0], [3], t_max=1, compat="original").makespan == 3


# ------------------------------------------------- regressions
def test_relaxed_phase_keeps_printing_until_lp_feasible():
    """Regression: with a time limit, an instance proved infeasible after the first
    failed relaxed horizon used to return at once, dropping the original's deterministic
    "Trying T = 3 ... relaxed model feasible." line (live Java, LINE3 swap, seed 5,
    bDebugInfo=false). The relaxed phase now keeps raising T until the relaxation is
    feasible; the integer loop (silent without debug) is then cut short."""
    g = Graph.from_adjacency({0: [0, 1], 1: [0, 1, 2], 2: [1, 2]})
    java = ("\nSolving relaxed model first. \nTrying T = 2 with max time limit <t> ... relaxed model not "
            "feasible, increase T...\nTrying T = 3 with max time limit <t> ... relaxed model feasible.\n")
    for backend in ("highs", "builtin"):
        r = S.solve_time(g, [0, 2], [2, 0], True, True, 0, True, 3.0, rng=MathRandom(5), backend=backend,
                         compat="original")
        assert norm(r.log) == java
        assert (r.paths, r.status, r.extra) == (None, "time_limit", {"shortcut": True})
        assert [(a["T"], a["phase"]) for a in r.attempts] == [(2, "lp"), (3, "lp"), (3, "ilp")]
    # duplicate goals: the relaxation is never feasible (two flows end on one vertex), so
    # the relaxed phase is still cut short at once
    r = S.solve_time(g, [0, 2], [1, 1], True, True, 0, True, 3.0, rng=MathRandom(5), compat="original")
    assert r.extra == {"shortcut": True} and [a["phase"] for a in r.attempts] == ["lp"]


def test_fixed_makespan_time_limit_and_failure_branches(monkeypatch):
    """fixed solve_time with a stubbed backend and an injected clock: an LP relaxation that
    times out or fails, the relaxed phase stopped by the limit, and the integer loop stopped
    before its next solve. Each gives status "time_limit" (or raises) and its log line."""
    from mppflow.lp.linear import Solution

    g = create_2d_grid_graph(3, 3)
    now = [0.0]
    run = S._run
    stub = {}

    def fake(m, backend, params, time_limit, mip_gap):
        f = stub.get("ilp" if m.model.is_mip else "lp")
        return run(m, backend, params, time_limit, mip_gap) if f is None else f(m)

    monkeypatch.setattr(S, "_run", fake)

    def solve(relaxed=True, **kw):
        now[0] = 0.0
        return S.solve_time(g, [0, 2], [2, 0], relaxed, True, 0, True, 10.0, rng=MathRandom(5),
                            clock=lambda: now[0], **kw)

    stub["lp"] = lambda m: Solution(status="time_limit", backend="fake")
    r = solve()
    assert (r.status, r.paths, [a["phase"] for a in r.attempts]) == ("time_limit", None, ["lp"])
    assert r.log.endswith("Trying T = 2 with max time limit 10.0 ... time limit reached.\n")
    stub["lp"] = lambda m: Solution(status="error", backend="fake")
    with pytest.raises(RuntimeError, match="LP relaxation at T=2 returned 'error'"):
        solve()

    def lp_infeasible_late(m):  # relaxation value 0 < n, and the limit passes meanwhile
        now[0] = 1e6
        return Solution(status="optimal", objective=0.0, x=[0.0] * m.model.num_columns, backend="fake")

    stub["lp"] = lp_infeasible_late
    r = solve()
    assert (r.status, r.T) == ("time_limit", 2)
    assert r.log.endswith("... relaxed model not feasible, time limit reached.\n")
    # the integer loop: T = 2 has no plan, and the limit passes before the solve at T = 3
    stub.clear()
    stub["ilp"] = lambda m: Solution(status="infeasible", backend="fake")
    reach = S.update_reachable_sets

    def late_reach(graph, starts, goals, T, compat):
        if T > 2:
            now[0] = 1e6
        return reach(graph, starts, goals, T, compat)

    monkeypatch.setattr(S, "update_reachable_sets", late_reach)
    r = solve(relaxed=False, debug=True)
    assert (r.status, r.T, [(a["T"], a["phase"]) for a in r.attempts]) == ("time_limit", 3, [(2, "ilp")])
    assert r.log.endswith("integer model infeasible.\n time out.\n")


def test_relaxed_phase_guard_with_frozen_clock(monkeypatch):
    """A relaxation that never becomes feasible (simulated) under a clock that never expires:
    the relaxed phase gives up after |V| + n further horizons instead of looping."""
    from mppflow.lp.linear import Solution

    def fake(m, backend, params, time_limit, mip_gap):
        return Solution(status="optimal", objective=0.0, x=[0.0] * m.model.num_columns, backend="fake")

    monkeypatch.setattr(S, "_run", fake)
    g = Graph.from_adjacency({0: [0, 1], 1: [0, 1, 2], 2: [1, 2]})
    r = S.solve_time(g, [0, 2], [2, 0], True, True, 0, True, 3.0, rng=MathRandom(5), compat="original",
                     clock=lambda: 0.0)
    assert r.extra == {"shortcut": True}
    assert [a["T"] for a in r.attempts] == list(range(2, 2 + 1 + 3 + 2))  # proved at T=2, then 5 more


DENSE_INFEASIBLE = ([[1, 2], [0, 3], [0, 3, 5], [1, 2, 4], [3, 6], [2], [4]],  # 3x3 grid minus 2 cells
                    [1, 0, 2, 5, 3, 4], [6, 3, 0, 2, 4, 1])


def test_inconclusive_feasibility_check_is_retried():
    """Regression (feasibility-budget-hang): the default budget runs out on this instance
    (840 reachable joint configurations, none the goal); the fixed-mode loop used to keep the
    inconclusive answer and try T = 7, 8, ... forever. The check is now retried with a growing
    budget, so every fixed-mode entry point raises Infeasible after a couple of horizons."""
    nbrs, s, g = DENSE_INFEASIBLE
    G = Graph.from_adjacency(nbrs)
    assert S.joint_feasibility(G, s, g) is None  # the default budget alone is inconclusive
    assert S.joint_feasibility(G, s, g, budget=2 * S.DEFAULT_FEASIBILITY_BUDGET) is False
    for f in (S.min_makespan, S.min_total_time, S.min_total_distance, S.min_max_distance):
        with pytest.raises(Infeasible, match="joint-state search"):
            f(G, s, g)
    with pytest.raises(Infeasible, match="joint-state search"):
        S.min_makespan(G, s, g, solve_relaxed_first=True)
    with pytest.raises(Infeasible, match="joint-state search"):  # exposed on the wrappers
        S.min_makespan(G, s, g, feasibility_budget=10**6)


TREE6 = ([[1, 3], [0, 2, 5], [1, 4], [0], [2], [1]], [5, 0, 1], [0, 2, 1])  # oracle: TT 17, TD 16, MD 6
DENSE6 = ([[1, 3], [0, 4], [3], [0, 2, 4, 5], [1, 3], [3]], [5, 3, 2, 1, 0], [2, 0, 3, 1, 4])  # 37, 26, 6


@pytest.mark.parametrize("inst", [TREE6, pytest.param(DENSE6, marks=pytest.mark.slow)],  # dense6: about 25 s
                         ids=["tree6", "dense6"])
def test_horizon_search_proves_optimality_by_joint_search(inst):
    """Regression (horizon-search-T_hi-too-large): when the plan at T0 = T_opt is already
    optimal but above the trivial bound, the search used to solve one more model at
    T_hi (16..33 here) to prove it: 30 s to over 15 min with HiGHS or Gurobi. The exact
    joint-state search now settles it, and no model at T_hi is solved."""
    nbrs, s, t = inst
    g = Graph.from_adjacency(nbrs)
    want = {"tt": 17, "td": 16, "md": 6} if inst is TREE6 else {"tt": 37, "td": 26, "md": 6}
    for key, f in (("tt", S.min_total_time), ("td", S.min_total_distance), ("md", S.min_max_distance)):
        r = f(g, s, t)
        hs = r.extra["horizon_search"]
        assert r.objective == want[key] and r.status == "optimal", (key, r.objective)
        assert hs["proof"] == "joint-state search" and hs["T_hi"] > hs["T0"]
        assert hs["T_hi"] not in [a["T"] for a in r.attempts]
        check_plan(g, s, t, r.paths)


def test_joint_optimum_improves_and_matches_model():
    """joint_optimum on the single-horizon TT gadget (paper-vs-code) finds the optimum 32
    (which needs T >= 19; the model at T_opt = 13 gives 36) with the smallest makespan, 19."""
    L = 12
    l0, l1, l2, r1, r2 = L + 1, L + 2, L + 3, L + 4, L + 5
    g = from_edges(L + 6, [(i, i + 1) for i in range(L)] + [(l0, 0), (l1, 0), (l2, 0), (r1, 4), (r2, 4)])
    s, t = [l0, r1, r2], [L, l1, l2]
    val, paths = S.joint_optimum(g, s, t, "total_time")
    assert val == 32 == total_time(paths, t) and len(paths[0]) - 1 == 19
    check_plan(g, s, t, paths)
    assert S.joint_optimum(g, s, t, "distance")[0] == S.solve_distance(g, s, t, 0.0, 30, horizon="single").objective
    line = from_edges(3, [(0, 1), (1, 2)])
    assert S.joint_optimum(line, [0, 2], [2, 0], "distance") is False
    assert S.joint_optimum(line, [0, 2], [2, 0], "total_time", budget=1) is None
    assert S.joint_optimum(line, [0], [2], "max_distance", upper=2) == (2, None)


def test_horizon_search_time_limit_at_T_hi_is_not_optimal(monkeypatch):
    """When the time limit stops the proof at T_hi, the plan at T0 is not proved optimal
    for every horizon: status "feasible" (it used to say "optimal")."""
    monkeypatch.setattr(S, "JOINT_SEARCH_MAX_CONFIGS", 0)  # force the model at T_hi
    nbrs, s, t = TREE6
    g = Graph.from_adjacency(nbrs)
    r = S.min_total_distance(g, s, t, time_limit=1.0)
    assert r.objective == 16 and r.status == "feasible" and r.extra["time_limit_at_T_hi"]
    assert "proof" not in r.extra["horizon_search"]


# ------------------------------------------- t_max, gap and single-horizon statuses
MAZE8 = ([[0, 1, 3, 6], [0, 1, 2], [1, 2, 5, 6], [0, 3, 4, 6], [3, 4], [2, 5, 7], [0, 2, 3, 6], [5, 7]],
         [1, 4, 2, 3, 0], [2, 1, 4, 6, 0])  # optimal makespan 4; oracle: TT 17, TD 14 (makespan 5), MD 4
OBJECTIVE_FNS = (S.min_total_time, S.min_total_distance, S.min_max_distance)


@pytest.mark.parametrize("f", OBJECTIVE_FNS, ids=["tt", "td", "md"])
def test_t_max_below_optimal_makespan_raises(f):
    """Regression: t_max below the optimal makespan used to replace T_hi and return the plan at
    T0 (makespan 4 > t_max = 3) with status "optimal". Now every objective raises, as
    min_makespan does, also when the first horizon T is given above t_max."""
    nbrs, s, t = MAZE8
    g = Graph.from_adjacency(nbrs)
    with pytest.raises(Infeasible, match="t_max = 3"):
        S.min_makespan(g, s, t, t_max=3)
    for T in (None, 9):
        with pytest.raises(Infeasible, match="t_max = 3"):
            f(g, s, t, T=T, t_max=3)


def test_t_max_caps_the_horizon_search():
    """t_max caps T_hi (it used to replace it) and lowers a given T: the result is the optimum
    among the plans with makespan <= t_max, never a plan above it."""
    nbrs, s, t = MAZE8
    g = Graph.from_adjacency(nbrs)
    r = S.min_total_distance(g, s, t, t_max=4)
    assert (r.status, r.objective, r.makespan) == ("optimal", 16, 4)  # 14 needs makespan 5
    assert r.extra["horizon_search"]["T_hi"] == 4
    r = S.min_total_distance(g, s, t, T=9, t_max=4)
    assert (r.status, r.objective, r.makespan) == ("optimal", 16, 4)
    assert max(a["T"] for a in r.attempts) <= 4
    r = S.min_total_distance(g, s, t, t_max=5)
    assert (r.status, r.objective) == ("optimal", 14) and r.makespan <= 5
    r = S.min_total_distance(g, s, t, t_max=60)  # a large t_max no longer replaces the provable T_hi
    assert r.objective == 14 and r.extra["horizon_search"]["T_hi"] < 60


def test_given_T_below_the_optimal_makespan_falls_back_to_T_opt():
    """Algorithm 1 with a given T that has no plan: T0 becomes max(T, T_opt) (docstring of
    _search), and the result is the optimum of the search without T."""
    from mppflow.problem import create_grid_problem
    p = create_grid_problem(3, 4, 0.0, 6, MathRandom(35))  # optimal makespan 3
    for f in OBJECTIVE_FNS:
        a = f(p.graph, p.starts, p.goals, T=1)
        b = f(p.graph, p.starts, p.goals)
        assert (a.status, a.objective) == (b.status, b.objective) == ("optimal", {
            "min_total_time": 13, "min_total_distance": 12, "min_max_distance": 3}[f.__name__])
        assert a.extra["horizon_search"]["T0"] == a.extra["horizon_search"]["T_opt"] == 3


def test_budget_spent_before_the_T_hi_solve_is_not_optimal(monkeypatch):
    """The time limit runs out after the solve at T0 and before the one at T_hi (an injected
    clock jumps when the joint-state search is considered): the plan at T0 is returned with
    status "feasible" and extra["time_limit_at_T_hi"], and the model at T_hi is never built."""
    nbrs, s, t = MAZE8
    g = Graph.from_adjacency(nbrs)
    now = [0.0]

    def few_configs(nv, A):  # called right before the T_hi stage; no joint-state search
        now[0] = 1e6
        return False

    monkeypatch.setattr(S, "_few_configs", few_configs)
    calls = {
        "tt": lambda: S.solve_total_time(g, s, t, None, 0.0, None, True, 100.0, clock=lambda: now[0]),
        "td": lambda: S.solve_distance(g, s, t, 0.0, None, 100.0, clock=lambda: now[0]),
        "md": lambda: S.solve_max_distance(g, s, t, 0.0, None, 100.0, clock=lambda: now[0]),
    }
    want = {"tt": (17, 12), "td": (16, 16), "md": (4, 20)}  # (value at T0 = 4, T_hi)
    for k, call in calls.items():
        now[0] = 0.0
        r = call()
        hs = r.extra["horizon_search"]
        assert (r.status, r.objective, hs["T0"], hs["T_hi"]) == ("feasible", want[k][0], 4, want[k][1]), k
        assert r.extra["time_limit_at_T_hi"] and "proof" not in hs
        assert max(a["T"] for a in r.attempts) == 4, k


def test_gap_result_is_not_called_optimal(monkeypatch):
    """Regression: with gap > 0 the backend stops within the gap and says "optimal"; the
    solvers copied that, labelling a plan of total distance 16 (optimum 14) "optimal". Now a
    gap result is "feasible" unless its bound closes or the lower bound / joint-state search
    proves it."""
    nbrs, s, t = MAZE8
    g = Graph.from_adjacency(nbrs)
    assert S.min_total_distance(g, s, t, gap=0.5).objective == 14  # joint-state search: proved
    assert S.min_total_distance(g, s, t, gap=0.5).status == "optimal"
    monkeypatch.setattr(S, "JOINT_SEARCH_MAX_CONFIGS", 0)  # instances above the joint-search size
    want = {"min_total_time": 17, "min_total_distance": 14}
    for f in (S.min_total_time, S.min_total_distance):
        r = f(g, s, t, gap=0.5)
        assert r.objective >= want[f.__name__]
        assert r.status == "feasible" or r.objective == want[f.__name__], (f.__name__, r.status, r.objective)
    r = S.min_total_distance(g, s, t, gap=0.5)
    # 16 when HiGHS stops within the gap (scipy 1.17.1); other HiGHS versions may reach 14
    assert r.objective in (14, 16) and (r.status == "feasible" or r.objective == 14)


def test_proven_needs_closed_bound_when_gap_positive():
    from mppflow.lp import Solution

    sol = Solution(status="optimal", objective=16.0, bound=9.0, x=[], backend="fake")
    # "optimal" with gap 0 but an open bound: a tolerance in params (Gurobi MIPGapAbs, HiGHS
    # mip_abs_gap) stopped the solve, so it proves nothing; without a bound it is trusted
    assert not S._proven(sol, 0.0, 16, 9.0) and not S._proven(sol, 0.5, 16, 9.0)
    assert S._proven(sol, 0.0, 16, None) and S._proven(sol, 0.0, 16, 15.9999995)
    assert S._proven(sol, 0.5, 16, 15.2) and not S._proven(sol, 0.5, 16, None)
    assert S._proven(Solution(status="time_limit", objective=16.0, bound=16.0, x=[], backend="fake"), 0.5, 16, 16.0)
    # boundary: a bound one unit below the value proves nothing (with a 1e-6 tolerance), a
    # bound above that rounds up to the value
    assert not S._proven(sol, 0.5, 16, 15.0) and not S._proven(sol, 0.5, 16, 15.0000001)
    assert S._proven(sol, 0.5, 16, 15.01) and S._proven(sol, 0.5, 16, 15.9999999)


def test_single_horizon_status_and_infeasible():
    """horizon="single": a plan optimal at one T is "optimal" only when it meets the lower bound
    (else "feasible", extra["optimal_at_T"]); the min_* wrappers raise Infeasible when T is too
    short (they used to return status "infeasible" without raising)."""
    g = create_2d_grid_graph(3, 4)
    from mppflow.problem import create_grid_problem
    p = create_grid_problem(3, 4, 0.0, 6, MathRandom(35))
    a = S.min_total_time(p.graph, p.starts, p.goals, T=3, horizon="single")
    b = S.min_total_time(p.graph, p.starts, p.goals)
    assert (a.objective, b.objective) == (14, 13)
    assert a.status == "feasible" and a.extra["optimal_at_T"] and b.status == "optimal"
    for f in OBJECTIVE_FNS:
        with pytest.raises(Infeasible, match="T = 1"):
            f(g, [0], [11], T=1, horizon="single")
        r = f(g, [0], [11], T=5, horizon="single")  # meets the lower bound 5
        assert r.status == "optimal" and "optimal_at_T" not in r.extra


def test_extra_steps_makespan_is_not_optimal():
    """A makespan loop started at T_min + extra_steps proves nothing below its first horizon."""
    from mppflow.problem import create_grid_problem
    p = create_grid_problem(4, 4, 0.0, 3, MathRandom(5))
    a = S.solve_time(p.graph, p.starts, p.goals, extra_steps=0)
    b = S.solve_time(p.graph, p.starts, p.goals, extra_steps=2)
    assert (a.status, a.objective, b.status, b.objective) == ("optimal", 6, "feasible", 8)
    g = create_2d_grid_graph(3, 3)
    assert S.solve_time(g, [0], [0], extra_steps=0).status == "optimal"


def test_max_distance_joint_search_pareto_pruning():
    """Regression: the max-distance joint search over (configuration, moves per robot) ran out
    of budget on 8 vertices / 5 robots, and the fallback model at T_hi = 35 ran for over 15
    minutes. Keeping only the Pareto-minimal move vectors proves the optimum 7 in well under a
    second (an unpruned search with a 100x budget and an independent oracle agree)."""
    nbrs = [[0, 1, 2], [0, 1, 3, 4, 7], [0, 2, 5], [1, 3, 6], [1, 4, 7], [2, 5], [3, 6], [1, 4, 7]]
    s, t = [1, 0, 3, 7, 5], [7, 4, 0, 1, 3]
    g = Graph.from_adjacency(nbrs)
    assert S.joint_optimum(g, s, t, "max_distance", upper=7) == (7, None)
    val, paths = S.joint_optimum(g, s, t, "max_distance", upper=8)
    assert val == 7 and max(sum(p[i] != p[i + 1] for i in range(len(p) - 1)) for p in paths) == 7
    check_plan(g, s, t, paths)
    for upper in (9, 10):  # the search keeps lowering D past its first improvement
        val, paths = S.joint_optimum(g, s, t, "max_distance", upper=upper)
        assert val == 7 and max(sum(p[i] != p[i + 1] for i in range(len(p) - 1)) for p in paths) == 7


def test_solvers_reject_unconverted_graph_ids():
    """The holes generators return 1-based ids with gaps; the solvers name the cause instead of
    reporting an existing vertex as out of range."""
    from mppflow.graph import convert_graph, create_generic_2d_grid_graph_with_holes
    g = create_generic_2d_grid_graph_with_holes(4, 4, 0.0, MathRandom(1))
    for f in (S.min_makespan, S.min_total_time):
        with pytest.raises(ValueError, match="convert_graph"):
            f(g, [1], [16])
    assert S.min_makespan(convert_graph(g), [0], [15]).objective == 6


def _bare(adj):
    """A graph with exactly these arcs (no wait loops added, unlike Graph.from_adjacency)."""
    g = Graph()
    for i, nb in enumerate(adj):
        g.adj[i] = sorted(nb)
        g.xy[i] = None
    g.vertices = list(range(len(adj)))
    return g


def test_single_horizon_respects_t_max():
    """horizon="single" solves at min(T, t_max): the plan fits t_max, and Infeasible is raised
    when no plan does (t_max used to be ignored in single mode)."""
    g = Graph.from_adjacency([[1], [0, 2], [1]])
    for f in OBJECTIVE_FNS:
        r = f(g, [0], [2], T=4, t_max=2, horizon="single")
        assert r.T == 2 and len(r.paths[0]) - 1 == 2 and r.extra["T_requested"] == 4
        with pytest.raises(Infeasible, match="t_max = 1"):
            f(g, [0], [2], T=4, t_max=1, horizon="single")
        with pytest.raises(Infeasible, match="t_max = 3"):  # the swap has no plan at all
            f(g, [0, 2], [2, 0], T=5, t_max=3, horizon="single")
        assert "T_requested" not in f(g, [0], [2], T=3, t_max=5, horizon="single").extra
    r = S.solve_distance(g, [0], [2], 0.0, 4, horizon="single", t_max=2)
    assert r.T == 2 and r.objective == 2


def test_single_horizon_without_T_names_T():
    """horizon="single" without a horizon: the error names T, the keyword of the min_*
    functions (it used to name only the inner parameters time_steps / expansion)."""
    g = Graph.from_adjacency([[1], [0, 2], [1]])
    for f in OBJECTIVE_FNS:
        with pytest.raises(ValueError, match=r"needs the horizon T \((time_steps|expansion)\)"):
            f(g, [0], [2], horizon="single")


def test_makespan_loop_above_feasible_horizons_ends():
    """Without wait loops the feasible horizons need not be upward-closed: a loop started at
    T_min + extra_steps above all of them must end (it used to try T, T+1, ... forever,
    because the proof answered "feasible for some T")."""
    path = _bare([[1], [0, 2], [1]])  # 0 - 1 - 2, no wait loops
    dag = _bare([[1], [2], []])        # 0 -> 1 -> 2, no wait loops
    with pytest.raises(Infeasible, match="T >= 1"):
        S.solve_time(path, [0, 2], [0, 2], False, True, 1)
    with pytest.raises(Infeasible, match="T >= 1"):
        P.PathPlanner().plan_paths_advanced(path, [0, 2], [0, 2], False, 1, True, -1.0)
    with pytest.raises(Infeasible, match="T >= 3"):
        S.solve_time(dag, [0], [2], False, True, 1)
    assert S.solve_time(dag, [0], [2]).paths == [[0, 1, 2]]
    assert S.solve_time(path, [0, 2], [0, 2]).paths == [[0], [2]]
    assert S.joint_feasibility(path, [0, 2], [0, 2]) is True
    assert S.joint_feasibility(path, [0, 2], [0, 2], t_first=1) is False
    assert S.joint_feasibility(dag, [0], [2], t_first=3) is False
    assert S.joint_feasibility(dag, [0], [2], t_first=2) is True
    # a one-way 6-cycle without wait loops: plans exist at multiples of 6 only
    c6 = _bare([[1], [2], [3], [4], [5], [0]])
    assert S.joint_feasibility(c6, [5, 0, 2], [5, 0, 2], t_first=1) is True
    r = S.solve_time(c6, [5, 0, 2], [5, 0, 2], False, False, 1)
    assert (r.status, r.T) == ("feasible", 6)
    with pytest.raises(OriginalWouldHang, match="T >= 1"):
        S.solve_time(path, [0, 2], [0, 2], False, True, 1, compat="original")


def test_original_hang_detected_for_robots_at_their_goals():
    """compat="original": robots whose start is their goal need a closed walk at T >= 1; on a
    one-way 6-cycle without wait loops the original's forward pruning removes every horizon
    and live Java loops forever. Both certificates used to treat start == goal as solved."""
    c6 = _bare([[1], [2], [3], [4], [5], [0]])
    assert S._original_pruning_dead(c6, [5, 0, 2], [5, 0, 2]) is True
    assert S._original_pruning_dead(c6, [5], [5], per_robot_only=True) is True
    r = S.solve_time(c6, [5, 0, 2], [5, 0, 2], False, False, 1, True, 1e6, rng=MathRandom(1), compat="original")
    assert (r.status, r.paths, r.extra.get("shortcut")) == ("time_limit", None, True)
    with pytest.raises(OriginalWouldHang):
        S.solve_time(c6, [5, 0, 2], [5, 0, 2], False, False, 1, rng=MathRandom(1), compat="original")


def test_horizon_search_does_not_solve_unlimited_after_the_budget(monkeypatch):
    """When the makespan stage uses up time_limit, the search returns "time_limit" instead of
    solving the objective model at T0 without a limit."""
    from mppflow import lp
    from mppflow.problem import create_grid_problem
    seen = []
    orig = lp.solve

    def spy(m, backend="highs", time_limit=None, **k):
        seen.append(time_limit)
        return orig(m, backend=backend, time_limit=time_limit, **k)

    monkeypatch.setattr(S.lp, "solve", spy)
    p = create_grid_problem(3, 4, 0.0, 6, MathRandom(35))
    for f in (lambda c: S.solve_total_time(p.graph, p.starts, p.goals, None, 0.0, None, True, 10.0, clock=c),
              lambda c: S.solve_distance(p.graph, p.starts, p.goals, 0.0, None, 10.0, clock=c),
              lambda c: S.solve_max_distance(p.graph, p.starts, p.goals, 0.0, None, 10.0, clock=c)):
        n = []
        seen.clear()

        def clock():  # 12 s spent of a 10 s budget once the makespan stage is done
            n.append(1)
            return 0.0 if len(n) == 1 else (9.0 if len(n) < 3 else 12.0)

        r = f(clock)
        assert r.status == "time_limit" and r.paths is None
        assert None not in seen and len(seen) == 1


# ------------------------------------------------ goals without wait self-loops
def _no_wait_grid():
    """2x5 grid with wait loops only at 2 and 8; robot 1's goal 5 has none. Exhaustive best
    total time per horizon: T=4: 8, T=5: 7, T=6: 8, T=7: 9 (horizons are not nested)."""
    g = create_2d_grid_graph(2, 5)
    for v in (0, 1, 3, 4, 5, 6, 7, 9):
        g.adj[v].remove(v)
    return g, [6, 8], [8, 5]


NOLOOP6 = ([[3], [1, 2, 4, 5], [2, 3], [0, 2, 3, 4], [1, 2, 3, 4], [1, 4]], [1, 0, 2, 5], [0, 5, 2, 4])


def _raw_graph(adj):
    g = Graph()
    for v, nb in enumerate(adj):
        g.adj[v] = list(nb)
        g.xy[v] = None
    g.vertices = list(range(len(adj)))
    return g


@pytest.mark.parametrize("joint", [True, False], ids=["joint-search", "scan"])
@pytest.mark.parametrize("backend", ["highs", "builtin"])
def test_horizon_search_without_goal_wait_loops(monkeypatch, joint, backend):
    """Regression (C-NOWAIT-TT-FALSE-OPTIMAL): without a wait loop at a goal the optimum is not
    monotone in T. The search solved only T0 = 4 (total time 8) and T_hi = 6 (8 again) and
    reported 8 as optimal; the optimum 7 sits at T = 5. It now settles such instances by the
    joint-state search, or else solves every horizon up to T_hi."""
    if not joint:
        monkeypatch.setattr(S, "JOINT_SEARCH_MAX_CONFIGS", 0)
    g, s, t = _no_wait_grid()
    for f, want, T in ((S.min_total_time, 7, 5), (S.min_total_distance, 7, 4), (S.min_max_distance, 4, 4)):
        r = f(g, s, t, backend=backend)
        hs = r.extra["horizon_search"]
        assert (r.status, r.objective, r.T) == ("optimal", want, T), f.__name__
        assert hs["every_horizon"] and hs["T_opt"] == 4 and ("proof" in hs) == joint
        assert P.path_set_error(r.paths, g, s, t) is None
        if not joint:  # every horizon from T_opt up to T_hi was solved
            hz = [a["T"] for a in r.attempts]
            # (after the makespan stage; on builtin it ends with the joint-state plan at T = 4)
            assert hz[len(hz) - (hs["T_hi"] - 3):] == list(range(4, hs["T_hi"] + 1)), (f.__name__, hz)
    # a given T does not restrict the search, which starts at the optimal makespan
    assert S.min_total_time(g, s, t, T=6, backend=backend).objective == 7
    if joint or backend != "builtin":  # the builtin scan up to T_hi = 12 takes minutes
        g6 = _raw_graph(NOLOOP6[0])
        r = S.min_total_time(g6, NOLOOP6[1], NOLOOP6[2], backend=backend)
        assert (r.status, r.objective) == ("optimal", 14)  # it was "optimal" 16 at T = 4


def test_horizon_search_without_goal_wait_loops_time_limit(monkeypatch):
    """A time limit during the scan returns the best plan so far as "feasible"."""
    monkeypatch.setattr(S, "JOINT_SEARCH_MAX_CONFIGS", 0)
    g, s, t = _no_wait_grid()
    ticks = []

    def clock():  # the budget runs out after the solve at T_opt
        ticks.append(1)
        return 0.0 if len(ticks) < 6 else 100.0

    r = S.solve_total_time(g, s, t, None, 0.0, None, True, 50.0, clock=clock)
    assert r.paths is not None and r.status == "feasible" and r.extra["time_limit_in_scan"]


NOLOOP_TREE7 = ([[1, 3], [0, 2, 6], [1, 4, 5], [0], [2], [2], [1]], {1, 2, 3, 5, 6}, [0, 2, 3], [2, 6, 4])
"""A tree with wait loops only at 1, 2, 3, 5, 6 (goal 4 has none): max distance 6 at the optimal
makespan 6 and 4 (the lower bound) at T = 7 (exhaustive search, tests/oracles.py)."""


def _loops_only(adj, loops):
    g = Graph.from_adjacency(adj)
    for v in range(len(adj)):
        if v not in loops:
            g.adj[v].remove(v)
    return g


def _scan_horizons(r, T_opt):
    """The horizons solved by the scan (the attempts from the solve at ``T_opt`` on)."""
    hz = [a["T"] for a in r.attempts]
    return hz[len(hz) - 1 - hz[::-1].index(T_opt):]


def test_horizon_scan_lower_bound_exits(monkeypatch):
    """The scan without goal wait loops stops as soon as the value meets the lower bound: at
    the optimal makespan (no scan at all), or at the first horizon above it that reaches it."""
    monkeypatch.setattr(S, "JOINT_SEARCH_MAX_CONFIGS", 0)
    path = _raw_graph([[1], [0, 2], [1]])  # no wait loops: total time 2 = lb at T_opt = 2
    r = S.min_total_time(path, [0], [2])
    hs = r.extra["horizon_search"]
    assert (r.status, r.objective, r.paths, hs["T_opt"], hs["T_hi"]) == ("optimal", 2, [[0, 1, 2]], 2, None)
    adj, loops, s, t = NOLOOP_TREE7
    g = _loops_only(adj, loops)
    for backend in ("highs", "builtin"):
        r = S.min_max_distance(g, s, t, backend=backend)
        hs = r.extra["horizon_search"]
        assert (r.status, r.objective, r.lower_bound, r.T, hs["T_opt"]) == ("optimal", 4, 4, 7, 6), backend
        assert hs["T_hi"] > 7 and _scan_horizons(r, 6) == [6, 7], (backend, r.attempts)  # stopped at 7
        assert P.path_set_error(r.paths, g, s, t) is None and P.plan_metrics(r.paths, t)["max_distance"] == 4
    # the value meets the lower bound, so "optimal" even if an earlier solve (here the one at
    # T_opt) only proved its gap
    orig = S._dist_single

    def within_gap(graph, starts, goals, gap, T, *a, **k):
        r = orig(graph, starts, goals, gap, T, *a, **k)
        if T == 6 and r.status == "optimal":
            r.status = "feasible"
        return r

    monkeypatch.setattr(S, "_dist_single", within_gap)
    r = S.min_max_distance(g, s, t)
    assert (r.status, r.objective, r.T) == ("optimal", 4, 7)


def _stub_scan(monkeypatch, change):
    """``S._tt_single`` (the total-time solve of one horizon) replaced, for the horizons above
    T_opt = 4 of :func:`_no_wait_grid`, by ``change(T, solve)`` (``solve()`` runs the real
    one); the horizons asked for are recorded."""
    monkeypatch.setattr(S, "JOINT_SEARCH_MAX_CONFIGS", 0)
    orig, asked = S._tt_single, []

    def stub(graph, starts, goals, opt, gap, T, *a, **k):
        asked.append(T)
        solve = lambda: orig(graph, starts, goals, opt, gap, T, *a, **k)  # noqa: E731
        return change(T, solve) if T > 4 else solve()

    monkeypatch.setattr(S, "_tt_single", stub)
    return asked


def test_horizon_scan_within_the_gap_is_not_a_proof(monkeypatch):
    """A scan horizon solved only within the gap (status "feasible") leaves the result
    "feasible": the optimum of that horizon, and so over all horizons, is not proved."""
    def within_gap(T, solve):
        r = solve()
        if r.status == "optimal":
            r.status = "feasible"
        return r

    asked = _stub_scan(monkeypatch, within_gap)
    g, s, t = _no_wait_grid()
    r = S.min_total_time(g, s, t)
    assert (r.status, r.objective, r.T) == ("feasible", 7, 5)
    assert asked == list(range(4, r.extra["horizon_search"]["T_hi"] + 1))  # the whole scan ran


def test_horizon_scan_time_limit_in_a_solve(monkeypatch):
    """A scan solve stopped by the time limit ends the scan: the best plan so far (8 at
    T_opt = 4; the optimum 7 is at T = 5, the stopped solve) is returned as "feasible", and no
    horizon above the stopped one is solved (T = 6 and 7 would give 8 and 9, so going on would
    report 8 as optimal)."""
    asked = _stub_scan(monkeypatch, lambda T, solve: S.SolveResult(None, "time_limit", T=T) if T == 5 else solve())
    g, s, t = _no_wait_grid()
    r = S.min_total_time(g, s, t)
    assert (r.status, r.objective, r.T) == ("feasible", 8, 4) and r.extra["time_limit_in_scan"]
    assert asked == [4, 5]


def test_horizon_scan_time_limit_at_every_step(monkeypatch):
    """The budget running out at any point of the scan: either no plan ("time_limit"), the
    best plan so far ("feasible", marked when the scan was cut) or the optimum 7 ("optimal",
    only once every horizon up to T_hi was solved)."""
    monkeypatch.setattr(S, "JOINT_SEARCH_MAX_CONFIGS", 0)
    g, s, t = _no_wait_grid()
    seen = set()
    for n in range(1, 60):
        ticks = []

        def clock():  # the budget runs out at the n-th reading of the clock
            ticks.append(1)
            return 0.0 if len(ticks) < n else 100.0

        r = S.solve_total_time(g, s, t, None, 0.0, None, True, 50.0, clock=clock)
        seen.add(r.status)
        if r.paths is None:
            assert r.status == "time_limit", n
            continue
        assert P.path_set_error(r.paths, g, s, t) is None
        assert P.plan_metrics(r.paths, t)["total_time"] == r.objective, n
        if r.status == "optimal":
            assert r.objective == 7 and not r.extra.get("time_limit_in_scan"), n
        else:
            assert r.status == "feasible" and r.extra.get("time_limit_in_scan"), (n, r.status, r.extra)
        if len(ticks) < n:  # the budget never ran out: the full search
            assert r.status == "optimal"
            break
    assert seen == {"time_limit", "feasible", "optimal"}, seen


def test_joint_optimum_without_goal_wait_loops():
    """joint_optimum used to return None when a goal has no self-loop; the searches are exact
    there too (a robot cannot be frozen at such a goal before the last step)."""
    g, s, t = _no_wait_grid()
    val, paths = S.joint_optimum(g, s, t, "total_time")
    assert val == 7 == P.plan_metrics(paths, t)["total_time"] and P.path_set_error(paths, g, s, t) is None
    assert S.joint_optimum(g, s, t, "distance")[0] == 7
    assert S.joint_optimum(g, s, t, "max_distance", upper=5)[0] == 4
    path = _raw_graph([[1], [0, 2], [1]])  # no wait loops at all: robots must keep moving
    assert S.joint_optimum(path, [0], [2], "total_time") == (2, [[0, 1, 2]])
    assert S.joint_optimum(path, [0, 2], [0, 2], "total_time") == (0, [[0], [2]])
    assert S.joint_optimum(path, [0, 1], [2, 0], "total_time") is False  # no plan (oracle-checked)


# ------------------------------------------------------------------ input checks
def test_numpy_vertex_ids_are_accepted():
    """Regression (C-NUMPY-*): numpy integer starts/goals/adjacency crashed every solver in the
    Java-key emulation (OverflowError); bool and float ids now raise TypeError."""
    np = pytest.importorskip("numpy")
    g = Graph.from_adjacency([[1], [0, 2], [1, 3], [2]])
    want = S.min_makespan(g, [0, 3], [1, 2]).objective
    for dt in (np.int64, np.int32):
        r = S.min_makespan(g, np.array([0, 3], dtype=dt), np.array([1, 2], dtype=dt), backend="builtin")
        assert r.objective == want and all(type(v) is int for p in r.paths for v in p)
        assert S.min_total_time(g, np.array([0], dtype=dt), np.array([3], dtype=dt)).objective == 3
    assert S.min_makespan(Graph.from_adjacency(np.array([[1], [0]])), [0], [1]).objective == 1
    r = P.plan((g, np.array([0]), np.array([3])), "makespan", split=np.int64(2), backend="builtin")
    assert r.paths is not None
    assert P.plan_metrics(np.array(r.paths), np.array([3]))["makespan"] == 3
    for bad in ([True], [0.0]):
        with pytest.raises(TypeError):
            S.min_makespan(g, bad, [3])
    with pytest.raises(TypeError):
        P.plan((g, [0], [3]), split=True)


@pytest.mark.parametrize("budget", [0, -5, float("nan")])
def test_nonpositive_feasibility_budget_raises(budget):
    """Regression (C-BUDGET-NONPOSITIVE-HANG): a budget <= 0 left every joint-state proof
    inconclusive, and the makespan loop on the path swap never ended."""
    g = Graph.from_adjacency([[1], [0, 2], [1]])
    for f in (lambda: S.min_makespan(g, [0, 2], [2, 0], feasibility_budget=budget),
              lambda: S.min_total_time(g, [0, 2], [2, 0], feasibility_budget=budget),
              lambda: P.plan((g, [0, 2], [2, 0]), feasibility_budget=budget),
              lambda: S.solve_time(g, [0, 2], [2, 0], compat="original", feasibility_budget=budget)):
        with pytest.raises(ValueError, match="feasibility_budget"):
            f()
    with pytest.raises(Infeasible):
        S.min_makespan(g, [0, 2], [2, 0], feasibility_budget=1)


@pytest.mark.parametrize("gap", [-0.1, float("nan"), float("inf")])
def test_invalid_gap_raises(gap):
    """Regression (C-GAP-NOT-VALIDATED): a negative gap raised in Gurobi but only warned in
    HiGHS; NaN and inf were accepted silently (Gurobi then stopped at 69 for an optimum 50)."""
    g = Graph.from_adjacency([[1], [0, 2], [1]])
    for f in (S.min_total_time, S.min_total_distance, S.min_max_distance):
        with pytest.raises(ValueError, match="gap"):
            f(g, [0], [2], gap=gap)
    with pytest.raises(ValueError, match="gap"):
        P.plan((g, [0], [2]), "total_time", gap=gap)


def test_unreachable_goal_is_reported_before_t_max():
    """Regression (C-JS-PY-INFEASIBLE-MESSAGE-ORDER): min_total_time's single horizon checked
    t_max before reachability, unlike min_total_distance and the JS engine."""
    g = Graph.from_adjacency([[4], [2, 5], [1, 4], [], [0, 2], [1]])
    for f in OBJECTIVE_FNS:
        with pytest.raises(Infeasible, match="goal of robot 1 is unreachable"):
            f(g, [1, 4, 3, 0], [4, 3, 1, 2], T=5, horizon="single", t_max=0, backend="builtin")


# ------------------------------------------------- round 3: infeasibility, gaps, horizons
def _corridor(L):
    return Graph.from_adjacency([[v for v in (u - 1, u + 1) if 0 <= v < L] for u in range(L)])


def _two_grids_with_bridge():
    """Two 3x3 grids (0-8, 9-17) joined by the bridge 8 - 9."""
    edges = []
    for off in (0, 9):
        for r in range(3):
            for c in range(3):
                v = off + 3 * r + c
                if c < 2:
                    edges.append((v, v + 1))
                if r < 2:
                    edges.append((v, v + 3))
    return from_edges(18, edges + [(8, 9)])


def test_structural_certificates_prove_large_infeasible_instances():
    """Regression (F1C-INFEASIBLE-UNDETECTED-LARGE-JOINT-SPACE, C2B-INFEASIBLE-CORRIDOR-NEVER-
    DECIDED): the joint-state search alone was inconclusive on these (about 10^11 joint
    configurations for the first), so min_makespan tried T = T_min, T_min + 1, ... until the
    time limit (or, without one, practically forever)."""
    grid = create_2d_grid_graph(5, 5)
    big = Graph.from_adjacency([list(grid.adj[v]) for v in range(25)] + [[26], [25]])
    s, t = [0, 4, 20, 24, 12, 7, 25, 26], [24, 20, 4, 0, 2, 22, 26, 25]  # robots 6, 7 swap on 25 - 26
    bridge = _two_grids_with_bridge()
    bg = list(range(18))
    bg[8], bg[9] = 9, 8
    cases = [(big, s, t), (_corridor(16), list(range(5)), [15, 14, 13, 12, 11]),
             (_corridor(30), list(range(8)), [29 - i for i in range(8)]), (bridge, list(range(18)), bg)]
    for g, s_, t_ in cases:
        assert S.joint_feasibility(g, s_, t_) is False
        with pytest.raises(Infeasible, match="joint-state search"):
            S.min_makespan(g, s_, t_)
    with pytest.raises(Infeasible):
        P.plan((bridge, list(range(18)), bg), "makespan", split=2)
    # a pendant vertex on a fully occupied 3x5 grid: its robot can never leave
    g35 = create_2d_grid_graph(3, 5)
    adj = [list(g35.adj[v]) for v in range(15)] + [[0]]
    adj[0].append(15)
    gp = Graph.from_adjacency(adj)
    goals = list(range(16))
    goals[0], goals[15] = 15, 0
    assert S.joint_feasibility(gp, list(range(16)), goals) is False
    # cycles: rotations keep the cyclic order (feasible), a transposition does not
    gc = Graph.from_adjacency([[(u - 1) % 6, (u + 1) % 6] for u in range(6)])
    assert S.joint_feasibility(gc, [0, 2, 4], [2, 4, 0]) is True
    assert S.joint_feasibility(gc, [0, 2, 4], [2, 0, 4]) is False
    # a fully occupied ring rotates (the centre stays): feasible
    assert S.joint_feasibility(create_2d_grid_graph(3, 3), [0, 1, 2, 5, 8, 7, 6, 3, 4],
                               [1, 2, 5, 8, 7, 6, 3, 0, 4]) is True


def test_structural_certificates_are_sound(monkeypatch):
    """Every certificate (and the per-component search) agrees with the exact joint-state
    search on random small instances, with one-way arcs, missing wait loops, t_first and one
    move per step."""
    import random
    R = random.Random(7)
    hits = 0
    for _ in range(600):
        n = R.randint(2, 7)
        if R.random() < 0.4:
            E = {(u, u + 1) for u in range(n - 1) if R.random() < 0.85} | ({(0, n - 1)} if R.random() < 0.4 else set())
        else:
            E = {(u, v) for u in range(n) for v in range(u + 1, n) if R.random() < 0.3}
        oneway, loops = R.random() < 0.25, R.random() < 0.7
        adj = {v: set() for v in range(n)}
        for u, v in E:
            if oneway and R.random() < 0.5:
                (adj[u].add(v) if R.random() < 0.5 else adj[v].add(u))
            else:
                adj[u].add(v)
                adj[v].add(u)
        for v in range(n):
            if loops or R.random() < 0.3:
                adj[v].add(v)
        g = Graph()
        for v in range(n):
            g.xy[v] = None
        g.adj = {v: sorted(a) for v, a in adj.items()}
        g.finish_building_graph()
        A = R.randint(1, n)
        st, go = R.sample(range(n), A), R.sample(range(n), A)
        tf, one = R.choice([0, 0, R.randint(0, 5)]), R.random() < 0.2
        if any(d is None for d in S._distances(g, st, go)):
            continue
        und = S._undirected(g, st + go)
        comp, comps = S._components(und)
        cert = (S._structural_refutation(und, comp, comps, st, go) is not None
                or S._component_refutes(g, comp, comps, st, go, 10**6, one, tf))
        with monkeypatch.context() as m:
            m.setattr(S, "_structural_refutation", lambda *a: None)
            m.setattr(S, "_component_refutes", lambda *a: False)
            exact = S._joint_bfs(g, st, go, 10**6, one, tf)[0]
        hits += cert
        assert not cert or exact is False, (g.adj, st, go, tf, one)
        assert S._joint_bfs(g, st, go, 10**6, one, tf)[0] == exact
    assert hits > 50


def test_joint_checks_keep_pace_with_slow_models(monkeypatch):
    """Regression (C2B-INFEASIBLE-CORRIDOR-NEVER-DECIDED), without the certificates: the search
    with 800000 work units settles the corridor in 0.2 s, but the doubling schedule reached that
    budget only at the 16th failed horizon (T = 30), after ILP solves of over 90 s per horizon
    from T = 23 on. The checks are now also run when the models took longer than the checks."""
    monkeypatch.setattr(S, "_structural_refutation", lambda *a: None)
    g = _corridor(16)
    s, t = list(range(5)), [15, 14, 13, 12, 11]
    assert S.joint_feasibility(g, s, t) is None
    with pytest.raises(Infeasible, match="joint-state search"):
        S.min_makespan(g, s, t, time_limit=300)


_TT_GAP_CASE = (create_2d_grid_graph(6, 6),
                [8, 4, 16, 7, 31, 24, 14, 15, 20, 12, 25, 6, 3, 28, 0, 26, 13, 21, 29, 35],
                [8, 4, 16, 29, 31, 24, 14, 7, 20, 12, 26, 6, 3, 28, 0, 15, 13, 21, 25, 35])


def test_tt_gap_applies_to_the_total_time(monkeypatch):
    """Regression (F1C-TT-GAP-APPLIED-TO-MODEL-OBJECTIVE): the gap reached the backend unchanged,
    as a gap on the model's objective n*T - TT; with gap 0.1, Gurobi stopped at TT 48 against
    a TT bound of 29 (40%). The backend now gets gap * sum(d) / (n*T - sum(d))."""
    g, st, go = _TT_GAP_CASE
    lb = sum(S._distances(g, st, go))
    seen = []
    real = S._run

    def spy(m, backend, params, tl, gap):
        seen.append(gap)
        return real(m, backend, params, tl, gap)
    with monkeypatch.context() as m:
        m.setattr(S, "_run", spy)
        S.min_total_time(g, st, go, T=20, horizon="single", gap=0.1)
    assert seen == [pytest.approx(0.1 * lb / (20 * len(st) - lb))]
    assert S._tt_model_gap(0.0, 3, 5, 4) == 0 and S._tt_model_gap(0.2, 3, 5, 0) == 0


@needs_gurobi
def test_tt_gap_holds_on_the_total_time_with_gurobi():
    """The Gurobi half of :func:`test_tt_gap_applies_to_the_total_time`: the returned total time
    is within ``gap`` of the bound on the total time."""
    g, st, go = _TT_GAP_CASE
    for gap in (0.1, 0.3):
        r = S.min_total_time(g, st, go, T=20, horizon="single", gap=gap, backend="gurobi", params=GRB)
        bound = 20 * len(st) - r.attempts[-1]["bound"]
        assert (r.objective - bound) / r.objective <= gap + 1e-9, (gap, r.objective, bound)


def test_params_tolerance_does_not_prove_optimality(monkeypatch):
    """Regression (C2B-PARAMS-GAP-FALSE-OPTIMAL): with gap=0 the backend's "optimal" was taken as
    a proof, but a tolerance in params (Gurobi MIPGapAbs, HiGHS mip_abs_gap) stops it early;
    plan() then reported "optimal" for a total time of 171 while 166 exists. The bound must now
    close. Simulated: the backend says "optimal" with an open bound."""
    real = S._run

    def loose(m, backend, params, tl, gap):
        sol = real(m, backend, params, tl, gap)
        if sol.status == "optimal" and sol.bound is not None:
            sol.bound += 3.0 if m.model.sense == "max" else -3.0
        return sol
    g = create_2d_grid_graph(3, 4)
    with monkeypatch.context() as m:
        m.setattr(S, "_run", loose)
        for f in OBJECTIVE_FNS:
            r = f(g, [0], [11], T=5, horizon="single")  # meets the lower bound 5, bound opened by 3
            assert r.status == "feasible", f.__name__
            # makespan: "optimal" below n with a bound that admits n decides nothing
    def short(m, backend, params, tl, gap):
        sol = real(m, backend, params, tl, gap)
        return S.lp.Solution(status="optimal", objective=sol.objective - 1, x=sol.x, bound=sol.objective,
                             backend="fake") if sol.has_solution else sol
    with monkeypatch.context() as m:
        m.setattr(S, "_run", short)
        r = S.min_makespan(g, [0, 1], [11, 10])
        assert r.paths is None and r.status == "time_limit" and r.extra["solver_status"] == "optimal"


def test_params_cannot_override_the_gap_argument():
    """C2B-PARAMS-GAP-FALSE-OPTIMAL: HiGHS let params={"mip_rel_gap": x} override gap=, Gurobi
    and builtin did not; the explicit argument now wins on every backend."""
    from mppflow.lp import highs as H
    seen = {}

    class Stop(Exception):
        pass

    import scipy.optimize as so
    real = so.milp

    def fake(*a, **kw):
        seen.update(kw.get("options") or {})
        raise Stop()
    so.milp = fake
    try:
        m = S.build_distance_model(create_2d_grid_graph(2, 2), [0], [3], 2, True, None)
        with pytest.raises(Stop):
            H.solve(m.model, mip_gap=0.0, time_limit=5.0, params={"mip_rel_gap": 0.95, "time_limit": 99,
                                                                  "isolate": False})
    finally:
        so.milp = real
    assert seen["mip_rel_gap"] == 0.0 and 4.0 < seen["time_limit"] <= 5.0


def test_single_horizon_without_goal_wait_loops_says_exactly_T():
    """Regression (F1C-NOWAIT-SINGLE-HORIZON-FALSE-INFEASIBLE): without a wait loop at a goal
    the feasible horizons are not upward-closed; horizon="single" claimed "no plan with makespan
    <= T" (or <= t_max) although [[0, 1]] (makespan 1) is a plan."""
    g = Graph()
    for v in range(2):
        g.xy[v] = None
    g.adj = {0: [1], 1: [0]}
    g.finish_building_graph()
    for f in OBJECTIVE_FNS:
        with pytest.raises(Infeasible, match="makespan exactly T = 2"):
            f(g, [0], [1], T=2, horizon="single")
        with pytest.raises(Infeasible, match="makespan exactly t_max = 2"):
            f(g, [0], [1], T=3, t_max=2, horizon="single")
        r = f(g, [0], [1], t_max=2)
        assert (r.status, r.objective, r.paths) == ("optimal", 1, [[0, 1]])


@pytest.mark.parametrize("kw", [{"t_max": -1}, {"t_max": -1, "T": 3}, {"T": -1, "horizon": "single"}])
def test_negative_horizon_arguments_raise(kw):
    """Regression (F1C-NEGATIVE-T-TMAX-UNVALIDATED): a negative t_max gave Infeasible, a
    ValueError naming T, or (no robots) an "optimal" empty plan, depending on the path."""
    g = Graph.from_adjacency([[1], [0, 2], [1]])
    what = "t_max" if "t_max" in kw else "T"
    for s, t in (([0], [2]), ([], [])):
        for f in OBJECTIVE_FNS:
            with pytest.raises(ValueError, match=f"^{what} must be >= 0"):
                f(g, s, t, **kw)
        if "T" not in kw:
            with pytest.raises(ValueError, match="^t_max must be >= 0"):
                S.min_makespan(g, s, t, **kw)
