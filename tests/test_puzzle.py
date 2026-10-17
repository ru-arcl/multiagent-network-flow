"""Tests of mppflow.puzzle: the PuzzleSolver golden records under Gurobi (Threads=1), the
deterministic emulation of the two-model race (which plan is returned, the deadlock), the
fixed-mode corrections (no deadlock, Infeasible, t_max, time limits), solve_problem's
dispatch, and agreement with the MINMAKESPAN solver."""
from __future__ import annotations

import os

import pytest

from mppflow import puzzle as P
from mppflow import solvers as S
from mppflow._java import MathRandom
from mppflow.errors import Infeasible, JavaNullPointerException, OriginalWouldHang
from mppflow.graph import Graph, create_2d_grid_graph, create_main_case3_graph
from mppflow.problem import Problem, create_grid_problem, create_n2_puzzle
from helpers import GRB, graph_from_json, load_golden, needs_gurobi  # noqa: F401


HERE = os.path.dirname(__file__)
GOLDEN = os.path.join(HERE, "golden")


RECS = {r["id"]: r for r in load_golden("solvers.jsonl") if r["fn"].startswith("PuzzleSolver")}
MODEL_RECS = {r["id"]: r for r in load_golden("models.jsonl") if r["fn"].startswith("PuzzleSolver")}


def check_plan(g, starts, goals, paths):
    L = len(paths[0])
    assert [p[0] for p in paths] == list(starts)
    assert [p[-1] for p in paths] == list(goals)
    for t in range(L):
        assert len({p[t] for p in paths}) == len(paths)
        if t:
            for p in paths:
                assert p[t] in g.adj[p[t - 1]]
            mv = {(p[t - 1], p[t]) for p in paths if p[t - 1] != p[t]}
            assert not any((b, a) in mv for a, b in mv)


def run_record(r, compat, **kw):
    a = r["args"]
    rng = MathRandom(r["seed"])
    if r["fn"] == "PuzzleSolver.solve(double)":
        ps = P.PuzzleSolver(graph_from_json(a["graph"]), a["starts"], a["goals"], compat=compat, backend="gurobi",
                            params=GRB, rng=rng, **kw)
        return ps.solve(a["timeLimit"]), rng
    p = Problem(graph_from_json(a["p"]["graph"]), a["p"]["sg"][0], a["p"]["sg"][1])
    st = r.get("statics") or {}
    return P.solve_problem(p, a["timeLimit"], compat=compat, backend="gurobi", params=GRB, rng=rng,
                           debug=st.get("MultiagentGraphSolverGurobiTime.bDebugInfo", True),
                           print_paths=st.get("MultiagentGraphSolverGurobiTime.bPrintPaths", True)), rng


PLAN_IDS = [
    "solvers/PuzzleSolver.solve(double)/3x3",
    "solvers/PuzzleSolver.solve(Problem,double)/n2p3_s0",
    pytest.param("solvers/PuzzleSolver.solve(Problem,double)/n2p4_s1", marks=pytest.mark.slow),
    "solvers/PuzzleSolver.solve(Problem,double)/n2m1p3_s2",
]


@needs_gurobi
@pytest.mark.gurobi
@pytest.mark.parametrize("compat", ["original", "fixed"])
@pytest.mark.parametrize("rid", PLAN_IDS)
def test_golden_plans(rid, compat):
    """Plans, printed text, draws and the next draw equal the original's (both modes). The
    3x3 record is marked nondeterministic (the race picks the plan); ours is instance 2's."""
    r = RECS[rid]
    res, rng = run_record(r, compat)
    assert res.paths == r["result"]
    assert res.log == r["stdout"]
    assert rng.draws == r["draws"]
    assert rng.random() == r["rng_next"]
    assert res.status == "optimal"
    if "PuzzleSolver.solve(double)" in rid:
        assert res.extra["good_solution"] == 2


@needs_gurobi
@pytest.mark.gurobi
def test_golden_deadlock_star():
    """Star swap, time limit 10: both models fail at T = 2 within the first poll and the
    original waits forever (live Java hung; original-bugs B17). Fixed moves on
    and returns the optimal makespan 4, as MINMAKESPAN does."""
    r = RECS["solvers/PuzzleSolver.solve(double)/star_deadlock"]
    assert r.get("hang") is True
    with pytest.raises(OriginalWouldHang) as ei:
        run_record(r, "original")
    assert ei.value.output == r["stdout"]
    for race in ("fast", "measured"):
        with pytest.raises(OriginalWouldHang):
            run_record(r, "original", race=race)
    res, rng = run_record(r, "original", race="slow")  # a worker still running at the poll
    assert res.makespan == 4
    res, rng = run_record(r, "fixed")
    a = r["args"]
    g = graph_from_json(a["graph"])
    check_plan(g, a["starts"], a["goals"], res.paths)
    assert res.makespan == 4 and res.lower_bound == 2
    assert [x["T"] for x in res.attempts] == [2, 2, 3, 3, 4, 4]
    ref = S.solve_time(g, a["starts"], a["goals"], rng=MathRandom(1), backend="gurobi", params=GRB)
    assert ref.makespan == res.makespan


@pytest.mark.parametrize("compat", ["original", "fixed"])
def test_update_reachable_sets_golden(compat):
    r = MODEL_RECS["models/PuzzleSolver/updateReachableSets_star_T3"]
    a = r["args"]
    ps = P.PuzzleSolver(graph_from_json(a["graph"]), a["starts"], a["goals"], compat=compat)
    reach = ps.update_reachable_sets(a["timeSteps"])
    assert reach.vertex_lists() == r["result"]
    assert ps.reachable_vertices is reach


@pytest.mark.parametrize("models", [(1,), (2,), (1, 2), (2, 1)])
def test_models_choice_same_makespan(models):
    """Both models have a plan of value n at exactly the same horizons."""
    g = create_main_case3_graph()
    res = P.PuzzleSolver(g, [0, 3], [3, 0], models=models, rng=MathRandom(1)).solve()
    check_plan(g, [0, 3], [3, 0], res.paths)
    assert res.makespan == 4
    assert res.extra["good_solution"] == (2 if 2 in models else 1)


def test_highs_random_grids_match_minmakespan():
    for seed in range(6):
        p = create_grid_problem(4, 4, 0.1, 6, MathRandom(seed))
        res = P.PuzzleSolver(p.graph, p.starts, p.goals, rng=MathRandom(seed)).solve()
        check_plan(p.graph, p.starts, p.goals, res.paths)
        ref = S.solve_time(p.graph, p.starts, p.goals, rng=MathRandom(seed))
        assert res.makespan == ref.makespan
        assert res.lower_bound == ref.lower_bound


def test_start_time_steps_persists():
    """fixed: a second solve() starts at the stored horizon (the original's field) and is
    optimal again. original: the stale optimizationDone aborts the first horizon of the second
    call, which returns makespan T + 1 (see test_original_stale_optimization_done)."""
    g = create_main_case3_graph()
    ps = P.PuzzleSolver(g, [0, 3], [3, 0], rng=MathRandom(1))
    assert ps.solve().makespan == 4 and ps.start_time_steps == 4
    res = ps.solve()
    assert res.makespan == 4 and [x["T"] for x in res.attempts] == [4, 4]
    ps = P.PuzzleSolver(g, [0, 3], [3, 0], rng=MathRandom(1), compat="original", race="slow")
    assert ps.solve().makespan == 4 and ps.start_time_steps == 4 and ps.optimization_done
    res = ps.solve()
    assert res.makespan == 5 and [x["T"] for x in res.attempts] == [4, 4, 5, 5]
    assert [x["status"] for x in res.attempts[:2]] == ["interrupted", "interrupted"]
    check_plan(g, [0, 3], [3, 0], res.paths)


def _swap_line():
    return Graph.from_adjacency([[1], [0, 2], [1]])  # 0-1-2, two robots swap ends: infeasible


def test_infeasible_instance():
    g = _swap_line()
    with pytest.raises(Infeasible):
        P.PuzzleSolver(g, [0, 2], [2, 0]).solve()
    # original: race "slow" avoids the deadlock, then the endless T loop is detected
    with pytest.raises(OriginalWouldHang):
        P.PuzzleSolver(g, [0, 2], [2, 0], compat="original", race="slow").solve()
    res = P.PuzzleSolver(g, [0, 2], [2, 0], compat="original", race="slow").solve(1000.0)
    assert res.paths is None and res.status == "time_limit" and res.extra["shortcut"] is True
    # the deadlock comes first with fast workers, whatever the time limit
    with pytest.raises(OriginalWouldHang):
        P.PuzzleSolver(g, [0, 2], [2, 0], compat="original", race="fast").solve(1000.0)
    # ... unless the limit has already expired at the first poll
    res = P.PuzzleSolver(g, [0, 2], [2, 0], compat="original", race="fast").solve(0.5)
    assert res.paths is None and res.status == "time_limit"


def test_t_max_and_checks():
    g = create_main_case3_graph()
    with pytest.raises(Infeasible):
        P.PuzzleSolver(g, [0, 3], [3, 0], t_max=3).solve()
    with pytest.raises(Infeasible):
        P.PuzzleSolver(g, [0, 3], [3, 0], t_max=1).solve()
    assert P.PuzzleSolver(g, [0, 3], [3, 0], t_max=4).solve().makespan == 4
    with pytest.raises(ValueError):
        P.PuzzleSolver(g, [0, 0], [3, 2]).solve()
    with pytest.raises(Infeasible):
        P.PuzzleSolver(g, [0, 2], [3, 3]).solve()
    with pytest.raises(ValueError):
        P.PuzzleSolver(g, [0], [3], models=(3,))
    with pytest.raises(ValueError):
        P.PuzzleSolver(g, [0], [3], race="random")


def test_unreachable_goal():
    g = Graph.from_adjacency([[1], [0], [2]])
    rng = MathRandom(1)
    with pytest.raises(Infeasible):
        P.PuzzleSolver(g, [0], [2], rng=rng).solve()
    assert rng.draws == 0
    # fixed raises before any draw, also when other robots draw first
    g2 = Graph.from_adjacency([[1, 3], [0, 2, 4], [1, 5], [0, 4], [1, 3, 5], [2, 4], [7], [6]])
    rng = MathRandom(1)
    with pytest.raises(Infeasible):
        P.PuzzleSolver(g2, [0, 1], [5, 7], rng=rng).solve()
    assert rng.draws == 0
    with pytest.raises(JavaNullPointerException):
        P.PuzzleSolver(g, [0], [2], compat="original", rng=MathRandom(1)).solve()


def test_time_limit_fake_clock():
    """A clock that jumps 100 s per reading: the limit has expired before the first model."""
    g = create_main_case3_graph()
    ticks = iter(range(0, 10 ** 6, 100))
    res = P.PuzzleSolver(g, [0, 3], [3, 0], clock=lambda: float(next(ticks))).solve(50.0)
    assert res.paths is None and res.status == "time_limit"
    ticks = iter(range(0, 10 ** 6, 100))
    res = P.PuzzleSolver(g, [0, 3], [3, 0], compat="original", race="slow",
                         clock=lambda: float(next(ticks))).solve(50.0)
    assert res.paths is None and res.status == "time_limit"


@pytest.mark.parametrize("compat", ["original", "fixed"])
def test_zero_agents(compat):
    res = P.PuzzleSolver(create_main_case3_graph(), [], [], compat=compat).solve()
    assert res.paths == [] and res.log == "\n"


def test_solve_problem_dispatch():
    """More than 16 robots go to the portfolio, 16 or fewer to planPathsAdvanced."""
    p = create_grid_problem(6, 6, 0.0, 17, MathRandom(3))
    res = P.solve_problem(p, -1.0, rng=MathRandom(3))
    assert "good_solution" in res.extra and res.status == "optimal"
    check_plan(p.graph, p.starts, p.goals, res.paths)
    ref = S.solve_time(p.graph, p.starts, p.goals, rng=MathRandom(3))
    assert res.makespan == ref.makespan
    p16 = Problem(p.graph, p.starts[:16], p.goals[:16])
    res16 = P.solve_problem(p16, -1.0, rng=MathRandom(3))
    assert "good_solution" not in res16.extra and res16.status == "optimal"
    check_plan(p.graph, p16.starts, p16.goals, res16.paths)
    # debug text of the Time solver (bDebugInfo / bPrintPaths default to true in Java)
    res16d = P.solve_problem(p16, -1.0, rng=MathRandom(3), debug=True, print_paths=True)
    assert "Trying T = " in res16d.log and "Agent 0: 0:" in res16d.log
    assert P.PUZZLE_SOLVER_MIN_AGENTS == 17


def test_solve_problem_null_sg():
    p = Problem(create_main_case3_graph())
    with pytest.raises(JavaNullPointerException):
        P.solve_problem(p, -1.0, compat="original")
    with pytest.raises(ValueError):
        P.solve_problem(p, -1.0)


@pytest.mark.slow
def test_paper_9_puzzle_portfolio():
    """Random 9-puzzles through the portfolio match MINMAKESPAN (T-RO 2016 Sec. VI-B: N^2
    puzzles are always solvable for N >= 3)."""
    for seed in range(3):
        p = create_n2_puzzle(3, MathRandom(seed))
        res = P.PuzzleSolver(p.graph, p.starts, p.goals, rng=MathRandom(seed), models=(2,)).solve()
        check_plan(p.graph, p.starts, p.goals, res.paths)
        ref = S.solve_time(p.graph, p.starts, p.goals, rng=MathRandom(seed))
        assert res.makespan == ref.makespan


def test_grid_puzzle_one_way_original_pruning():
    """On a one-way 4-cycle without wait loops the original's pruning never admits a plan;
    original mode reports the endless loop, fixed solves it (makespan 3)."""
    g = Graph()
    for v in range(4):
        g.xy[v] = None
        g.adj[v] = [(v + 1) % 4]
    g.finish_building_graph()
    res = P.PuzzleSolver(g, [0], [3]).solve()
    assert res.makespan == 3 and res.paths == [[0, 1, 2, 3]]
    with pytest.raises(OriginalWouldHang):
        P.PuzzleSolver(g, [0], [3], compat="original", race="slow").solve()


def test_grid_8x8_highs_sanity():
    g = create_2d_grid_graph(3, 3)
    res = P.PuzzleSolver(g, [0, 6], [2, 8], rng=MathRandom(1)).solve()
    assert res.paths == [[0, 1, 2], [6, 7, 8]]


# ------------------------------------------------------------ regressions
@needs_gurobi
@pytest.mark.gurobi
@pytest.mark.parametrize("tl, java", [(0.3, None), (0.8, None), (1.5, [[0, 1, 2], [6, 7, 8]])])
def test_original_time_limit_uses_poll_live_java(tl, java):
    """Live Java on the golden 3x3 record: the main thread sleeps 1 s and tests
    elapsed > timeLimit before it looks at the result, so a plan found at once is dropped
    whenever timeLimit < 1 (live Java probe). fixed returns the plan."""
    r = RECS["solvers/PuzzleSolver.solve(double)/3x3"]
    a = r["args"]
    for compat in ("original", "fixed"):
        rng = MathRandom(r["seed"])
        ps = P.PuzzleSolver(graph_from_json(a["graph"]), a["starts"], a["goals"], compat=compat,
                            backend="gurobi", params=GRB, rng=rng)
        res = ps.solve(tl)
        exp = java if compat == "original" else [[0, 1, 2], [6, 7, 8]]
        assert res.paths == exp
        assert res.log == ("" if exp is None else "\n")
        assert res.status == ("time_limit" if exp is None else "optimal")
        assert rng.draws == 13 and rng.random() == r["rng_next"]


@pytest.mark.parametrize("race", ["fast", "measured"])
def test_original_time_limit_uses_poll_highs(race):
    g = create_2d_grid_graph(3, 3)
    res = P.PuzzleSolver(g, [0, 6], [2, 8], compat="original", race=race, rng=MathRandom(1)).solve(0.5)
    assert res.paths is None and res.status == "time_limit"
    res = P.PuzzleSolver(g, [0, 6], [2, 8], compat="original", race=race, rng=MathRandom(1)).solve(1.5)
    assert res.paths == [[0, 1, 2], [6, 7, 8]]
    res = P.PuzzleSolver(g, [0, 6], [2, 8], rng=MathRandom(1)).solve(0.5)
    assert res.paths == [[0, 1, 2], [6, 7, 8]]
    # zero robots go through the same poll in original
    assert P.PuzzleSolver(g, [], [], compat="original").solve(0.5).paths is None
    assert P.PuzzleSolver(g, [], [], compat="fixed").solve(0.5).paths == []


@needs_gurobi
@pytest.mark.gurobi
def test_original_stale_optimization_done_live_java():
    """Java probe (seed 13, create2DGridGraph(3,3,true), starts {0,6}, goals {2,8},
    solve(-1) twice): 'R1 [[0, 1, 2], [6, 7, 8]] T=2 done=true', 'R2 [[0, 0, 1, 2],
    [6, 6, 7, 8]] T=3'."""
    out = {}
    for compat in ("original", "fixed"):
        ps = P.PuzzleSolver(create_2d_grid_graph(3, 3), [0, 6], [2, 8], compat=compat, backend="gurobi",
                            params=GRB, rng=MathRandom(13))
        r1 = ps.solve()
        assert r1.paths == [[0, 1, 2], [6, 7, 8]] and ps.start_time_steps == 2 and ps.optimization_done
        r2 = ps.solve()
        out[compat] = (r2.paths, ps.start_time_steps)
    assert out["original"] == ([[0, 0, 1, 2], [6, 6, 7, 8]], 3)
    assert out["fixed"] == ([[0, 1, 2], [6, 7, 8]], 2)


@needs_gurobi
@pytest.mark.gurobi
def test_original_stale_flags_after_time_limit_live_java():
    """Java RP2 (seed 13, create2DGridGraph(3,3,true), starts {0,6}, goals {2,8}, Threads 1):
    solve(0.3) -> 'R1 null T=2 done=true f=truetrue'; solve(-1) -> 'R2 [[0, 1, 2], [6, 7, 8]]
    T=2'. The time-limit return leaves flag1/flag2 and sol1/sol2 set, so the next call
    returns the dropped plan at the same T. The third call is a race in Java: T=3 in one run
    of three, T=4 in the others. We give T=3, the plan Java gave in that run."""
    rng = MathRandom(13)
    ps = P.PuzzleSolver(create_2d_grid_graph(3, 3), [0, 6], [2, 8], compat="original", backend="gurobi",
                        params=GRB, rng=rng)
    r1 = ps.solve(0.3)
    assert r1.paths is None and ps.start_time_steps == 2 and ps.optimization_done
    assert ps.flag1 and ps.flag2 and ps.sol2 == [[0, 1, 2], [6, 7, 8]]
    r2 = ps.solve(-1)
    assert r2.paths == [[0, 1, 2], [6, 7, 8]] and ps.start_time_steps == 2 and r2.log == "\n"
    assert r2.status == "optimal" and r2.extra["stale_result"] is True
    assert [(x["T"], x["status"]) for x in r2.attempts] == [(2, "abandoned"), (2, "abandoned")]
    assert ps.optimization_done and ps.flag1 and ps.flag2 and ps.sol1 is None and ps.sol2 is None
    r3 = ps.solve(-1)
    assert r3.paths == [[0, 0, 1, 2], [6, 6, 7, 8]] and ps.start_time_steps == 3
    # fixed searches afresh: the same plan at T=2 on the second call
    ps = P.PuzzleSolver(create_2d_grid_graph(3, 3), [0, 6], [2, 8], backend="gurobi", params=GRB,
                        rng=MathRandom(13))
    assert ps.solve(0.3).paths == [[0, 1, 2], [6, 7, 8]]
    assert ps.solve(-1).paths == [[0, 1, 2], [6, 7, 8]] and ps.start_time_steps == 2


@pytest.mark.parametrize("race", ["fast", "measured"])
def test_original_stale_flags_after_time_limit_highs(race):
    g = create_2d_grid_graph(3, 3)
    ps = P.PuzzleSolver(g, [0, 6], [2, 8], compat="original", race=race, rng=MathRandom(13))
    assert ps.solve(0.3).paths is None
    r2 = ps.solve(5.0)  # the stale result is returned without a poll, whatever the limit
    assert r2.paths == [[0, 1, 2], [6, 7, 8]] and ps.start_time_steps == 2 and r2.log == "\n"
    assert all(x["status"] == "abandoned" and x["stale_done"] for x in r2.attempts)
    r3 = ps.solve()
    assert r3.makespan == 3 and ps.start_time_steps == 3
    assert [(x["T"], x["status"]) for x in r3.attempts][:2] == [(2, "abandoned"), (2, "abandoned")]
    check_plan(g, [0, 6], [2, 8], r3.paths)


class _FakeSol:
    def __init__(self, status, runtime):
        self.status, self.runtime = status, runtime


def _scripted(ps, script):
    """Replace the backend: script[(T, instance)] = (paths or None, status, runtime)."""
    def solve_model(inst, T, left):
        paths, status, rt = script[(T, inst)]
        return paths, _FakeSol(status, rt), {"T": T, "phase": "ilp", "instance": inst, "status": status,
                                              "objective": None, "bound": None, "runtime": rt, "left": left}
    ps._solve_model = solve_model
    ps._after_failure = lambda T, has: None
    return ps


def test_original_poll_emulation_scripted():
    """The deadlock needs both workers finished before the poll that notices the failure (not
    only before the first poll); a worker still running is aborted; minute dots per poll."""
    g = create_2d_grid_graph(3, 3)
    plan = [[0, 1, 2], [6, 7, 8]]

    def mk():
        return P.PuzzleSolver(g, [0, 6], [2, 8], compat="original", rng=MathRandom(1))

    # both fail after the first poll, both done by the second: hang
    ps = _scripted(mk(), {(2, 1): (None, "infeasible", 1.5), (2, 2): (None, "infeasible", 1.8)})
    with pytest.raises(OriginalWouldHang):
        ps.solve()
    # worker 2 still running when the failure is noticed at poll 1: no hang, T + 1
    ps = _scripted(mk(), {(2, 1): (None, "infeasible", 0.5), (2, 2): (None, "infeasible", 1.5),
                          (3, 1): (plan, "optimal", 0.1), (3, 2): (plan, "optimal", 0.1)})
    res = ps.solve()
    assert res.paths == plan and ps.start_time_steps == 3 and ps.all_thread_finished is False
    # a stale allThreadFinished (an aborted worker of an earlier horizon) prevents the hang
    ps = _scripted(mk(), {(2, 1): (None, "infeasible", 0.5), (2, 2): (None, "infeasible", 0.6),
                          (3, 1): (plan, "optimal", 0.1), (3, 2): (plan, "optimal", 0.1)})
    ps.all_thread_finished = True
    assert ps.solve().paths == plan
    # instance 1 succeeds at 0.5 s, instance 2 still running at poll 1: instance 1's plan
    plan2 = [[0, 3, 4, 5, 2], [6, 7, 8, 8, 8]]
    ps = _scripted(mk(), {(2, 1): (plan, "optimal", 0.5), (2, 2): (plan2, "optimal", 3.0)})
    res = ps.solve()
    assert res.paths == plan and res.extra["good_solution"] == 1 and ps.all_thread_finished
    # a 125-s solve: one dot at the polls of minute 1 and 2, then the final newline
    ps = _scripted(mk(), {(2, 1): (plan, "optimal", 125.0), (2, 2): (plan, "optimal", 125.0)})
    assert ps.solve().log == "..\n"
    # with a time limit of 2.5 s the third poll returns null; the backend got 3 polls' time
    ps = _scripted(mk(), {(2, 1): (None, "time_limit", 3.0), (2, 2): (None, "time_limit", 3.0)})
    res = ps.solve(2.5)
    assert res.paths is None and res.status == "time_limit"
    assert [x["left"] for x in res.attempts] == [3.0, 3.0]


def test_original_stale_flags_scripted():
    """What a time-limit return leaves for the next call: the finished workers' sols and
    goodSolution, failedAttempt, allThreadFinished from an aborted running worker."""
    g = create_2d_grid_graph(3, 3)
    plan = [[0, 1, 2], [6, 7, 8]]
    plan3 = [[0, 0, 1, 2], [6, 6, 7, 8]]

    def mk():
        return P.PuzzleSolver(g, [0, 6], [2, 8], compat="original", rng=MathRandom(1))

    # instance 1 found a plan, instance 2 still running at the null poll: sol1 is returned next
    ps = _scripted(mk(), {(2, 1): (plan, "optimal", 0.5), (2, 2): (None, "time_limit", 1.0)})
    assert ps.solve(0.5).paths is None
    assert ps.good_solution == 1 and ps.sol1 == plan and ps.sol2 is None and ps.all_thread_finished
    res = ps.solve()
    assert res.paths == plan and res.extra["good_solution"] == 1 and len(res.attempts) == 2
    # both still running: null sols, the next call moves on to T + 1 at once
    ps = _scripted(mk(), {(2, 1): (None, "time_limit", 1.0), (2, 2): (None, "time_limit", 1.0),
                          (3, 1): (plan3, "optimal", 0.1), (3, 2): (plan3, "optimal", 0.1)})
    assert ps.solve(0.5).paths is None and ps.flag1 and ps.flag2 and ps.all_thread_finished
    res = ps.solve()
    assert res.paths == plan3 and [x["status"] for x in res.attempts] == ["abandoned", "abandoned",
                                                                         "optimal", "optimal"]
    assert not ps.flag1 and not ps.flag2
    # both failed before the null poll: failedAttempt stays set, so T + 1's first poll waits for
    # allThreadFinished; with both T + 1 workers done by then it never comes
    script = {(2, 1): (None, "infeasible", 0.2), (2, 2): (None, "infeasible", 0.2),
              (3, 1): (plan3, "optimal", 0.1), (3, 2): (plan3, "optimal", 0.1)}
    ps = _scripted(mk(), script)
    assert ps.solve(0.5).paths is None and ps.failed_attempt and not ps.all_thread_finished
    with pytest.raises(OriginalWouldHang):
        ps.solve()
    # ... unless a T + 1 worker is still running at that poll: it is aborted, the other's plan
    ps = _scripted(mk(), {**script, (3, 2): (plan3, "optimal", 1.5)})
    ps.solve(0.5)
    res = ps.solve()
    assert res.paths == plan3 and res.extra["good_solution"] == 1 and not ps.failed_attempt


def test_params_tolerance_does_not_refute_horizon(monkeypatch):
    """Regression (C2D-PUZZLE-REFUTES-IGNORES-TOLERANCE): an "optimal" below n whose bound still
    admits n (a tolerance in params, such as Gurobi's MIPGapAbs, stopped the solve) proved T
    infeasible in the puzzle loop and in solve_time_puzzle; with models=(1,) the loop skipped
    feasible horizons and reported a far too large makespan as "optimal". Simulated: the
    backend stops one unit short of n with the bound at the true value."""
    from mppflow import lp

    real = lp.solve

    def short(model, *a, **kw):
        sol = real(model, *a, **kw)
        if not sol.has_solution:
            return sol
        return lp.Solution(status="optimal", objective=sol.objective - 1, x=sol.x, bound=sol.objective,
                           backend="fake")
    p = create_n2_puzzle(3, MathRandom(2))
    T = S.min_makespan(p.graph, p.starts, p.goals).objective
    monkeypatch.setattr(lp, "solve", short)
    r = P.PuzzleSolver(p.graph, p.starts, p.goals, rng=MathRandom(5), models=(1,)).solve()
    assert r.paths is None and r.status == "time_limit" and r.extra["solver_status"] == "optimal"
    assert max(a["T"] for a in r.attempts) == T  # stopped at the optimum, did not move on to T + 1
    r = S.solve_time_puzzle(p.graph, p.starts, p.goals, False, T)
    assert r.paths is None and r.status == "time_limit" and r.extra["solver_status"] == "optimal"
    monkeypatch.setattr(lp, "solve", real)
    # a genuine refutation still moves on: below the optimum the bound itself is below n
    r = S.solve_time_puzzle(p.graph, p.starts, p.goals, False, T - 1)
    assert r.status == "infeasible"
