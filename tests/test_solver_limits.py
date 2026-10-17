"""Solver limits in fixed mode: a node or solution limit (``params``), an interrupted Gurobi
solve, or a solver stop without a solution never counts as a proof of infeasibility; the
makespan loop does not prove horizons it can settle by the joint-state search; horizon
arguments must be integers; model building counts against ``time_limit``; and the joint
optimum refuses more robots than the JS port's 32-bit robot sets hold."""
from __future__ import annotations

import math
import time

import pytest

import mppflow
from mppflow import solvers as S
from mppflow._java import MathRandom
from mppflow.lp.linear import Solution
from mppflow.model import BuildStopped, build_time_model
from helpers import needs_gurobi  # noqa: F401


def _grid5():
    return mppflow.create_grid_problem(5, 5, 0.1, 10, MathRandom(1))  # optimal makespan 6


@pytest.mark.parametrize("backend", ["highs", "builtin"])
@pytest.mark.parametrize("fn", [mppflow.min_total_time, mppflow.min_total_distance, mppflow.min_max_distance])
def test_node_limit_single_horizon_is_undecided(fn, backend):
    p = _grid5()
    r = fn(p.graph, p.starts, p.goals, T=8, horizon="single", backend=backend, params={"node_limit": 0})
    assert r.paths is None and r.status == "time_limit"  # formerly Infeasible("no plan with makespan <= T = 8")
    assert [a["status"] for a in r.attempts] == ["node_limit"]


@pytest.mark.parametrize("backend", ["highs", "builtin"])
def test_node_limit_stops_the_makespan_loop(backend):
    """Formerly every horizon ended at the node limit and the loop tried T + 1 forever."""
    p = _grid5()
    t0 = time.time()
    r = mppflow.min_makespan(p.graph, p.starts, p.goals, backend=backend, params={"node_limit": 0})
    assert time.time() - t0 < 30
    assert r.paths is None and r.status == "time_limit" and r.extra["solver_status"] == "node_limit"
    assert {a["T"] for a in r.attempts} == {6}
    pr = mppflow.plan(p, "makespan", backend=backend, params={"node_limit": 0})  # with the LP stage
    assert pr.paths is None and pr.status == "time_limit"


def test_undecided_T_hi_is_not_a_proof(monkeypatch):
    """The total-time search at T_hi ends at a node limit: the plan at T0 is not proved optimal."""
    p = mppflow.create_grid_problem(3, 4, 0.0, 6, MathRandom(35))
    monkeypatch.setattr(S, "JOINT_SEARCH_MAX_CONFIGS", 0)
    ref = mppflow.min_total_time(p.graph, p.starts, p.goals)
    T_hi = ref.extra["horizon_search"]["T_hi"]
    assert ref.status == "optimal" and T_hi > ref.extra["horizon_search"]["T0"]
    run = S._run

    def limited(m, backend, params, tl, gap):
        if m.T == T_hi:
            return Solution(status="node_limit", backend="test", raw_status="node limit")
        return run(m, backend, params, tl, gap)

    monkeypatch.setattr(S, "_run", limited)
    r = mppflow.min_total_time(p.graph, p.starts, p.goals)
    assert r.paths is not None and r.status == "feasible"


def test_makespan_loop_takes_the_joint_state_plan():
    """A 2-way split piece of this tree instance fails at T = 2 and is feasible only at T = 6;
    the builtin branch and bound needed minutes for T = 3..6 (exponential in T), the joint-state
    search of the failed horizon gives a plan at T = 6 at once."""
    g = mppflow.from_adjacency([[1, 7], [0, 2, 6], [1, 3, 5], [2, 4], [3], [2], [1], [0]])
    s, t = [4, 6, 1, 3], [2, 6, 0, 7]
    t0 = time.time()
    r = mppflow.plan((g, s, t), "makespan", 2, backend="builtin", rng=MathRandom(1))
    assert time.time() - t0 < 30
    ref = mppflow.plan((g, s, t), "makespan", 2, backend="highs", rng=MathRandom(1))
    assert r.status == "feasible" and r.value == ref.value == 8
    assert mppflow.validate((g, s, t), r.paths) is None
    # one piece: the plan of the joint-state search, optimal
    r1 = S.min_makespan(g, [3, 6, 0, 1], [2, 6, 0, 7], backend="builtin")
    assert r1.status == "optimal" and r1.objective == 6 and r1.extra["proof"] == "joint-state search"
    # the failed ILP at T = 2, then the plan's own horizon (phase "joint": no model solved there)
    assert [(a["T"], a["phase"]) for a in r1.attempts] == [(2, "ilp"), (6, "joint")]
    assert r1.attempts[-1]["runtime"] is None
    assert mppflow.validate((g, [3, 6, 0, 1], [2, 6, 0, 7]), r1.paths) is None
    hi = S.min_makespan(g, [3, 6, 0, 1], [2, 6, 0, 7], backend="highs")
    assert hi.objective == 6


def test_joint_state_plan_respects_t_max():
    g = mppflow.from_adjacency([[1, 7], [0, 2, 6], [1, 3, 5], [2, 4], [3], [2], [1], [0]])
    with pytest.raises(mppflow.Infeasible, match="smallest feasible makespan is 6"):
        S.min_makespan(g, [3, 6, 0, 1], [2, 6, 0, 7], backend="builtin", t_max=5)


@pytest.mark.parametrize("bad", [5.5, 3.5, math.nan, True, 6.0])
def test_horizon_arguments_must_be_integers(bad):
    p = mppflow.create_grid_problem(3, 4, 0.0, 6, MathRandom(35))
    with pytest.raises(TypeError, match="t_max must be an integer"):
        mppflow.min_total_time(p.graph, p.starts, p.goals, t_max=bad)
    with pytest.raises(TypeError, match="T must be an integer"):
        mppflow.min_total_distance(p.graph, p.starts, p.goals, T=bad, horizon="single")
    with pytest.raises(TypeError, match="t_max must be an integer"):
        mppflow.min_makespan(p.graph, p.starts, p.goals, t_max=bad)


def test_numpy_integer_horizon_is_accepted():
    np = pytest.importorskip("numpy")
    p = mppflow.create_grid_problem(3, 4, 0.0, 6, MathRandom(35))
    r = mppflow.min_total_time(p.graph, p.starts, p.goals, t_max=np.int64(7))
    assert r.status == "optimal" and r.objective == 13


def test_builder_stop():
    p = _grid5()
    with pytest.raises(BuildStopped):
        build_time_model(p.graph, p.starts, p.goals, 8, stop=lambda: True)
    m = build_time_model(p.graph, p.starts, p.goals, 8, stop=lambda: False)
    assert m.canonical_text() == build_time_model(p.graph, p.starts, p.goals, 8).canonical_text()


def test_model_building_counts_against_the_time_limit():
    """30x30 grid, 120 robots: the first model has 5 million columns and took 20 s to build
    (plus 10 s of HiGHS at a 1 s limit); now the build stops at the limit."""
    p = mppflow.create_grid_problem(30, 30, 0.1, 120, rng=MathRandom(5))
    for fn in (mppflow.min_makespan, mppflow.min_total_time, mppflow.min_total_distance):
        t0 = time.time()
        r = fn(p.graph, p.starts, p.goals, time_limit=1.0)
        assert r.status == "time_limit" and r.paths is None
        assert time.time() - t0 < 6, fn.__name__


@pytest.mark.parametrize("A,objective", [(31, "total_time"), (31, "distance"), (33, "distance")])
def test_joint_optimum_robot_limit(A, objective):
    """A train of A robots shifting by one on a path: JS keeps robot sets in 32-bit masks, so both
    return None above 30 robots (Python formerly gave 33 here, JS 32 or a bogus plan of value 0)."""
    g = mppflow.from_adjacency([[1]] + [[i - 1, i + 1] for i in range(1, A)] + [[A - 1]])
    assert S.joint_optimum(g, list(range(A)), list(range(1, A + 1)), objective, 2_000_000) is None
    g30 = mppflow.from_adjacency([[1]] + [[i - 1, i + 1] for i in range(1, 30)] + [[29]])
    assert S.joint_optimum(g30, list(range(30)), list(range(1, 31)), "distance", 2_000_000)[0] == 30



@pytest.mark.parametrize("A", [24, 30])
def test_joint_optimum_budget_bounds_closures(A):
    """Regression (C2D-JOINT-OPTIMUM-BUDGET-IGNORES-CLOSURES): with k robots at their goals the
    total-time search enumerates 2^k closure states, and all of them were on the heap before the
    first budget check (24 robots: 23 s and 2.4 GB at budget 1000). Now one enumeration may not
    exceed what is left of the budget."""
    g = mppflow.create_2d_grid_graph(5, 6)
    s, gl = list(range(A)), list(range(A))
    s[0] = 29
    t0 = time.time()
    assert S.joint_optimum(g, s, gl, "total_time", budget=1000) is None
    assert time.time() - t0 < 2

@needs_gurobi
def test_gurobi_stop_without_solution_is_an_error():
    p = _grid5()
    with pytest.raises(RuntimeError, match="MEM_LIMIT"):
        mppflow.min_total_time(p.graph, p.starts, p.goals, T=8, horizon="single", backend="gurobi",
                               params={"SoftMemLimit": 1e-5, "Threads": 1})


@needs_gurobi
@pytest.mark.parametrize("prm", [{"NodeLimit": 0}, {"SolutionLimit": 1}])
def test_gurobi_limits_are_undecided(prm):
    p = _grid5()
    r = mppflow.min_total_time(p.graph, p.starts, p.goals, T=8, horizon="single", backend="gurobi",
                               params={"Threads": 1, **prm})
    assert r.paths is None and r.status == "time_limit"


@needs_gurobi
def test_gurobi_interrupt_raises_keyboard_interrupt():
    """gurobipy turns Ctrl-C into status INTERRUPTED; the backend passes it on (formerly the
    horizon counted as infeasible and the makespan search went on to a too-large 'optimal')."""
    import os
    import signal
    import threading

    p = mppflow.create_n2_puzzle(4, MathRandom(1))
    m = build_time_model(p.graph, p.starts, p.goals, 6)  # infeasible; Gurobi needs > 20 s to prove it
    timer = threading.Timer(1.5, lambda: os.kill(os.getpid(), signal.SIGINT))
    timer.start()
    try:
        with pytest.raises(KeyboardInterrupt):
            mppflow.lp.solve(m.model, backend="gurobi", params={"Threads": 1})
    finally:
        timer.cancel()


def test_puzzle_solver_node_limit_is_undecided():
    """PuzzleSolver moved on to T + 1 after a horizon whose models all stopped at a limit."""
    from mppflow.puzzle import PuzzleSolver

    p = mppflow.create_n2_puzzle(3, MathRandom(1))
    r = PuzzleSolver(p.graph, p.starts, p.goals, backend="highs", params={"node_limit": 0}).solve()
    assert r.paths is None and r.status == "time_limit" and r.extra["solver_status"] == "node_limit"
    assert PuzzleSolver(p.graph, p.starts, p.goals, backend="highs").solve().objective == 4


# ---------------------------------------------------------------- time_limit validation
_BAD_LIMITS = [0, 0.0, -1, -1.0, math.nan, math.inf, -math.inf, True, "5"]


@pytest.mark.parametrize("bad", _BAD_LIMITS)
@pytest.mark.parametrize("objective,split", [("makespan", 1), ("makespan", 2), ("total_time", 1),
                                             ("total_distance", 2), ("max_distance", 1)])
def test_plan_rejects_a_time_limit_that_is_not_positive(bad, objective, split):
    """``plan`` (fixed): ``None`` is the only "no limit". 0, a negative limit and NaN used
    to mean no limit (an unbounded solve), while a tiny positive limit timed out at once."""
    p = _grid5()
    with pytest.raises(ValueError, match="time_limit"):
        mppflow.plan(p, objective, split, time_limit=bad)


@pytest.mark.parametrize("bad", _BAD_LIMITS)
@pytest.mark.parametrize("fn", [mppflow.min_makespan, mppflow.min_total_time, mppflow.min_total_distance,
                                mppflow.min_max_distance])
def test_min_functions_reject_a_time_limit_that_is_not_positive(fn, bad):
    p = _grid5()
    with pytest.raises(ValueError, match="time_limit"):
        fn(p.graph, p.starts, p.goals, time_limit=bad)


def test_plan_time_limit_none_and_positive():
    p = _grid5()
    assert mppflow.plan(p, "makespan", time_limit=None).status == "optimal"
    assert mppflow.plan(p, "makespan", time_limit=60).status == "optimal"
    assert mppflow.plan(p, "makespan", time_limit=1e-9).status == "time_limit"
    # original: the original's rule, a limit <= 0 means no limit
    assert mppflow.plan(p, "makespan", time_limit=0, compat="original").status == "optimal"


@pytest.mark.parametrize("call", [
    lambda p, tl: S.solve_time(p.graph, p.starts, p.goals, False, True, 0, True, tl),
    lambda p, tl: S.solve_total_time(p.graph, p.starts, p.goals, None, 0.0, None, True, tl),
    lambda p, tl: S.solve_distance(p.graph, p.starts, p.goals, 0.0, None, tl),
    lambda p, tl: S.solve_max_distance(p.graph, p.starts, p.goals, 0.0, None, tl),
])
def test_solvers_with_the_original_signature_reject_nan(call, monkeypatch):
    """The solvers with the original's signature keep its sentinel (``time_limit <= 0``: no
    limit, default -1.0) but reject NaN, which compared false and so meant no limit too.
    The sentinel is checked on a small instance (every solve gets no limit): on _grid5 the
    unlimited TT / DIST solves took up to a minute (TS1A-NAN-TEST-UNLIMITED-SOLVES)."""
    with pytest.raises(ValueError, match="time_limit"):
        call(_grid5(), math.nan)
    p = mppflow.create_grid_problem(3, 4, 0.0, 6, MathRandom(35))
    real, limits = mppflow.lp.solve, []

    def spy(model, *a, **k):
        limits.append(k.get("time_limit"))
        return real(model, *a, **k)

    monkeypatch.setattr(mppflow.lp, "solve", spy)
    assert call(p, -1.0).paths is not None
    assert limits and set(limits) == {None}
