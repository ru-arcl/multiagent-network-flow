"""The local-repair heuristics test ``if (timeLeft < 0) return`` before each sub-problem
(HeuristicPathPlanner.java:177-180, PathPlanner.java:214-217), so an exactly spent budget,
timeLeft == 0.0, gets through: the clock has millisecond steps and budgets such as 1.75 s are
exact doubles. planPathsAdvanced(..., timeLeft) then calls Time.solve with
hasTimeLimit = timeLeft > 0 = false, i.e. without any limit, and on a sub-problem that has no
solution the attempt never returns. Here a scripted clock (0 at the attempt's start, then
exactly 1.75 s) drives one attempt on a corridor swap, whose sub-problems are infeasible.
"""
import _common  # noqa: F401

from mppflow._java import MathRandom
from mppflow.errors import OriginalWouldHang
from mppflow.graph import Graph
from mppflow.heuristic import HeuristicPathPlanner

g = Graph.from_adjacency([[1], [0, 2], [1, 3], [2, 4], [3, 5], [4, 6], [5]])
for compat, budget in (("original", 1.75), ("original", 1.751), ("fixed", 1.75)):
    ticks = {"n": 0}

    def clock():
        ticks["n"] += 1
        return 0.0 if ticks["n"] == 1 else 1.75

    pl = HeuristicPathPlanner(compat=compat, rng=MathRandom(3), clock=clock)
    pl._set_distances(g, [6, 0])  # as planHeuristicPaths does before its attempts
    try:
        r = pl.plan_heuristic_path(g, [0, 6], [6, 0], budget)
        print(f"{compat}, budget {budget} s, 1.75 s elapsed: stop {r.extra['stop']!r}, "
              f"sub-problems solved {r.extra['solved']} of {r.extra['examined']}")
    except OriginalWouldHang as e:
        print(f"{compat}, budget {budget} s, 1.75 s elapsed: OriginalWouldHang: {e}")
