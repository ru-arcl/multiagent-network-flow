"""PathPlanner.planPathsAdvancedSplitDist (PathPlanner.java:680-713).

1. The leaf dereferences the makespan result without a null check (line 709): a time-out
   (or an infeasible piece) throws NullPointerException. Fake clock, 0.5 s per reading.
2. With timeLimit <= 0 (the code base's "no limit") and splits > 0, `if (timeLimit > 0)`
   skips the second half after `timeLimit -= elapsed`, so the result is always null.
"""
import itertools

import _common
from _common import edges_graph, grid, show

from mppflow._java import MathRandom
from mppflow.planner import PathPlanner


def clock():
    c = itertools.count()
    return lambda: next(c) * 0.5


def mk(p):
    return "null" if p is None else f"makespan {len(p[0]) - 1}"


star = edges_graph(4, [(0, 1), (1, 2), (1, 3)])
g3 = grid(3, 3)
for c in ("original", "fixed"):
    pl = PathPlanner(compat=c, rng=MathRandom(1), clock=clock())
    show(f"{c}: star swap, timeLimit 1, splits 0", lambda: mk(pl.plan_paths_advanced_split_dist(star, [0, 3], [3, 0], 0.0, 1.0, 0)))
    pl = PathPlanner(compat=c, rng=MathRandom(1))
    show(f"{c}: 3x3 grid, timeLimit -1, splits 1", lambda: mk(pl.plan_paths_advanced_split_dist(g3, [0, 2, 6], [8, 6, 1], 0.0, -1.0, 1)))
