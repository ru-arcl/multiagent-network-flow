"""Main.solveProblemSuboptimal prints the paths before its null check (Main.java:148), so a
planner failure (time-out) throws NullPointerException and the drivers' `stat == null`
branches never run. Instance: Main case 3 (star 0-1, 1-2, 1-3; robots swap 0 and 3), time
limit 1e-4 s, a fake clock that advances 0.5 s per reading.
"""
import itertools

import _common
from _common import edges_graph, show

from mppflow.experiments import Experiments
from mppflow.problem import Problem

p = Problem(edges_graph(4, [(0, 1), (1, 2), (1, 3)]), [0, 3], [3, 0])
for c in ("original", "fixed"):
    tick = itertools.count()
    ex = Experiments(compat=c, rng=1, clock=lambda: next(tick) * 0.5)
    show(f"{c}: solveProblemSuboptimal(p, false, true, 0, 1e-4, 0, false)",
         ex.solve_problem_suboptimal, p, False, True, 0, 1e-4, 0, False)
    if ex.failures:
        print("   failures:", ex.failures)
