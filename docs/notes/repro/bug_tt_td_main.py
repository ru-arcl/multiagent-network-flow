"""Main.solveProblemTotalTime / solveProblemDistance (Main.java:221-262) subtract the split
time from the limit and `if (timeLimit <= 0) return null`, so the code base's "no limit" (-1)
always fails; they also read paths[0] of the 4-way split without a null check (lines 229, 257).
3x3 grid, robots [0, 2, 6] -> [8, 6, 1]; prints [objective, lower bound].
"""
import _common
from _common import grid, show

from mppflow.experiments import Experiments
from mppflow.problem import Problem

p = Problem(grid(3, 3), [0, 2, 6], [8, 6, 1])
for c in ("original", "fixed"):
    ex = Experiments(compat=c, rng=1)
    show(f"{c}: solveProblemTotalTime(p, 0, -1)", lambda: (lambda s: s and [s[0], s[3]])(ex.solve_problem_total_time(p, 0.0, -1.0)))
    show(f"{c}: solveProblemDistance(p, 0, -1)", lambda: (lambda s: s and [s[0], s[3]])(ex.solve_problem_distance(p, 0.0, -1.0)))
