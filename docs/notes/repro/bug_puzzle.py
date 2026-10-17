"""PuzzleSolver (PuzzleSolver.java:46-108).

1. Deadlock: when both workers fail at a horizon and finish before the main thread's 1 s
   poll, it waits for allThreadFinished, which only a Gurobi message callback sets
   (lines 63-76): it waits forever and ignores the time limit. Star swap (T = 2 infeasible).
2. A second solve() on the same object: optimizationDone is still true, so the first horizon
   is skipped and the plan is one step longer than optimal.
race="fast" assumes every worker finishes before the first poll (true for small instances).
"""
import _common
from _common import edges_graph, grid, show

from mppflow._java import MathRandom
from mppflow.puzzle import PuzzleSolver

star = edges_graph(4, [(0, 1), (1, 2), (1, 3)])
for c in ("original", "fixed"):
    ps = PuzzleSolver(star, [0, 3], [3, 0], compat=c, rng=MathRandom(1), race="fast")
    show(f"{c}: star swap, solve(10)", lambda: ps.solve(10.0).makespan)
for c in ("original", "fixed"):
    ps = PuzzleSolver(grid(3, 3), [0, 6], [2, 8], compat=c, rng=MathRandom(1), race="fast")
    print(f"{c}: 3x3 [0, 6] -> [2, 8]: first solve() makespan {ps.solve(-1.0).makespan}, "
          f"second solve() makespan {ps.solve(-1.0).makespan}")
