"""PathPlanner.multiThreadedSplitPlanning / createThread (PathPlanner.java:1040-1131).

1. A worker sets mdone[i] only when solve returns; an exception (GRBException is caught and
   printed, anything else kills the thread) leaves it false and the main thread polls
   forever (lines 1075-1090). Trigger: an unreachable goal (NullPointerException in solve).
2. mpath is an instance field that is never cleared (line 1038): on a reused planner a piece
   that fails (time-out or invalid) silently keeps the previous call's piece. Here call B
   uses timeLimit -1, which makes every piece that is infeasible at its first horizon fail
   (createThread always passes hasTimeLimit = true, line 1114).
The pure-Python builtin backend gives the same plans on every platform.
"""
import _common
from _common import edges_graph, grid, show

from mppflow._java import MathRandom
from mppflow.planner import PathPlanner, path_set_error

g2 = edges_graph(4, [(0, 1), (2, 3)])
for c in ("original", "fixed"):
    show(f"{c}: worker throws",
         PathPlanner(compat=c, rng=MathRandom(1), backend="builtin").multi_threaded_split_planning,
         g2, [0], [3], 30.0, 0)
g3 = grid(3, 3)
A, B = ([0, 2, 6], [8, 6, 1]), ([8, 6, 1], [0, 2, 6])
for c in ("original", "fixed"):
    pl = PathPlanner(compat=c, rng=MathRandom(1), backend="builtin")
    pl.multi_threaded_split_planning(g3, *A, 100.0, 1)
    pb = pl.multi_threaded_split_planning(g3, *B, -1.0, 1)
    fresh = PathPlanner(compat=c, rng=MathRandom(1), backend="builtin").multi_threaded_split_planning(g3, *B, -1.0, 1)
    print(f"{c}: B on the reused planner: {pb}\n   valid for B: {path_set_error(pb, g3, *B) or True}; "
          f"B on a fresh planner: {fresh}")
