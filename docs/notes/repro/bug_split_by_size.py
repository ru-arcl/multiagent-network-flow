"""Dead split-by-size code (only commented-out calls in Main.java:139-140).

1. splitPathBySize (PathPlanner.java:788-858): the goal-side window is off by one and dMax
   never grows, so the loop can spin forever (1 robot 0 -> 8 on a 3x3 grid, firstSize 2).
2. getLongestPathLength (PathPlanner.java:777-786) compares edges (size - 1) but stores
   vertices (size): the result depends on the robot order. Path graph 0..7.
"""
import _common
from _common import edges_graph, grid, show

from mppflow._java import MathRandom
from mppflow.planner import get_longest_path_length, split_path_by_size

line = edges_graph(8, [(i, i + 1) for i in range(7)])
for c in ("original", "fixed"):
    show(f"{c}: splitPathBySize(3x3, [0] -> [8], firstSize 2)", split_path_by_size, grid(3, 3), [0], [8], False, 2,
         rng=MathRandom(1), compat=c)
    show(f"{c}: getLongestPathLength([0,1] -> [4,6])", get_longest_path_length, line, [0, 1], [4, 6],
         rng=MathRandom(1), compat=c)
    show(f"{c}: getLongestPathLength([1,0] -> [6,4])", get_longest_path_length, line, [1, 0], [6, 4],
         rng=MathRandom(1), compat=c)
