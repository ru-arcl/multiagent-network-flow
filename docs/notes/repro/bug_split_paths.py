"""PathPlanner.splitPaths (PathPlanner.java:577-648) picks robot i's intermediate goal from
findGoalsInDistanceRange(start) and (goal) intersected (PathFinder.java:61-102). That search
starts at distance 1 and never returns its centre, so a robot's own start and goal can never
be picked:

1. If every other vertex is already some robot's intermediate goal, the window widens
   forever (2x2 grid, 3 robots; the instance itself is solvable).
2. A robot at its goal, or next to it, is sent on a detour (the paper takes the middle of
   the robot's path, which is its start / goal there).
"""
import _common
from _common import grid, show

from mppflow._java import MathRandom
from mppflow.planner import PathPlanner, split_paths

for c in ("original", "fixed"):
    show(f"{c}: splitPaths(2x2, [0,2,3] -> [3,1,2])", split_paths, grid(2, 2), [0, 2, 3], [3, 1, 2],
         rng=MathRandom(1), compat=c)
g3 = grid(3, 3)
for c in ("original", "fixed"):
    for s, t in (([0, 4], [1, 4]), ([4], [4]), ([0, 2], [1, 2])):
        mid = split_paths(g3, s, t, rng=MathRandom(1), compat=c)
        p = PathPlanner(compat=c, rng=MathRandom(1)).plan_paths_advanced_split(g3, s, t, True, -1.0, 1)
        print(f"{c}: 3x3 {s} -> {t}: intermediate goals {mid}, 2-way split makespan {len(p[0]) - 1}")
