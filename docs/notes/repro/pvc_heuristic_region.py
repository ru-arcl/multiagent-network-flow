"""The local-repair heuristic is described only in ICRA 2013 Sec. VI-C ("apply MinMakespan to
[each conflict's] neighborhood of distance 2"); T-RO 2016 does not use it. The code's
createSubProblem (PathPlanner.java:303, HeuristicPathPlanner.java:314) adds, for every
distance-2 visit (s, u, w), the neighbours of w with probability 1/2 (PathPlanner) or 1/4
(HeuristicPathPlanner), one Math.random() draw per visit: a random region between the
distance-2 and distance-3 balls. Conflict at the centre vertex 44 of a 10x10 grid.
"""
import _common
from _common import grid

from mppflow._java import MathRandom
from mppflow.heuristic import HeuristicPathPlanner, LocalRepairPlanner
from mppflow.pathfinder import get_distances_to_all_vertices

g = grid(10, 10)
d = get_distances_to_all_vertices(g, 44)
print("distance-2 ball:", sum(x <= 2 for x in d), "vertices; distance-3 ball:", sum(x <= 3 for x in d))
paths, goals = [[44, 45], [45, 44]], [0, 99]
print("PathPlanner regions, seeds 1..10:",
      [LocalRepairPlanner(rng=MathRandom(s)).create_sub_problem(g, paths, [44], goals, 2).graph.n for s in range(1, 11)])
print("HeuristicPathPlanner regions, seeds 1..10:",
      [HeuristicPathPlanner(rng=MathRandom(s)).create_sub_problem(g, paths, 0, [44], goals, 2).graph.n for s in range(1, 11)])
