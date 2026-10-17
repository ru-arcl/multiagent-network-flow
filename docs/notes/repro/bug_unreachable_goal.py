"""PathFinder.findOneShortestPath returns null for an unreachable goal (PathFinder.java:233)
and every caller reads paths[i].vertexList: NullPointerException (lower bounds, every solver).
Graph with two components {0-1} and {2-3}; robot 0 -> 3.
"""
import _common
from _common import edges_graph, show

from mppflow._java import MathRandom
from mppflow.pathfinder import get_makespan_lower_bound
from mppflow.solvers import solve_time

g = edges_graph(4, [(0, 1), (2, 3)])
for c in ("original", "fixed"):
    show(f"{c}: getMakespanLowerBound", get_makespan_lower_bound, g, [0], [3], MathRandom(1), compat=c)
    show(f"{c}: Time.solve", lambda: solve_time(g, [0], [3], compat=c, rng=MathRandom(1)).paths)
