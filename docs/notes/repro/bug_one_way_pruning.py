"""updateReachableSets (MultiagentGraphSolverGurobiTime.java:135-160, same in every solver)
uses distFromGoal = BFS *from* the goal, i.e. dist(g, v), where dist(v, g) is needed. On a
one-way (directed) graph this prunes vertices a robot must pass. Every graph the original
builds is symmetric, so this is latent. One-way 4-cycle 0 -> 1 -> 2 -> 3 -> 0 with waits.
"""
import _common
from _common import show

from mppflow._java import MathRandom
from mppflow.graph import Graph
from mppflow.solvers import solve_time

g = Graph.from_adjacency([[1], [2], [3], [0]])  # reverse arcs are not added
for c in ("original", "fixed"):
    show(f"{c}: robot 0 -> 3", lambda: solve_time(g, [0], [3], compat=c, rng=MathRandom(1)).paths)
