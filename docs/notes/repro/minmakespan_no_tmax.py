"""ICRA 2013 Sec. IV-A / Prop. 3: MinMakespan ("if the model is not feasible, we increase T
and try again") has no upper bound on T, so it never terminates on an infeasible instance.

T-RO 2016 Sec. IV-E adds the missing pieces (a feasibility test and T_max). The original
Java follows the ICRA loop: with no time limit it runs forever. Instance: two robots swap
the ends of a single edge (infeasible: head-on swaps are forbidden).

The fixed mode's proof is a joint-state search, which is practical only on small joint state
spaces, plus polynomial certificates. Two larger infeasible instances that the certificates
settle at the first failed horizon: robots 6 and 7 of an 8-robot instance must swap on a
separate edge next to a 5x5 grid (about 10^11 joint configurations), and 5 robots must reverse
their order in a 16-vertex corridor (before the certificates, the loop reached its first
conclusive search only at T = 30, after ILP solves of over 90 s per horizon from T = 23 on).
"""
import _common
from _common import create_2d_grid_graph, edges_graph

from mppflow.errors import Infeasible, OriginalWouldHang
from mppflow.graph import Graph
from mppflow.solvers import min_makespan, solve_time

g = edges_graph(2, [(0, 1)])
try:
    solve_time(g, [0, 1], [1, 0], compat="original", backend="highs")
except OriginalWouldHang as e:
    print("original:", type(e).__name__, "-", e)
try:
    solve_time(g, [0, 1], [1, 0])
except Infeasible as e:
    print("fixed:   ", type(e).__name__, "-", e)

grid = create_2d_grid_graph(5, 5)
big = Graph.from_adjacency([list(grid.adj[v]) for v in range(25)] + [[26], [25]])
try:
    min_makespan(big, [0, 4, 20, 24, 12, 7, 25, 26], [24, 20, 4, 0, 2, 22, 26, 25])
except Infeasible as e:
    print("fixed, 5x5 grid + separate edge:", type(e).__name__, "-", e)
corridor = Graph.from_adjacency([[v for v in (u - 1, u + 1) if 0 <= v < 16] for u in range(16)])
try:
    min_makespan(corridor, [0, 1, 2, 3, 4], [15, 14, 13, 12, 11])
except Infeasible as e:
    print("fixed, corridor:", type(e).__name__, "-", e)
