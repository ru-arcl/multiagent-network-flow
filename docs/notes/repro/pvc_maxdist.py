"""MINMAXDIST (T-RO 2016 Sec. IV-B, eqs. (11)-(13)) is not in the released Java code; the
paper's Sec. VI-D used MINMAKESPAN in its place. mppflow adds it (compat="fixed" only).
Fig. 4(a) instance.
"""
import _common
from _common import edges_graph, show

from mppflow.solvers import min_makespan, solve_max_distance

g = edges_graph(6, [(0, 2), (1, 2), (2, 4), (3, 4), (4, 5)])
show("fixed: max distance", lambda: solve_max_distance(g, [3, 5], [0, 1]).objective)
show("original", solve_max_distance, g, [3, 5], [0, 1], compat="original")
print("makespan (the paper's stand-in):", min_makespan(g, [3, 5], [0, 1]).makespan)
