"""STAR 2013 Theorem 8: T = n + ell - 1 is called "necessary and sufficient" for the unlabeled
problem (ell = max distance from a start to a goal). It is sufficient and tight in the worst
case, but not necessary for a given instance.

The unlabeled optimum is the best labeled optimum over all assignments of goals to robots.
"""
import itertools

import _common
from _common import edges_graph

from mppflow.errors import Infeasible
from mppflow.pathfinder import get_distances_to_all_vertices
from mppflow.solvers import min_makespan


def unlabeled(g, S, G):
    ell = max(get_distances_to_all_vertices(g, u)[v] for u in S for v in G)
    best = None
    for perm in itertools.permutations(G):
        try:
            T = min_makespan(g, S, list(perm)).makespan
        except Infeasible:
            continue
        best = T if best is None else min(best, T)
    return ell, len(S) + ell - 1, best


path5 = edges_graph(5, [(0, 1), (1, 2), (2, 3), (3, 4)])
fig4a = edges_graph(6, [(0, 2), (1, 2), (2, 4), (3, 4), (4, 5)])
for name, g, S, G in [("path 0-1-2-3-4, S={0,4}, G={1,3}", path5, [0, 4], [1, 3]),
                      ("STAR Fig. 3 / T-RO Fig. 4(a)", fig4a, [3, 5], [0, 1])]:
    ell, bound, opt = unlabeled(g, S, G)
    print(f"{name}: ell={ell}, n+ell-1={bound}, optimal makespan={opt}")
