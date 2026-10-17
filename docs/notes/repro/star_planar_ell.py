"""STAR 2013, remark after Thm. 10: "If G is a planar graph, we have E ~ O(V) and
ell ~ O(V^{1/2})". Planarity bounds E but not the diameter: the path graph P_V is planar
and has ell = V - 1. Starts {0..n-1}, goals {V-n..V-1}; the optimal makespan is V - n,
so the horizon really is Theta(V) for fixed n.
"""
import math

import _common
from _common import edges_graph

from mppflow.pathfinder import get_distances_to_all_vertices
from mppflow.solvers import min_makespan

for V, n in ((9, 1), (16, 2), (36, 4), (64, 4)):
    g = edges_graph(V, [(i, i + 1) for i in range(V - 1)])
    S, G = list(range(n)), list(range(V - n, V))
    ell = max(get_distances_to_all_vertices(g, u)[v] for u in S for v in G)
    print(f"V={V} n={n}: ell={ell} (sqrt(V)={math.isqrt(V)}), n+ell-1={n + ell - 1}, "
          f"optimal makespan={min_makespan(g, S, G).makespan}")
