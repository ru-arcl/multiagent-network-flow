"""T-RO 2016, text after eq. (12): "Note that x_{i,j} = 0 for j < n" is wrong.

Loopback arcs are e_1..e_n (e_j belongs to robot r_j). The constraint after eq. (8) sets
x_{i,j} = 0 for i != j, and eq. (11) sets x_{i,i} = 1. With n = 2: x_{1,1} = 1 although
1 < n. The loopbacks cost 0, so the restriction j > n in (12)/(16) is redundant anyway.
"""
import _common
from _common import edges_graph

from mppflow.model import build_full_network_model

g = edges_graph(6, [(0, 2), (1, 2), (2, 4), (3, 4), (4, 5)])  # Fig. 4(a)
fm = build_full_network_model(g, [3, 5], [0, 1], 4, objective="distance")
n = 2
for i in range(n):
    for j in range(n):  # arc j is the loopback e_{j+1}
        lb, ub = fm.model.lb[fm.x[i][j]], fm.model.ub[fm.x[i][j]]
        print(f"x_{{{i + 1},{j + 1}}} in [{lb:g}, {ub:g}]")
print("loopback costs:", [fm.network.arc_cost[j] for j in range(n)],
      "(paper costs:", [fm.network.paper_cost[j] for j in range(n)], ")")
