"""T-RO 2016 Fig. 5 (also ICRA 2013 Fig. 5) draws loopback e2 from the wrong row, and the
drawn T = 2 network has no full flow.

In Fig. 4(a), r2's goal s2- is the pendant vertex on junction A. The loopback of robot 2
must leave s2-(T)'. The figure draws e2 (and, in T-RO, r2's bold goal arcs) on the row of A,
which has degree 3. Separately, the instance needs T = 4: the maximum multiflow is below n
for T <= 3.
"""
import _common
from _common import edges_graph

from mppflow import lp
from mppflow.model import build_full_network_model
from mppflow.network import FullNetwork

names = {0: "s1-", 1: "s2-", 2: "A", 3: "s1+", 4: "B", 5: "s2+"}
g = edges_graph(6, [(0, 2), (1, 2), (2, 4), (3, 4), (4, 5)])
starts, goals = [3, 5], [0, 1]
print("degrees (without the wait loop):", {names[v]: len(g.adj[v]) - 1 for v in range(6)})
net = FullNetwork.build(g, starts, goals, 2)
for i in range(2):
    a, b, kind = net.arcs[i]
    print(f"loopback e{i + 1}: {names[net.nodes[a][1]]}(2)' -> {names[net.nodes[b][1]]}(0)")
flow = {}
for T in range(6):
    fm = build_full_network_model(g, starts, goals, T)
    flow[T] = round(lp.solve(fm.model, backend="highs").objective)
print("maximum multiflow by T:", flow)
