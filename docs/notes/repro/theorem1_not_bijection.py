"""T-RO 2016 Theorem 1 (also ICRA 2013 Thm. 2, STAR 2013 Thm. 3): the map from integer flows
on the full network to MPP solutions is onto, but not one-to-one.

A robot that waits at u during [t, t+1] can take the green arc u(t)' -> u(t+1) or go
through any incident merge-split gadget, u(t)' -> m -> s -> u(t+1). We enumerate every
unit flow of a single robot (a path from s(0) to g(T)' in the network) and decode it to
the robot's vertex sequence (the blue arcs it uses).
"""
import _common
from _common import edges_graph

from mppflow.network import FullNetwork


def count(g, s, goal, T):
    net = FullNetwork.build(g, [s], [goal], T)
    outs = net.out_arcs()
    src, dst = net.copy_node(s, 0), net.copy_node(goal, T, prime=True)
    flows, paths = 0, set()

    def dfs(node, seq):
        nonlocal flows
        if node == dst:
            flows += 1
            paths.add(tuple(seq))
            return
        for j in outs[node]:
            a, b, kind = net.arcs[j]
            if kind == "loop":
                continue
            key = net.nodes[b]
            dfs(b, seq + [key[1]] if kind == "blue" else seq)

    dfs(src, [s])
    return flows, len(paths)


uw = edges_graph(2, [(0, 1)])
fig4a = edges_graph(6, [(0, 2), (1, 2), (2, 4), (3, 4), (4, 5)])
for name, g, s, goal, T in [("edge u-w, stay at u", uw, 0, 0, 1),
                            ("edge u-w, move u->w", uw, 0, 1, 1),
                            ("Fig. 4(a), r1 s1+ -> s1-", fig4a, 3, 0, 3),
                            ("Fig. 4(a), r1 s1+ -> s1-", fig4a, 3, 0, 4),
                            ("Fig. 4(a), r1 s1+ -> s1-", fig4a, 3, 0, 5)]:
    f, p = count(g, s, goal, T)
    print(f"{name}, T={T}: unit flows={f}, distinct paths={p}")
