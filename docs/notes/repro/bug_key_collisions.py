"""The model maps of MultiagentGraphSolverGurobiTime (and siblings) are keyed by packed int/long
ids (MultiagentGraphSolverGurobiTime.java:162-181): getVertexTimeId = v*10000 + t,
getEdgeId = a*10^12 + v1*10^8 + v2*10^4 + t, getEdgeTimeId = min*10^9 + max*10^4 + t, the
int products wrapping at 32 bits. Distinct (vertex, time) or (edge, time) pairs share a key
once T >= 10000, or on large graphs with long horizons, and their rows / variables are merged.
getEdgeId's v1*10^8 wraps from v1 = 22 on, so two arcs of a 1590-vertex graph can share a key
only 48 steps apart (1589->1379 at t and the wait 0->0 at t+48); with vertex ids < 1024 the
smallest time difference that makes two getEdgeId keys equal is 1328.
mppflow reproduces the keys (and the merges in compat="original"); MPPModel.key_collisions
reports them, and compat="fixed" keeps the groups apart.
"""
import _common

import mppflow
from mppflow.model import build_time_model, edge_id, edge_time_id, i32, vertex_time_id

print("getVertexTimeId(0, 10000) =", vertex_time_id(0, 10000), "= getVertexTimeId(1, 0) =", vertex_time_id(1, 0))
print("getEdgeId(0, 5, 0, 10000) =", edge_id(0, 5, 0, 10000), "= getEdgeId(0, 5, 1, 0) =", edge_id(0, 5, 1, 0))
print("getEdgeTimeId(4, 29501, 0) =", edge_time_id(4, 29501, 0),
      "= getEdgeTimeId(0, 4, 2704) =", edge_time_id(0, 4, 2704))
print("getEdgeId(0, 1589, 1379, 10) =", edge_id(0, 1589, 1379, 10), "= getEdgeId(0, 0, 0, 58) =",
      edge_id(0, 0, 0, 58))

# Smallest time difference at which two getEdgeId keys (same agent, vertex ids < n) coincide.
def smallest_gap(n):
    wrapped = sorted({i32(v * 100000000) for v in range(n)})
    best = None
    for w1 in wrapped:
        for w2 in wrapped:
            d = w1 - w2
            if d == 0:
                continue
            k = max(-(n - 1), min(n - 1, round(-d / 10000)))  # k = v2 - v2'
            gap = abs(d + k * 10000)
            best = gap if best is None else min(best, gap)
    return best


print("smallest time difference of colliding getEdgeId keys, ids < 1024:", smallest_gap(1024),
      "; ids < 1590:", smallest_gap(1590))

# A 1590-vertex graph (one robot 1589 -> 0 through the arcs 1589-0 and 1589-1379):
# the edgeVarMap keys collide from T = 49 on, in both modes (fixed keeps them apart).
adj = [[] for _ in range(1590)]
adj[0], adj[1589], adj[1379] = [1589], [0, 1379], [1589]
g = mppflow.from_adjacency(adj)
for T in (48, 49, 60):
    print(f"T = {T}: edgeVar collisions",
          {c: build_time_model(g, [1589], [0], T, compat=c).key_collisions["edgeVar"]
           for c in ("original", "fixed")})
