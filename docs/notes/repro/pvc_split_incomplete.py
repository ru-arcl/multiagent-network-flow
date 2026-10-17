"""The k-way split of T-RO Sec. V-B is a heuristic and can fail on a solvable instance: the
intermediate configuration it takes from the robots' shortest paths may be unreachable from
the start (or the goal unreachable from it), e.g. on a cycle, where robots cannot change their
cyclic order. Both modes; compat="original" reports "time_limit" without a time limit (B13).
"""
import _common  # noqa: F401

from mppflow import from_adjacency, plan
from mppflow.graph import create_2d_grid_graph

cases = [
    ("2x2 grid (a 4-cycle)", create_2d_grid_graph(2, 2), [0, 1, 3], [3, 2, 1]),
    ("6-cycle", from_adjacency([[1, 5], [0, 2], [1, 3], [2, 4], [3, 5], [0, 4]]), [0, 5, 1], [4, 2, 5]),
]
for name, g, s, t in cases:
    whole = plan((g, s, t), "makespan")
    print(f"{name}: k = 1 {whole.status}, makespan {whole.makespan}")
    for objective, k in (("makespan", 2), ("total_distance", 2), ("makespan", 4)):
        r = plan((g, s, t), objective, k)
        print(f"  {objective} k = {k}: {r.status}, intermediate goals {r.intermediate_goals}")
g, s, t = cases[0][1:]
r = plan((g, s, t), "makespan", 2, compat="original", backend="builtin")
print(f"original, 2x2 grid, makespan k = 2: {r.status}, intermediate goals {r.intermediate_goals}")
