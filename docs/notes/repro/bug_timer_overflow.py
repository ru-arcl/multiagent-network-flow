"""MultiagentGraphSolverGurobiTimeR (one move per time step) computes
getEdgeId = a*10^12 + v1*100000000 + v2*10000 + t with v1*100000000 in 32-bit int
(MultiagentGraphSolverGurobiTimeR.java:173-175). It wraps for v1 >= 22, so getTimeFromEdgeId
(id % 10000) no longer returns t and every arc leaving a vertex with id >= 22 is missing from
its "at most one move at step t" row (lines 293-302). On a 6x6 grid the swap 30 <-> 35 (ids
>= 22) and its mirror image 0 <-> 5 are the same problem; only the low-id one obeys the rule.
With one move per step a swap needs at least 12 moves (one robot detours by 2), so a makespan
below 12 implies simultaneous moves in every plan, whichever optimal plan the solver returns.
"""
import _common
from _common import grid

from mppflow._java import MathRandom
from mppflow.model import edge_id, time_from_edge_id
from mppflow.solvers import solve_time_r

for a, v1, v2, t in ((0, 21, 20, 3), (0, 22, 21, 3), (1, 22, 21, 3)):
    e = edge_id(a, v1, v2, t)
    print(f"getEdgeId({a}, {v1}, {v2}, {t}) = {e}, getTimeFromEdgeId = {time_from_edge_id(e)}")
g = grid(6, 6)
for c in ("original", "fixed"):
    for s, goal in (([30, 35], [35, 30]), ([0, 5], [5, 0])):
        r = solve_time_r(g, s, goal, compat=c, rng=MathRandom(1))
        multi = any(sum(p[t] != p[t + 1] for p in r.paths) > 1 for t in range(r.makespan))
        print(f"{c}: swap {s}: makespan {r.makespan}, more than one mover at some step: {'yes' if multi else 'no'}")
