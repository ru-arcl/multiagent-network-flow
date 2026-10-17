"""T-RO 2016 Fig. 8 / Sec. V-B: the 2-way split example needs 3 + 2 steps, not 2 + 2.

The grid in Fig. 8 has 4 x 4 vertices (ids row * 4 + col, row 0 at the top). r1 goes
(3,1) -> (0,0), r2 goes (2,0) -> (1,3); the intermediate goals are (1,1) and (2,2). In the
first child both robots' only 2-step paths pass (2,1) at t = 1 (a meet), so it needs 3 steps.
The pure-Python builtin backend gives the same printed plan on every platform.
"""
import _common
from _common import grid

from mppflow._java import MathRandom
from mppflow.planner import plan, split_paths
from mppflow.solvers import min_makespan

g = grid(4, 4)
starts, goals = [13, 8], [0, 7]
mid = split_paths(g, starts, goals, rng=MathRandom(1))
print("intermediate goals chosen by the code:", mid, "(paper: (1,1) = 5, (2,2) = 10)")
c1 = min_makespan(g, starts, [5, 10], backend="builtin")
c2 = min_makespan(g, [5, 10], goals, backend="builtin")
print("child 1 optimal makespan:", c1.makespan, c1.paths)
print("child 2 optimal makespan:", c2.makespan)
print("unsplit optimal makespan:", min_makespan(g, starts, goals, backend="builtin").makespan)
r = plan((g, starts, goals), split=2, rng=MathRandom(1), time_limit=60, backend="builtin")
print("2-way split plan: makespan", r.makespan, "intermediate goals", r.intermediate_goals)
