"""MultiagentGraphSolverGurobiTime.solve never checks the input. The vertex-capacity rows
cover times 1..T only (prepareModel, ~line 300), so duplicate starts give a plan that
collides at t = 0; duplicate goals make every horizon infeasible and the T loop never ends.
Star graph 0-1, 1-2, 1-3. The pure-Python builtin backend gives the same plans on every
platform (HiGHS versions return different, equally valid plans).
"""
import _common
from _common import edges_graph, show

from mppflow.planner import path_set_error
from mppflow.solvers import solve_time

star = edges_graph(4, [(0, 1), (1, 2), (1, 3)])
for c in ("original", "fixed"):
    r = show(f"{c}: starts [0, 0] -> goals [2, 3]",
             lambda: solve_time(star, [0, 0], [2, 3], compat=c, backend="builtin").paths)
    if r is not None:
        print("   validity:", path_set_error(r, star, [0, 0], [2, 3]))
    show(f"{c}: starts [0, 2] -> goals [3, 3]",
         lambda: solve_time(star, [0, 2], [3, 3], compat=c, backend="builtin").paths)
