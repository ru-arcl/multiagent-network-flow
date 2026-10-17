"""TotalTime.solve (MultiagentGraphSolverGurobiTotalTime.java:76-80) computes
agap = bnd/val - 1 and Distance.solve (MultiagentGraphSolverGurobiDistance.java:58)
1 - bnd/objVal. When the optimum is 0 both are 0/0 = NaN, `NaN <= gap` is false, and the
optimal solution is thrown away (" time out.", null).
TT: one robot 0 -> 2 on a path at T = 2 (it needs every step: sum of goal-waiting steps 0).
DIST: every robot already at its goal (nobody has to move).
"""
import _common
from _common import edges_graph

from mppflow.solvers import solve_distance, solve_total_time

line = edges_graph(3, [(0, 1), (1, 2)])
star = edges_graph(4, [(0, 1), (1, 2), (1, 3)])
for c in ("original", "fixed"):
    r = solve_total_time(line, [0], [2], None, 0.0, 2, horizon="single", compat=c, debug=True)
    print(f"{c} TT:   status {r.status}, total time {r.objective}, paths {r.paths}, "
          f"' time out.' printed: {' time out.' in r.log}")
    r = solve_distance(star, [0, 3], [0, 3], 0.0, 3, horizon="single", compat=c, debug=True)
    print(f"{c} DIST: status {r.status}, distance {r.objective}, paths {r.paths}, "
          f"' time out.' printed: {' time out.' in r.log}")
