"""T-RO 2016 Sec. II-A: the collision definition binds k ("there exists k in Z+") but its
body uses t. The intended definition (ICRA/STAR use k throughout) is: p_i, p_j collide if
for some t, p_i(t) = p_j(t) (meet) or p_i(t) = p_j(t+1) and p_i(t+1) = p_j(t) (head-on).
mppflow's validator checks exactly that; following and rotations are allowed (Fig. 2).
"""
import _common
from _common import edges_graph, grid

from mppflow.planner import path_set_error

line = edges_graph(3, [(0, 1), (1, 2)])
cases = {
    "Fig. 2(a) follow": (line, [[1, 2], [0, 1]]),
    "Fig. 2(b) head-on": (line, [[0, 1], [1, 0]]),
    "Fig. 2(c) meet": (line, [[0, 1], [2, 1]]),
    "rotation on a 4-cycle": (grid(2, 2), [[0, 1], [1, 3], [3, 2], [2, 0]]),
}
for name, (g, paths) in cases.items():
    starts, goals = [p[0] for p in paths], [p[-1] for p in paths]
    print(f"{name}: {path_set_error(paths, g, starts, goals) or 'valid'}")
