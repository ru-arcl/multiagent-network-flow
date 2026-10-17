"""T-RO 2016 eq. (15) is off by one, and minimizing it can return a non-optimal plan.

We build the paper's full time-expanded network (Sec. III, with x_{i,i} = 1, eq. (11)) and
add eq. (14) literally: y_i^T <= x_i^T, y_i^t <= x_i^t and y_i^t <= y_i^{t+1}, where x_i^t
(t = 1..T) is robot i's blue arc v(t) -> v(t)' at its goal. Maximizing sum y is minimizing
eq. (15), nT - sum y. We compare with the exact total arrival time (mppflow's compact model,
whose y variables sit on the goal wait arcs t = 0..T-1).
"""
import _common
from _common import arrival_times, edges_graph

from mppflow import lp
from mppflow.model import build_full_network_model
from mppflow.solvers import solve_total_time


def paper_eq15(g, starts, goals, T):
    fm = build_full_network_model(g, starts, goals, T, objective="makespan", set_optimal=True)
    m, net = fm.model, fm.network
    blue = {}
    for j, (a, b, kind) in enumerate(net.arcs):
        if kind == "blue":
            _, v, t, _ = net.nodes[a]
            blue[(v, t)] = j
    ys = []
    for i, gi in enumerate(goals):
        y = {t: m.add_column(0.0, 1.0, "B") for t in range(1, T + 1)}
        for t in range(1, T + 1):
            m.add_row([(y[t], 1.0), (fm.x[i][blue[(gi, t)]], -1.0)], "<=", 0.0)  # y <= x_i^t
            if t < T:
                m.add_row([(y[t], 1.0), (y[t + 1], -1.0)], "<=", 0.0)  # y <= y_i^{t+1}
        ys += list(y.values())
    m.set_objective([(c, 1.0) for c in ys], "max")
    sol = lp.solve(m, backend="highs", mip_gap=0.0)
    paths = fm.retrieve_paths(sol.x)
    return len(starts) * T - round(sol.objective), paths


# Trap: shortcut 0-1-2-3 with pockets 4 (at 1) and 5 (at 2); detour 0-6-7-8-9-10-11-3.
g = edges_graph(12, [(0, 1), (1, 2), (2, 3), (1, 4), (2, 5), (0, 6), (6, 7), (7, 8), (8, 9),
                     (9, 10), (10, 11), (11, 3)])
starts, goals, T = [0, 1, 2], [3, 1, 2], 8

val, paths = paper_eq15(g, starts, goals, T)
print("minimize eq. (15):  eq15 =", val, " arrivals =", arrival_times(paths, goals),
      " true total time =", sum(arrival_times(paths, goals)))
r = solve_total_time(g, starts, goals, None, 0.0, T, horizon="single")
print("exact total time:   value =", r.objective, " arrivals =", arrival_times(r.paths, goals))

# The per-robot term: one robot 0 -> 3 on a path, T = 5, arrives at t = 3.
line = edges_graph(4, [(0, 1), (1, 2), (2, 3)])
v1, p1 = paper_eq15(line, [0], [3], 5)
print("single robot, arrival", arrival_times(p1, [3])[0], "-> T - sum y =", v1)
