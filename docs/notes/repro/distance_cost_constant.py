"""T-RO 2016 eqs. (12)/(13) and (16) (and ICRA 2013 eq. (7)) are constant with the stated costs.

The paper gives the green (wait) arcs v(t)' -> v(t+1) unit cost, as well as the middle arc of
every merge-split gadget. Every unit flow then crosses each of the T layers through exactly
one cost-1 arc, so its cost is T. We minimize and maximize eq. (16) on the Fig. 4(a)
instance with the literal costs and with free waits (only gadget middle arcs cost 1).
"""
import _common
from _common import edges_graph

from mppflow import lp
from mppflow.model import build_full_network_model
from mppflow.solvers import min_max_distance, min_total_distance

# Fig. 4(a): 0 = s1-, 1 = s2- (pendant on A = 2), 3 = s1+ (pendant on B = 4), 5 = s2+.
g = edges_graph(6, [(0, 2), (1, 2), (2, 4), (3, 4), (4, 5)])
starts, goals = [3, 5], [0, 1]


def opt(T, objective, paper_costs, sense):
    fm = build_full_network_model(g, starts, goals, T, objective=objective, paper_costs=paper_costs)
    if sense == "max":  # same objective terms, maximized
        fm.model.set_objective(list(fm.model.objective_terms), "max")
    return round(lp.solve(fm.model, backend="highs", mip_gap=0.0).objective)


for T in (4, 5, 6, 8):
    print(f"T={T}: eq16 paper costs min={opt(T, 'distance', True, 'min')} "
          f"max={opt(T, 'distance', True, 'max')} (nT={2 * T}) | free waits "
          f"min={opt(T, 'distance', False, 'min')} | eq13 paper costs min x_max="
          f"{opt(T, 'max_distance', True, 'min')} | free waits min x_max="
          f"{opt(T, 'max_distance', False, 'min')}")
print("mppflow compact models: total distance", min_total_distance(g, starts, goals).objective,
      "max distance", min_max_distance(g, starts, goals).objective)
