"""T-RO 2016 eq. (5): "for all i in C, for all e in E, sum_i f_i(e) <= c1(e)" binds i twice.

The intended constraint is the joint capacity sum_{i in C} f_i(e) <= c1(e) for every arc e
(it generalizes eq. (1)); it is eq. (8) of the ILP. Read as a per-commodity bound
f_i(e) <= c1(e), it would let two robots share a vertex or swap along an edge. Instance:
two robots swap the ends of the path 0-1-2 (infeasible).
"""
import _common
from _common import edges_graph

from mppflow import lp
from mppflow.lp import LinearModel
from mppflow.model import build_full_network_model

g = edges_graph(3, [(0, 1), (1, 2)])
T = 4
fm = build_full_network_model(g, [0, 2], [2, 0], T)
m, n_arcs = fm.model, len(fm.network.arcs)
print("joint capacity rows (eq. 8): sum over both robots per arc, e.g.", m.row(0))
print("max multiflow with joint capacity:", round(lp.solve(m, backend="highs").objective))

per = LinearModel("max")  # same columns; per-commodity bounds only (x <= 1), no eq. (8) rows
for j in range(m.num_columns):
    per.add_column(m.lb[j], m.ub[j], m.vtypes[j])
for i, (terms, sense, rhs) in enumerate(m.rows()):
    if i >= n_arcs:
        per.add_row(terms, sense, rhs)
per.set_objective(list(m.objective_terms), "max")
print("max multiflow with per-commodity bounds only:", round(lp.solve(per, backend="highs").objective))
