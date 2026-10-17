"""Main.solveProblem (and planPathsAdvanced with solveRelaxed) first raises T with the LP
relaxation (MultiagentGraphSolverGurobiTime.java:58-85), a step the paper does not
describe. It is a sound pruning (an LP max-flow below n proves the ILP infeasible) and
does not change the result. Main case 3 (star swap): the models solved, in order.
"""
import _common
from _common import edges_graph

from mppflow.solvers import solve_time

star = edges_graph(4, [(0, 1), (1, 2), (1, 3)])
for lp_first in (False, True):
    r = solve_time(star, [0, 3], [3, 0], lp_first)
    print(f"solveRelaxedFirst={lp_first}: makespan {r.makespan}, models "
          f"{[(a['phase'], a['T'], round(a['objective'], 3) if a['objective'] is not None else None) for a in r.attempts]}")
