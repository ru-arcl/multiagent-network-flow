"""compat="original" extracts paths with the original's exact test X == 1.0
(MultiagentGraphSolverGurobiTime.java:209-216; when no arc matches it silently continues
from vertex 0). HiGHS may return binaries such as 0.9999999999999998, which breaks the plan;
mppflow then issues InexactSolutionWarning. compat="fixed" tests X > 0.5. Instance: the
Main case 6 spider tree with 8 robots (getRandomStartGoalMany, seed 8).
Whether and where HiGHS returns inexact values depends on its version, so the script prints the
scipy version (scipy bundles HiGHS) first.
"""
import warnings

import _common
import scipy
from _common import have_gurobi

from mppflow._java import MathRandom
from mppflow.graph import create_main_case6_tree, get_random_start_goal_many
from mppflow.model import InexactSolutionWarning
from mppflow.planner import path_set_error
from mppflow.solvers import solve_time

print("scipy", scipy.__version__)
tree = create_main_case6_tree()
runs = [("original", "highs", None), ("fixed", "highs", None)]
if have_gurobi():
    runs.append(("original", "gurobi", {"Threads": 1}))
for compat, backend, params in runs:
    rng = MathRandom(8)
    starts, goals = get_random_start_goal_many(tree, 8, rng)
    with warnings.catch_warnings(record=True) as w:
        warnings.simplefilter("always")
        r = solve_time(tree, starts, goals, True, compat=compat, backend=backend, params=params, rng=rng)
    inexact = sum(issubclass(x.category, InexactSolutionWarning) for x in w)
    print(f"{compat}/{backend}: makespan {r.makespan}, InexactSolutionWarning x{inexact}, "
          f"plan: {path_set_error(r.paths, tree, starts, goals) or 'valid'}")
