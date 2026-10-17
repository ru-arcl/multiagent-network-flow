"""HeuristicPathPlanner budgets attempts by wall clock (1.5 + (5 + 4i) n / 120 s,
HeuristicPathPlanner.java:177), and the sub-ILP gets the time left. The same seed gives
different plans on an idle and a loaded machine. mppflow's default clock is virtual (never
advances); here a clock that advances 0.05 s per reading stands in for a slow machine.
(The plans themselves are not printed: HiGHS versions return different, equally good sub-plans.)
"""
import itertools

import _common

from mppflow._java import MathRandom
from mppflow.heuristic import HeuristicPathPlanner
from mppflow.problem import create_grid_problem

p = create_grid_problem(10, 10, 0.1, 40, rng=MathRandom(7))
tick = itertools.count()
idle = None
for name, clock in (("idle (virtual clock)", None), ("loaded (0.05 s per reading)", lambda: next(tick) * 0.05)):
    r = HeuristicPathPlanner(compat="original", rng=MathRandom(7), clock=clock).plan_heuristic_paths(
        p.graph, p.starts, p.goals)
    same = "" if idle is None else f", same plan as idle: {'yes' if r.paths == idle else 'no'}"
    idle = r.paths if idle is None else idle
    print(f"{name}: {r.status}, makespan {r.objective}, attempts {len(r.attempts)}, "
          f"goals reached {r.extra.get('goals_reached')}/40{same}")
