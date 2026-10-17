"""Main.run32x32PerformanceMetric (T-RO Fig. 17) runs its 2/4/8/16-way splits with
noCycle = true (Main.java:611), i.e. planPathsAdvancedSplitNoCycle, which re-plans pieces
that contain rotations, instead of the k-way split of Sec. V-B used everywhere else
(multiThreadedSplitPlanning). Small example: 3x3 grid, 6 robots, 2-way split, solved with the
pure-Python builtin backend (the same plans on every platform; HiGHS versions return different,
equally valid plans, which changes the rotation counts).
"""
import _common
from _common import grid

from mppflow._java import MathRandom
from mppflow.planner import PathPlanner, has_cycles
from mppflow.problem import create_grid_problem

for seed in (5, 6, 7):
    p = create_grid_problem(3, 3, 0.0, 6, rng=MathRandom(seed))
    a = PathPlanner(rng=MathRandom(1), backend="builtin")
    pa = a.multi_threaded_split_planning(p.graph, p.starts, p.goals, 100.0, 1)
    b = PathPlanner(rng=MathRandom(1), backend="builtin")
    pb = b.plan_paths_advanced_split_no_cycle(p.graph, p.starts, p.goals, True, 100.0, 1, True)
    print(f"seed {seed}: k-way split makespan {len(pa[0]) - 1}, rotations {has_cycles(pa)}, solves {len(a.solves)} | "
          f"no-cycle split makespan {len(pb[0]) - 1}, rotations {has_cycles(pb)}, solves {len(b.solves)}")
