"""Main.solveProblemWithHeuristic (Main.java:323-358).

1. It sets PathFinder.bHeuristic = true and never resets it. With the flag, findOneShortestPath
   inserts at a random queue position (PathFinder.java:253): the search is no longer BFS, the
   "lower bounds" are too large and later exact solves start above T_opt (16 instead of 14).
2. It returns {paths[0].length (vertices, not makespan), time, goals, moves - 1, lb - 1}.
Heuristic run on a seeded 8x8 instance with 6 robots, then an exact solve of 3 robots on the
empty 8x8 grid in the same run; stat vectors without their time column.
"""
import _common
from _common import grid

from mppflow._java import MathRandom
from mppflow.experiments import Experiments
from mppflow.pathfinder import get_makespan_lower_bound, get_total_distance, get_total_time_lower_bound
from mppflow.problem import Problem, create_grid_problem

g8 = grid(8, 8)
print("getMakespanLowerBound(0 -> 63) - 1, seeds 15..24:",
      [get_makespan_lower_bound(g8, [0], [63], MathRandom(s), heuristic=True) - 1 for s in range(15, 25)])
p = create_grid_problem(8, 8, 0.0, 6, rng=MathRandom(3))
q = Problem(g8, [0, 7, 56], [63, 56, 7])
for seed in (1000, 1005):
    for c in ("original", "fixed"):
        ex = Experiments(compat=c, rng=seed, clock=lambda: 0.0)
        st = ex.solve_problem_with_heuristic(p)
        hp = ex.last_paths
        true = [len(hp[0]) - 1, get_total_distance(hp), get_total_time_lower_bound(p.graph, p.starts, p.goals)]
        ex2 = ex.solve_problem(q, True, -1)
        print(f"seed {seed} {c}: heuristic stat {st[:1] + st[2:]} (true makespan, moves, lb: {true}); "
              f"bHeuristic left set: {ex.b_heuristic}; exact solve afterwards: makespan {ex2[0]}, lb {ex2[3]} (optimum 14)")
