"""T-RO 2016 Algorithm 1 loops over every T in [T_min, T_max] and keeps the best solution.

It is correct but does more work than needed: for MINMAKESPAN the first feasible T is
optimal, and for the other objectives the optimum is non-increasing in T (a plan for T
extends to T + 1 by waiting at the goals), so one solve at T_max suffices. Instance: 2 x 3
grid, starts [0, 1, 2] -> goals [5, 4, 3], n = 3, T_opt = 3, T_max = n * T_opt = 9.
"""
import _common
from _common import grid

from mppflow.solvers import min_makespan, solve_distance, solve_max_distance, solve_total_time

g = grid(2, 3)
s, t = [0, 1, 2], [5, 4, 3]
Topt = min_makespan(g, s, t).makespan
print("T_opt =", Topt, " T_max =", len(s) * Topt)
for T in range(Topt, len(s) * Topt + 1):
    td = solve_distance(g, s, t, 0.0, T, horizon="single").objective
    tt = solve_total_time(g, s, t, None, 0.0, T, horizon="single").objective
    md = solve_max_distance(g, s, t, 0.0, T, horizon="single").objective
    print(f"T={T}: total distance {td:g}, total time {tt}, max distance {md:g}")
