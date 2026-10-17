"""Main.solveProblemTotalTime / solveProblemDistance (Main.java:221-262) solve MINTOTALTIME /
MINTOTALDIST once, at the makespan of a 4-way split (planPathsAdvancedSplit(..., 2)); the
paper's Algorithm 1 searches T up to n*T_opt. Corridor 0..24, pendant 25 at vertex 1,
garage 26 at vertex 10; robot 0: 0 -> 24, robot 1: 9 -> 25 (gap 0, no time limit). The
optimum needs robot 0 to wait (makespan 32 > the split's 29). The original returns null
for a time limit <= 0, so it gets a large limit. Prints [objective, lower bound].
"""
import _common
from _common import edges_graph

from mppflow.experiments import Experiments
from mppflow.problem import Problem
from mppflow.solvers import min_makespan

L = 24
g = edges_graph(L + 3, [(i, i + 1) for i in range(L)] + [(1, L + 1), (10, L + 2)])
p = Problem(g, [0, 9], [L, L + 1])
print("T_opt =", min_makespan(g, p.starts, p.goals).makespan, ", n * T_opt =", 2 * 24)
for c in ("original", "fixed"):
    ex = Experiments(compat=c, rng=1)
    tl = 1e6 if c == "original" else -1.0
    tt, td = ex.solve_problem_total_time(p, 0.0, tl), ex.solve_problem_distance(p, 0.0, tl)
    print(f"{c}: total time {[tt[0], tt[3]]}, total distance {[td[0], td[3]]}")
ex = Experiments(compat="fixed", rng=1, horizon="single")
print("fixed, horizon='single':", ex.solve_problem_total_time(p, 0.0, -1.0)[0], ex.solve_problem_distance(p, 0.0, -1.0)[0])
