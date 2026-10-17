"""The three backends on Main case 3 (star 0-1, 1-2, 1-3, robots swap 0 and 3), as
Main.solveProblem runs it (LP relaxation first, compat="original"). The live original (JDK 17,
Gurobi 13.0.2, Threads=1) prints Agent 0: 0 0 1 3 3 and Agent 1: 3 1 2 1 0. Only Gurobi with
the same version and Threads=1 is expected to return the very same plan; every backend
returns an optimal one. Then HiGHS and the builtin solver on growing grids (20 s limit).
"""
import time

import _common
from _common import edges_graph, have_gurobi

from mppflow._java import MathRandom
from mppflow.problem import create_grid_problem
from mppflow.solvers import min_makespan, solve_time

star = edges_graph(4, [(0, 1), (1, 2), (1, 3)])
backends = [("highs", None), ("builtin", None)] + ([("gurobi", {"Threads": 1})] if have_gurobi() else [])
for backend, params in backends:
    plans = {str(solve_time(star, [0, 3], [3, 0], True, compat="original", backend=backend, params=params,
                            rng=MathRandom(1)).paths) for _ in range(3)}
    print(f"{backend}{' Threads=1' if params else ''}: {len(plans)} distinct plan(s) in 3 runs: {sorted(plans)}")
for side, n in ((3, 3), (5, 8), (6, 12)):
    p = create_grid_problem(side, side, 0.0, n, rng=MathRandom(1))
    for backend in ("highs", "builtin"):
        t0 = time.time()
        r = min_makespan(p.graph, p.starts, p.goals, backend=backend, rng=MathRandom(1), time_limit=20)
        print(f"{side}x{side}, {n} robots, {backend}: {r.status}, makespan {r.makespan}, {time.time() - t0:.0f} s")
