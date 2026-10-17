"""A reduced version of our HiGHS stress test on MPP models: random small grid instances, the
four compact models (makespan, total time, total distance, max distance) at three horizons
each, solved by mppflow's HiGHS backend with presolve on and off (and by Gurobi when it is
available). Usage: python3 highs_mpp_stress.py [number of instances, default 50].
"""
import random
import sys

import _common
from _common import have_gurobi

from mppflow import lp
from mppflow import model as M
from mppflow._java import MathRandom
from mppflow.pathfinder import get_distances_to_all_vertices
from mppflow.problem import create_grid_problem

N = int(sys.argv[1]) if len(sys.argv) > 1 else 50
gurobi = have_gurobi()
builders = [("time", M.build_time_model), ("tt", M.build_total_time_model),
            ("dist", M.build_distance_model), ("maxd", M.build_max_distance_model)]
models = mismatches = 0
for seed in range(N):
    r = random.Random(seed)
    rows, cols, obs = r.randint(3, 5), r.randint(3, 5), r.choice([0, 0.1, 0.2])
    n = r.randint(2, max(2, int(rows * cols * (1 - obs) * 0.4)))
    p = create_grid_problem(cols, rows, obs, n, rng=MathRandom(seed))
    g, s, t = p.graph, p.starts, p.goals
    lb = max(get_distances_to_all_vertices(g, a)[b] for a, b in zip(s, t))
    for kind, build in builders:
        for T in (lb, lb + 1, lb + 3):
            m = build(g, s, t, T).model
            vals = [lp.solve(m, backend="highs", mip_gap=0.0).objective,
                    lp.solve(m, backend="highs", mip_gap=0.0, params={"presolve": False}).objective]
            if gurobi:
                vals.append(lp.solve(m, backend="gurobi", mip_gap=0.0, params={"Threads": 1}).objective)
            models += 1
            ref = vals[-1]
            if any((v is None) != (ref is None) or (v is not None and abs(v - ref) > 1e-6) for v in vals):
                mismatches += 1
                print("mismatch:", seed, kind, T, vals)
print(f"{models} models from {N} instances; solvers compared: HiGHS presolve on/off"
      + (" and Gurobi" if gurobi else "") + f"; mismatches: {mismatches}")
