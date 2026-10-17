"""HiGHS speed on the makespan model of a large sparse grid: a 20x20 grid without obstacles and
4 robots (create_grid_problem(20, 20, 0.0, 4, MathRandom(3)), optimal makespan 27). With
presolve on (the default) the one ILP at T = 27 takes most of the time; with presolve off the
same makespan comes back one to two orders of magnitude faster. The LP relaxation at T = 27,
which the makespan loop solves first, is already integral here. Times vary with the machine, its load and
the scipy (HiGHS) version; see docs/notes/solver-notes.md section 1.
"""
import time

import _common  # noqa: F401  (repository root on sys.path)

import mppflow
from mppflow import MathRandom, lp
from mppflow.model import build_time_model

p = mppflow.create_grid_problem(20, 20, 0.0, 4, MathRandom(3))
for name, params in (("presolve on ", None), ("presolve off", {"presolve": False})):
    t0 = time.perf_counter()
    r = mppflow.min_makespan(p.graph, p.starts, p.goals, params=params)
    ilp = sum(a["runtime"] or 0.0 for a in r.attempts if a["phase"] == "ilp")
    print(f"{name}: {r.status} {r.objective} in {time.perf_counter() - t0:.1f} s (ILP: {ilp:.1f} s)")
m = build_time_model(p.graph, p.starts, p.goals, 27, True, True, None).model
s = lp.solve(m, backend="highs", relax=True)
frac = sum(1 for v in s.x if abs(v - round(v)) > 1e-6)
print(f"LP relaxation at T = 27: {s.status} {s.objective:.1f}, {frac} fractional of {m.num_columns} columns")
