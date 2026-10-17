"""The builtin solver on a degenerate LP: the relaxation of the total-time model on a 3x3 grid
(starts 8 2 0 7 6 1, goals 8 7 2 5 0 4, T = 6; 532 columns), the tt532 case of
tests/test_oracles.py. Before the numerical safeguards of solver-notes.md section 4 it hit the
iteration cap; now it reaches the HiGHS value 23.333... in 32635 simplex iterations (the count
of exact rational arithmetic). About a minute in CPython.
"""
import time

from _common import grid

from mppflow import lp
from mppflow.model import build_total_time_model

m = build_total_time_model(grid(3, 3), [8, 2, 0, 7, 6, 1], [8, 7, 2, 5, 0, 4], 6).model
print(f"columns {m.num_columns}, rows {m.num_rows}")
h = lp.solve(m, backend="highs", relax=True)
print(f"highs:   {h.status} {h.objective:.6f}")
t0 = time.perf_counter()
b = lp.solve(m, backend="builtin", relax=True)
print(f"builtin: {b.status} {b.objective:.6f}, {b.iterations} iterations "
      f"({time.perf_counter() - t0:.0f} s, varies)")
