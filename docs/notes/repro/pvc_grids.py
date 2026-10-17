"""Grid conventions of the experiments (advanced/Problem.java:126-140, 202-250).

* createGridProblem(col, row, ...) passes col as the number of rows: the "24x18" instances
  have 24 rows and 18 columns (T-RO Fig. 9 is drawn rotated).
* Obstacles remove (int)(rows*cols*frac) vertices: 24x18 at 0..30% (the generator and the
  drivers go to 30%; the paper plots 0-25%).
* 8x8 and 16x16 ("crowded", Sec. VI-C) have no obstacles: the paper's robot-vertex ratios
  (60 robots = 94%, 190 robots = 74%) only hold for 64 and 256 vertices.
"""
import _common

from mppflow._java import MathRandom
from mppflow.problem import create_grid_problem

p = create_grid_problem(24, 18, 0.0, 10, rng=MathRandom(1))
xs = [p.graph.xy[v][0] for v in p.graph.vertices]
ys = [p.graph.xy[v][1] for v in p.graph.vertices]
print(f"create_grid_problem(24, 18, 0, 10): {max(ys) + 1} rows x {max(xs) + 1} columns")
print("24x18 vertices left per obstacle level:",
      {obs: create_grid_problem(24, 18, obs / 100, 10, rng=MathRandom(1)).graph.n for obs in range(0, 35, 5)})
for side, n in ((8, 60), (16, 190)):
    for frac in (0.0, 0.2):
        V = create_grid_problem(side, side, frac, 10, rng=MathRandom(1)).graph.n
        print(f"{side}x{side} with {frac:.0%} obstacles: {V} vertices, {n} robots = {n / V:.1%}")
