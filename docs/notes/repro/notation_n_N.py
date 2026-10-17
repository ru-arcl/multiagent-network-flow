"""T-RO 2016 Sec. VI-B writes "N^2-puzzle for n = 3, 4, 5, and 6", but n is the number of
robots (Sec. II). The side length N is meant: the N^2-puzzle has n = N^2 robots.
"""
import _common

from mppflow._java import MathRandom
from mppflow.problem import create_n2_puzzle

for N in (3, 4, 5, 6):
    p = create_n2_puzzle(N, rng=MathRandom(1))
    print(f"N={N}: {p.graph.n} vertices, n={p.n_agents} robots")
