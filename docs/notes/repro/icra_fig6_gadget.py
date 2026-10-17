"""ICRA 2013 Sec. III-C / Fig. 6: the diagonal-crossing gadget admits a head-on swap.

The gadget joins all four corners v_k(t)' of a grid cell to one unit-capacity middle arc and
splits it to all four corners v_l(t+1), edge-adjacent pairs included. So robot A can go
v0 -> v1 through the diagonal gadget while robot B goes v1 -> v0 through the ordinary edge
gadget: a head-on swap along the cell edge {v0, v1}, which Problem 1 forbids. We solve the
2-robot integer multiflow (T = 1) on one cell with and without the gadget.
"""
import _common

from mppflow import lp
from mppflow.lp import LinearModel

CORNERS = [0, 1, 2, 3]  # cell (0,0) (0,1) (1,0) (1,1)
EDGES = [(0, 1), (0, 2), (1, 3), (2, 3)]


def max_multiflow(starts, goals, diagonal_gadget):
    arcs = []  # (tail, head)
    for i, (s, g) in enumerate(zip(starts, goals)):
        arcs.append((("v", g, 1, 1), ("v", s, 0, 0)))  # loopback of robot i (arc index i)
    for v in CORNERS:
        arcs.append((("v", v, 0, 0), ("v", v, 1, 0)))  # green (wait)
        arcs.append((("v", v, 1, 0), ("v", v, 1, 1)))  # blue
    for (u, w) in EDGES:  # merge-split gadget of Fig. 4(b)
        m, s = ("m", u, w), ("s", u, w)
        arcs += [(("v", u, 0, 0), m), (("v", w, 0, 0), m), (m, s), (s, ("v", u, 1, 0)), (s, ("v", w, 1, 0))]
    if diagonal_gadget:  # Fig. 6: every corner in, one middle arc, every corner out
        arcs += [(("v", v, 0, 0), "D1") for v in CORNERS] + [("D1", "D2")]
        arcs += [("D2", ("v", v, 1, 0)) for v in CORNERS]
    n = len(starts)
    m = LinearModel("max")
    x = [[m.add_column(0, 0 if (j < n and j != i) else 1, "B") for j in range(len(arcs))] for i in range(n)]
    for j in range(len(arcs)):
        m.add_row([(x[i][j], 1.0) for i in range(n)], "<=", 1.0)
    for node in {a for arc in arcs for a in arc}:
        for i in range(n):
            terms = [(x[i][j], 1.0) for j, (a, b) in enumerate(arcs) if b == node]
            terms += [(x[i][j], -1.0) for j, (a, b) in enumerate(arcs) if a == node]
            m.add_row(terms, "==", 0.0)
    m.set_objective([(x[i][i], 1.0) for i in range(n)], "max")
    return round(lp.solve(m, backend="highs").objective)


print("swap v0 <-> v1 (cell edge), without the gadget:", max_multiflow([0, 1], [1, 0], False))
print("swap v0 <-> v1 (cell edge), with the gadget:   ", max_multiflow([0, 1], [1, 0], True))
print("crossing diagonals v0 -> v3, v1 -> v2, with the gadget:", max_multiflow([0, 1], [3, 2], True))
