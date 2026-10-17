"""T-RO 2016 Sec. VI-B (and ICRA 2013 Sec. VI-A): "the branching factor is over 1000" for the
16-puzzle. Under the papers' motion model the exact number is 950 (951 with the all-stay move).

On a fully occupied grid a joint move is a permutation sigma of the vertices with
sigma(v) in N[v] (stay or move along an edge) and no 2-cycles (no head-on swaps). We count
them by backtracking. Allowing swaps (not the papers' model) gives far more. For the 36-puzzle
(6x6, "well over 10^6" in the papers) backtracking is too slow, so a row-profile (transfer-matrix)
dynamic program counts the same permutations cell by cell; it agrees with backtracking up to 5x5.
"""
import _common
from _common import grid


def count(rows, cols, swaps=False):
    g = grid(rows, cols)
    n = rows * cols
    sigma, used = [None] * n, [False] * n

    def rec(v):
        if v == n:
            return 1
        total = 0
        for w in g.adj[v]:  # closed neighbourhood, ascending
            if used[w]:
                continue
            if not swaps and w < v and sigma[w] == v:  # w -> v and v -> w: head-on swap
                continue
            sigma[v], used[w] = w, True
            total += rec(v + 1)
            sigma[v], used[w] = None, False
        return total

    return rec(0)


def count_profile(rows, cols):
    """The same count as ``count(rows, cols)`` by a broken-profile DP in row-major order.

    Each vertex has one outgoing and one incoming move (stay = both to itself). The state holds,
    for each column, the vertical edge below the processed cells (0 none, 1 down, 2 up) and the
    horizontal edge to the right of the last processed cell (0 none, 1 right, 2 left). An edge
    carries at most one direction, which forbids the head-on swaps."""
    states = {((0,) * cols, 0): 1}
    for r in range(rows):
        for c in range(cols):
            nxt = {}
            for (vert, horiz), ways in states.items():
                top = vert[c] if r > 0 else 0
                left = horiz if c > 0 else 0
                ins = [d for d, on in (("top", top == 1), ("left", left == 1)) if on]
                outs = [d for d, on in (("top", top == 2), ("left", left == 2)) if on]
                if len(ins) > 1 or len(outs) > 1:
                    continue
                free = [d for d, ok in (("right", c + 1 < cols), ("bottom", r + 1 < rows)) if ok]
                options = []
                if not ins and not outs:
                    options.append(("self", "self"))
                for i in (ins or free):
                    for o in (outs or free):
                        if i != o:
                            options.append((i, o))
                for i, o in options:
                    right = 1 if o == "right" else 2 if i == "right" else 0
                    bottom = 1 if o == "bottom" else 2 if i == "bottom" else 0
                    key = (vert[:c] + (bottom,) + vert[c + 1:], right)
                    nxt[key] = nxt.get(key, 0) + ways
            states = nxt
    return states.get(((0,) * cols, 0), 0)


assert [count_profile(2, 3), count_profile(3, 4)] == [count(2, 3), count(3, 4)] == [7, 117]
for r in (2, 3, 4, 5):
    c = count(r, r)
    assert count_profile(r, r) == c
    print(f"{r}x{r}: joint moves incl. stay = {c}, non-trivial = {c - 1}"
          + (f", with head-on swaps allowed = {count(r, r, True)}" if r <= 4 else ""))
c = count_profile(6, 6)
print(f"6x6 (row-profile DP): joint moves incl. stay = {c}, non-trivial = {c - 1}")
