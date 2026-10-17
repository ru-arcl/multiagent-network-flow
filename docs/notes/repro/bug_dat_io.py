"""Problem/Graph file I/O of the original: write and read failures.

1. Graph.writeObject stores each vertex in a fixed int[13] row (id, x, y, degree, 9
   adjacency entries incl. the self-loop): a vertex with 9 neighbours overflows.
2. A vertex without coordinates (Graph.addVertex(int, int[])) makes both writers throw a
   NullPointerException, after leaving a truncated .dat / an empty .txt behind.
3. Problem.readFromFile swallows every I/O error and returns a Problem with graph = sg = null.
4. The text writer calls new File(name).getParentFile().mkdirs(): a name without a directory
   part (every default name on Linux, where '\\' is not a separator) throws an NPE.
"""
import contextlib
import io
import os

import _common
from _common import edges_graph, show

from mppflow.graph import Graph
from mppflow.problem import Problem, read_dat, write_dat, write_txt

tmp = _common.scratch_dir()
star9 = Graph.from_adjacency([list(range(1, 10))] + [[0]] * 9, xy=[(i, 0) for i in range(10)])
nocoord = edges_graph(4, [(0, 1), (1, 2), (1, 3)])  # Main case 3 graph, no coordinates
os.chdir(tmp)
os.mkdir("out")
for c in ("original", "fixed"):
    f1, f2, f3 = (f"out/{c}-{k}" for k in ("star.dat", "nocoord.dat", "nocoord.txt"))
    show(f"{c}: write .dat, vertex with 9 neighbours", write_dat, Problem(star9, [1], [2]), f1, compat=c)
    print(f"   file size {os.path.getsize(f1) if os.path.exists(f1) else None}")
    show(f"{c}: write .dat, no coordinates", write_dat, Problem(nocoord, [0], [3]), f2, compat=c)
    show(f"{c}: write .txt, no coordinates", write_txt, Problem(nocoord, [0], [3]), f3, compat=c)
    print(f"   .dat size {os.path.getsize(f2) if os.path.exists(f2) else None}, "
          f".txt size {os.path.getsize(f3) if os.path.exists(f3) else None}")
    err = io.StringIO()
    with contextlib.redirect_stderr(err):
        p = show(f"{c}: read a missing file", read_dat, "missing.dat", compat=c)
    if err.getvalue():
        print("   stderr:", err.getvalue().strip())
    if p is not None:
        print(f"   graph={p.graph} sg={p.sg}")
    show(f"{c}: write .txt named '\\8x8-grid\\x.txt' (no '/')", write_txt,
         Problem(star9, [1], [2]), "\\8x8-grid\\x.txt", compat=c)
