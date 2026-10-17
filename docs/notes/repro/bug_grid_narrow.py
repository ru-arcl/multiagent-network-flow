"""Graph.create2DGridGraph (cols == 1) and create2DGridGraph8Connected (cols <= 2) lose edges.

Neighbour ids are y*cols + x + 1 and the boundary rules remove id-1 / id+1 etc. by id
arithmetic. With cols == 1, id +- 1 == id +- cols, so the vertical edges are removed; in the
8-connected builder id +- cols -+ 1 aliases too (cols == 2: asymmetric adjacency; cols == 1:
even the self-loops go). update=False keeps the original 1-based ids.
"""
import _common

from mppflow.graph import create_2d_grid_graph, create_2d_grid_graph_8connected

for compat in ("original", "fixed"):
    print(compat)
    print("  4-connected 3x1:", create_2d_grid_graph(3, 1, False, compat=compat).adj)
    print("  8-connected 2x2:", create_2d_grid_graph_8connected(2, 2, False, compat=compat).adj)
    print("  8-connected 3x1:", create_2d_grid_graph_8connected(3, 1, False, compat=compat).adj)
