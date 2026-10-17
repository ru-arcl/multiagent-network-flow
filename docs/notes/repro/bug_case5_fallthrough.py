"""Main.main case 5 has no `break` and falls through into case 6 (Main.java:73-77).

Case 5 solves a random 16x16 instance with 60 robots; case 6 then builds the 17-vertex
spider tree and solves a second random instance. We count the robots whose paths are
printed and check whether case 6's "Adjacency list:" header appears. Uses Gurobi
(Threads=1) when a full license is available, else HiGHS; slow (minutes with HiGHS). On
HiGHS the original prints 60 'Agent' lines, not 68: case 6's path set is rejected because
the original needs exact binaries (original-bugs.md B9).
"""
import warnings

import _common
from _common import have_gurobi

from mppflow.experiments import Experiments

backend, params = ("gurobi", {"Threads": 1}) if have_gurobi() else ("highs", None)
warnings.simplefilter("ignore")  # HiGHS + original: inexact-binary warnings (see solver-notes.md)
for c in ("original", "fixed"):
    log = Experiments(compat=c, rng=1, backend=backend, params=params).main(["5"])
    print(f"{c}: 'Agent' lines printed: {log.count('Agent ')}, case 6 ran: {'Adjacency list:' in log}")
