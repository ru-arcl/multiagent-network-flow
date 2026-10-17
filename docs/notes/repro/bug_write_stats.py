"""Main.writeStats: the median/min/max are taken over a TreeSet<Double> (Main.java:369, 429-430),
so equal times collapse; with no success, ft[0] throws ArrayIndexOutOfBoundsException, which
is caught and printed, and no row is written. writeHeuristicStats divides the time of all
runs by the number of successes (Main.java:409-425, 443). Run in an empty folder (the rows are
appended to <basePath>\\n2puzzle-result.txt).
"""
import _common

from mppflow.experiments import Experiments

_common.scratch_dir(chdir=True)
for c in ("original", "fixed"):
    ex = Experiments("./" if c == "original" else ".", compat=c)
    ex.reset_stats("times 1,1,1,5 s: ")
    for ms in (1000, 1000, 1000, 5000):
        ex.process_stats([10, ms, 0, 7])  # {solution, time ms, -, lower bound}
    print(f"{c}: {ex.write_stats()!r}")
    ex.reset_stats("no success: ")
    print(f"{c}: {ex.write_stats()!r}; stderr: {ex.stderr.strip().splitlines()}")
for c in ("original", "fixed"):
    ex = Experiments("./" if c == "original" else ".", compat=c)
    ex.reset_heuristic_stats("1 s all goals, 9 s partial: ")
    ex.process_heuristic_stats([10, 1000, 5, 20, 20], 5)  # {length, time ms, goals reached, steps, exp}
    ex.process_heuristic_stats([10, 9000, 3, 20, 20], 5)
    print(f"{c}: {ex.write_heuristic_stats()!r}")
