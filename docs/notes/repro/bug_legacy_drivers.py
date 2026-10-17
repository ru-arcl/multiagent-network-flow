"""Legacy drivers ask for files the generator never writes (Main.java:966-967, 982):
run32x32PerformanceMetricHeuristic drops "-agts-" from the name, and
run32x32PerformanceMetricSplitting8Connected reads the plain-text .txt as serialized data
(and steps n by 25, which the generator never produces). readFromFile swallows the error
and the next p.sg[0] throws NullPointerException. Run in an empty folder.
"""
import _common
from _common import show

from mppflow.experiments import Experiments

_common.scratch_dir(chdir=True)
for c in ("original", "fixed"):
    for drv in ("32x32-heuristic", "32x32-splitting-8c"):
        ex = Experiments(base_path="./" if c == "original" else ".", compat=c, rng=1)
        show(f"{c} {drv}", ex.run_driver, drv)
        if ex.stderr:
            print("   stderr:", ex.stderr.strip().splitlines()[0])
