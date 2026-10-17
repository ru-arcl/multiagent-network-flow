"""Main.main argument handling: Integer.parseInt on a non-number (Main.java:16) and case 4
without a file name (argv[1], Main.java:67) throw; a case number outside 0..6 matches no
branch of the switch (no default), so it does nothing and exits normally.
"""
import _common
from _common import show

from mppflow.experiments import Experiments

for c in ("original", "fixed"):
    show(f"{c}: main(['x'])", Experiments(compat=c).main, ["x"])
    show(f"{c}: main(['4'])", Experiments(compat=c).main, ["4"])
    show(f"{c}: main(['7']) prints", lambda: repr(Experiments(compat=c).main(["7"])))
