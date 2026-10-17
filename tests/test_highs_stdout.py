"""The HiGHS backend drops scipy 1.17.1's stray native debug line from stdout.

``HighsMipSolverData::transformNewIntegerFeasibleSolution tmpSolver.run();`` is printed by
HiGHS's C++ code into the C ``stdout`` buffer on some MIPs (see ``mppflow.lp.highs``).
These tests run the solver in subprocesses with a real pipe as fd 1, and in-process under
pytest's own capture, and check that the line is gone, other output is kept in order, and
nothing breaks when fd 1 is closed or ``sys.stdout`` has no file descriptor (Jupyter).
"""

from __future__ import annotations

import os
import subprocess
import sys
import textwrap

import pytest

from mppflow.lp import highs

from helpers import child_env

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STRAY = highs.STRAY_LINE.decode()

# An equality knapsack on which scipy 1.17.1's HiGHS prints the line several times.
MODEL = textwrap.dedent("""
    import random
    from mppflow.lp import LinearModel

    def trigger_model(seed=11, n=40):
        r = random.Random(seed)
        w = [r.randint(1000, 9999) for _ in range(n)]
        sub = [r.random() < 0.5 for _ in range(n)]
        m = LinearModel("max")
        for _ in range(n):
            m.add_column(0, 1, "B")
        m.add_row([(i, float(w[i])) for i in range(n)], "==", float(sum(x for x, s in zip(w, sub) if s)))
        m.set_objective([(i, float(r.randint(1, 50))) for i in range(n)])
        return m
""")


def run(body: str, env: dict | None = None) -> subprocess.CompletedProcess:
    code = MODEL + textwrap.dedent(body)
    e = child_env()
    for k in ("MPPFLOW_HIGHS_RAW_STDOUT", "MPPFLOW_HIGHS_ISOLATE"):  # the defaults, unless env sets them
        e.pop(k, None)
    e.update(env or {})
    return subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, env=e, timeout=300,
                          cwd=ROOT)


SOLVE = """
    import sys
    from mppflow.lp import highs
    print("BEFORE", flush=True)
    s = highs.solve(trigger_model())
    print("AFTER", s.status, flush=True)
"""


def trigger_prints_line() -> bool:
    r = run(SOLVE, env={"MPPFLOW_HIGHS_RAW_STDOUT": "1"})
    assert r.returncode == 0, r.stderr
    return STRAY in r.stdout


def test_filter_keeps_other_native_output_in_order():
    """Independent of the HiGHS version: native writes to fd 1 inside the block survive,
    in order, except the stray line."""
    r = run("""
        import os, sys
        from mppflow.lp import highs
        print("one", flush=True)
        with highs.native_stdout_filtered() as active:
            os.write(1, b"two\\n" + highs.STRAY_LINE + b"\\n" + b"three\\n")
            print("four")                    # Python output inside the block is kept too
        print("five", flush=True)
        sys.stderr.write(f"active={active}\\n")
    """)
    assert r.returncode == 0, r.stderr
    assert r.stdout == "one\ntwo\nthree\nfour\nfive\n"
    assert "active=True" in r.stderr


def test_highs_solve_hides_the_stray_line():
    raw = trigger_prints_line()
    r = run(SOLVE)
    assert r.returncode == 0, r.stderr
    assert r.stdout == "BEFORE\nAFTER optimal\n"
    if not raw:
        pytest.skip("this HiGHS build does not print the debug line (filter checked above)")


def test_disp_true_is_not_redirected():
    """``params={"disp": True}``: fd 1 is still the caller's while HiGHS runs, so its log
    streams live (the filter would write it out only when the solve ends, which the output
    alone cannot tell apart). A spy on ``milp`` looks at fd 1 during each call; the default
    solve is the control (redirected)."""
    r = run("""
        import os, sys
        import scipy.optimize
        from mppflow.lp import highs
        real_milp = scipy.optimize.milp
        pipe = os.fstat(1)
        seen = {}
        def spy(*a, **k):
            fd1 = os.fstat(1)
            seen.setdefault(phase, set()).add(
                (highs._state["depth"], (fd1.st_dev, fd1.st_ino) == (pipe.st_dev, pipe.st_ino)))
            return real_milp(*a, **k)
        scipy.optimize.milp = spy
        phase = "default"
        highs.solve(trigger_model())
        phase = "disp"
        highs.solve(trigger_model(), params={"disp": True})
        sys.stderr.write(f"SEEN={sorted((k, sorted(v)) for k, v in seen.items())}\\n")
    """)
    assert r.returncode == 0, r.stderr
    assert "HiGHS" in r.stdout
    if highs._fflush is None:
        pytest.skip("no C fflush here: the filter never redirects")
    assert "SEEN=[('default', [(1, False)]), ('disp', [(0, True)])]" in r.stderr, r.stderr


def test_closed_fd1_and_fileless_sys_stdout():
    """fd 1 closed (daemon): the redirect is skipped; sys.stdout without fileno (Jupyter's
    OutStream): Python output still goes to sys.stdout, native noise is dropped."""
    r = run("""
        import io, os, sys
        from mppflow.lp import highs
        buf = io.StringIO()
        real = sys.stdout
        sys.stdout = buf                       # like ipykernel: no usable fileno()
        print("in notebook")
        s1 = highs.solve(trigger_model())
        print("still in notebook", s1.status)
        sys.stdout = real
        sys.stdout.flush()
        sys.stderr.write("BUF=" + repr(buf.getvalue()) + "\\n")
        os.close(1)                            # now no fd 1 at all
        s2 = highs.solve(trigger_model())
        with highs.native_stdout_filtered() as active:
            pass
        sys.stderr.write(f"closed: {s2.status} active={active}\\n")
    """)
    assert r.returncode == 0, r.stderr
    assert "BUF='in notebook\\nstill in notebook optimal\\n'" in r.stderr
    assert "closed: optimal active=False" in r.stderr
    assert STRAY not in r.stdout and r.stdout == ""


def test_concurrent_solves_share_one_redirect():
    r = run("""
        import os, threading
        from mppflow.lp import highs
        res = []
        ts = [threading.Thread(target=lambda k=k: res.append(highs.solve(trigger_model(11 + k % 3)).status))
              for k in range(4)]
        for t in ts: t.start()
        for t in ts: t.join()
        assert highs._state["depth"] == 0 and highs._state["saved"] is None
        os.fstat(1)
        print("done", sorted(res), flush=True)
    """)
    assert r.returncode == 0, r.stderr
    assert r.stdout == "done ['optimal', 'optimal', 'optimal', 'optimal']\n"


def test_under_pytest_fd_capture(capfd):
    print("py-before")
    s = highs.solve(_trigger_model())
    print("py-after")
    out, _ = capfd.readouterr()
    assert s.status == "optimal"
    assert out == "py-before\npy-after\n"
    assert highs._state["depth"] == 0


def test_under_pytest_sys_capture(capsys):
    print("py-before")
    s = highs.solve(_trigger_model())
    out, _ = capsys.readouterr()
    assert s.status == "optimal" and out == "py-before\n"


def _trigger_model():
    ns: dict = {}
    exec(MODEL, ns)
    return ns["trigger_model"]()
