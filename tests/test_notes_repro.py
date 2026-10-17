"""The reproduction scripts of docs/notes/repro/ print what docs/notes/*.md say they print.

Every ```text block that follows a "`python3 docs/notes/repro/NAME.py`" line is the documented
output of that script: the script must print exactly these lines, in this order, each documented
line matching a printed line of its own. A documented line that is exactly "..." marks elided
output: it stands for any number of printed lines (none included), and it is the only way a
printed line may be left out of a block. The scripts take
from a second to a few minutes each, so the test is marked slow. A script that reports Gurobi
results (it calls ``have_gurobi``) makes its blocks Gurobi tests (marker ``gurobi``).

Most scripts print only what every supported solver version gives (they use the deterministic
builtin backend, or print facts such as makespans and validity rather than one of several optimal
plans). A few are about a specific HiGHS: their output starts with a ``scipy X.Y.Z`` line (scipy
bundles HiGHS), and such a block is compared only when that scipy version is installed.
"""
from __future__ import annotations

import difflib
import glob
import os
import re
import shlex
import subprocess
import sys

import pytest

from helpers import child_env, needs_gurobi

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NOTES = os.path.join(ROOT, "docs", "notes")
BLOCK = re.compile(r"`python3 docs/notes/repro/(\w+)\.py([^`]*)`\s*\n\s*\n?```text\n(.*?)```", re.S)

SCIPY_LINE = re.compile(r"^scipy (\S+)$")
"""First line of a block documented for one scipy version (its bundled HiGHS)."""

ELISION = "..."
"""A documented line that stands for printed lines left out (any other line is compared, also
one that contains "...", such as the original's log "Trying T = 4 ... relaxed model")."""

WALL_CLOCK = {"highs_makespan_presolve_speed", "solver_backends", "builtin_tt532"}
"""Scripts that print wall-clock seconds: in their lines ``<number> s`` is not compared (in the
others it is: "budget 1.751 s" is not "budget 1.75 s")."""

MACHINE_DEPENDENT = {
    # the sub-ILPs get the time left on a virtual clock as a real (HiGHS wall-clock) limit
    "bug_wallclock_heuristic": re.compile(r"^loaded "),
}
"""Documented lines that depend on the machine's speed and load: not compared."""

_M320_INFEASIBLE = r"objective None, The problem is infeasible\. \(HiGHS Status 8: .*\)"
NONDETERMINISTIC = {
    # the m320 presolve defect of scipy 1.17.1 and 1.18.1 segfaults in most runs and answers "infeasible" in the
    # others (docs/notes/solver-notes.md); in isolate mode both end in the retry with presolve off
    "highs_presolve_bug": {
        "m320 raw milp, presolve on : ": re.compile(rf"process died \(exit code -11\)|{_M320_INFEASIBLE}"),
        "m320 mppflow highs         : ": re.compile(r"process died \(exit code -11\)|infeasible None"),
        "m320 mppflow highs isolate : ": re.compile(
            r"optimal 1\.0 \(first attempt: HiGHS status (-2|8); retried with presolve off\)"),
    },
}
"""Documented lines whose outcome varies between runs: ``{script: {prefix: accepted}}``. A
documented line that starts with ``prefix`` and is itself ``accepted`` is matched by any printed
line with that prefix whose rest is ``accepted`` (other documented lines with the prefix, such as
those of another scipy version, are compared exactly)."""


def _nondeterministic(name: str, line: str):
    """The pattern for the rest of a documented line of script ``name`` with a varying outcome
    (``(prefix, pattern)``), or ``None``."""
    for prefix, accepted in NONDETERMINISTIC.get(name, {}).items():
        if line.startswith(prefix) and accepted.fullmatch(line[len(prefix):]):
            return prefix, accepted
    return None


def _matches(line: str, printed: str, name: str) -> bool:
    """Whether documented ``line`` of script ``name`` is matched by the printed line ``printed``."""
    alt = _nondeterministic(name, line)
    if alt is None:
        return _normalized(line, name) == _normalized(printed, name)
    prefix, accepted = alt
    return printed.startswith(prefix) and accepted.fullmatch(printed[len(prefix):]) is not None


def _printed(line: str, got, name: str) -> bool:
    """Whether documented ``line`` of script ``name`` matches one of the printed lines ``got``."""
    return any(_matches(line, g, name) for g in got)


def _missing(expected: list[str], printed: list[str], name: str) -> list[str]:
    """The documented lines of script ``name`` that the printed lines do not match in order:
    each compared line takes the first printed line after the previous match that matches it
    (taking the first is never worse, so an empty result means an embedding exists)."""
    skip = MACHINE_DEPENDENT.get(name)
    missing, i = [], 0
    for line in expected:
        if line.strip() == ELISION or (skip and skip.search(line)):
            continue
        j = next((j for j in range(i, len(printed)) if _matches(line, printed[j], name)), None)
        if j is None:
            missing.append(line)
        else:
            i = j + 1
    return missing


def _covers(expected: list[str], printed: list[str], name: str) -> bool:
    """Whether the documented lines of script ``name`` account for every printed line: the
    compared lines match printed lines in order, and printed lines between them are allowed only
    where the block has a "..." line (a machine-dependent documented line matches one printed
    line that is machine-dependent too)."""
    skip = MACHINE_DEPENDENT.get(name)

    def one(line: str, got: str) -> bool:
        if skip and skip.search(line):
            return skip.search(got) is not None
        return _matches(line, got, name)

    # reach[j]: the first i documented lines can account for exactly the first j printed lines
    reach = [True] + [False] * len(printed)
    for line in expected:
        if line.strip() == ELISION:
            for j in range(1, len(reach)):
                reach[j] = reach[j] or reach[j - 1]
        else:
            reach = [False] + [reach[j] and one(line, printed[j]) for j in range(len(printed))]
    return reach[-1]


def _normalized(line: str, name: str) -> str:
    """``line`` with wall-clock seconds (``..., 20 s``) replaced by ``<t>`` if ``name`` is a
    :data:`WALL_CLOCK` script."""
    return re.sub(r"\b\d+(\.\d+)? s\b", "<t> s", line) if name in WALL_CLOCK else line


def _blocks() -> list[tuple[str, str, list[str]]]:
    out = []
    for md in sorted(glob.glob(os.path.join(NOTES, "*.md"))):
        with open(md, encoding="utf-8") as f:
            text = f.read()
        for m in BLOCK.finditer(text):
            lines = [line.rstrip() for line in m.group(3).strip().splitlines()]
            out.append((m.group(1), m.group(2).strip(), lines))
    return out


BLOCKS = _blocks()


def test_every_repro_script_is_documented():
    """Each script of docs/notes/repro/ (not the helper) has a documented output block, and
    every block names an existing script."""
    scripts = {os.path.splitext(os.path.basename(p))[0] for p in glob.glob(os.path.join(NOTES, "repro", "*.py"))}
    scripts.discard("_common")
    assert {name for name, _, _ in BLOCKS} == scripts


def test_nondeterministic_lines_are_documented():
    """Each varying-outcome pattern covers a documented line, and every alternative it accepts
    is one that docs/notes/solver-notes.md describes (checked on hand-written outcomes)."""
    for name, pats in NONDETERMINISTIC.items():
        lines = [line for n, _, block in BLOCKS if n == name for line in block]
        for prefix in pats:
            assert any(_nondeterministic(name, line) for line in lines), (name, prefix)
    got = {"m320 raw milp, presolve on : objective None, The problem is infeasible. (HiGHS Status 8: "
           "model_status is Infeasible; primal_status is None)",
           "m320 mppflow highs         : infeasible None",
           "m320 mppflow highs isolate : optimal 1.0 (first attempt: HiGHS status 8; retried with presolve off)"}
    doc = [line for n, _, block in BLOCKS if n == "highs_presolve_bug" and block[0] == "scipy 1.17.1"
           for line in block if line.startswith("m320 ") and "raw milp, presolve off" not in line]
    assert len(doc) == 3 and all(_printed(line, got, "highs_presolve_bug") for line in doc)
    wrong = {"m320 raw milp, presolve on : objective 1.0, Optimization terminated successfully. (HiGHS Status 7: Optimal)",
             "m320 mppflow highs         : optimal 1.0",
             "m320 mppflow highs isolate : optimal 1.0"}
    assert not any(_printed(line, wrong, "highs_presolve_bug") for line in doc)


def _reports_gurobi(name: str) -> bool:
    with open(os.path.join(NOTES, "repro", name + ".py"), encoding="utf-8") as f:
        return "have_gurobi" in f.read()


def test_documented_lines_are_matched_in_order():
    """The comparison of :func:`test_repro_script_prints_the_documented_output`, on hand-written
    output: order and multiplicity count, "..." inside a line is compared, wall-clock seconds
    only in the WALL_CLOCK scripts. Regression (TS3B-NOTES-REPRO-SECTION-UNVERIFIABLE): case (d)
    of bug_time_limits repeats two lines of case (b) and its other two lines contain "...";
    set membership and the "..." rule passed any output for it."""
    doc = next(b for n, _, b in BLOCKS if n == "bug_time_limits")
    d = doc.index(next(line for line in doc if line.startswith("(d) ")))
    assert _missing(doc, doc, "bug_time_limits") == []
    assert sum("..." in line for line in doc[d:]) == 2 and doc[d + 1:d + 3] == doc[4:6]
    wrong = doc[:d + 1] + ["  original: status optimal, makespan 4", "  fixed: status time_limit, makespan None",
                           "  original, LP relaxation first: WRONG", "  fixed, LP relaxation first: WRONG"] + doc[d + 5:]
    assert _missing(doc, wrong, "bug_time_limits") == doc[d + 1:d + 5]
    assert _missing(["a", "b"], ["b", "a"], "x") == ["b"] and _missing(["a", "a"], ["a"], "x") == ["a"]
    assert _missing(["a", ELISION, "b"], ["a", "c", "b"], "x") == []
    assert _missing(["budget 1.75 s"], ["budget 1.751 s"], "x") == ["budget 1.75 s"]
    assert _missing(["builtin: optimal 1, 57 s"], ["builtin: optimal 1, 3.5 s"], "builtin_tt532") == []
    # every printed line must be documented, or stand under a "..." line (D2-E11: a block that
    # showed only the first 2 of the 4 printed lines passed the ordered check)
    assert _covers(doc, doc, "bug_time_limits")
    assert not _covers(["a", "b"], ["a", "b", "c"], "x") and not _covers(["a", "c"], ["a", "b", "c"], "x")
    assert _covers(["a", ELISION, "c"], ["a", "b", "b", "c"], "x") and _covers(["a", ELISION], ["a"], "x")
    assert not _covers(["a", ELISION, "c"], ["a", "b"], "x")
    assert _covers(["loaded 3 s"], ["loaded 7 s"], "bug_wallclock_heuristic")
    assert not _covers(["loaded 3 s"], ["other"], "bug_wallclock_heuristic")


@pytest.mark.slow
@pytest.mark.parametrize("name,args,expected",
                         [pytest.param(n, a, e, marks=[needs_gurobi] if _reports_gurobi(n) else [])
                          for n, a, e in BLOCKS],
                         ids=[f"{n}{'-' + a.replace(' ', '-') if a else ''}-{i}" for i, (n, a, _) in enumerate(BLOCKS)])
def test_repro_script_prints_the_documented_output(name, args, expected):
    m = SCIPY_LINE.match(expected[0]) if expected else None
    if m:
        import scipy

        if scipy.__version__ != m.group(1):
            pytest.skip(f"the documented output is for scipy {m.group(1)} (its bundled HiGHS); "
                        f"installed: scipy {scipy.__version__}")
    path = os.path.join(NOTES, "repro", name + ".py")
    env = child_env()
    for k in ("MPPFLOW_HIGHS_RAW_STDOUT", "MPPFLOW_HIGHS_ISOLATE"):
        env.pop(k, None)
    r = subprocess.run([sys.executable, path, *shlex.split(args)], cwd=ROOT, env=env, capture_output=True,
                       text=True, timeout=1800)
    assert r.returncode == 0, r.stderr[-3000:]
    printed = [line.rstrip() for line in r.stdout.splitlines()]
    missing = _missing(expected, printed, name)
    assert not missing, ("documented but not printed (in this order):\n" + "\n".join(missing)
                         + "\n--- printed:\n" + r.stdout[-3000:])
    assert _covers(expected, printed, name), (
        "printed lines that the block leaves out without a '...' line:\n"
        + "\n".join(difflib.unified_diff(expected, printed, "documented", "printed", lineterm="")))
