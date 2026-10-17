#!/usr/bin/env python3
"""Record golden fixtures from the live original Java code.

    python3 tools/golden/record.py --original-src ~/src/optimal-mrppg-journal/source \\
        --gurobi-jar $GUROBI_HOME/lib/gurobi.jar [--jdk /path/to/jdk] [--work DIR] [--out tests/golden]

Steps: build the harness (build.sh), run the phase-1 cases (instances from the original
generators), generate the phase-2 cases from them (cases.py), run those, and write one JSON
line per case to ``<out>/<group>.jsonl`` plus ``<out>/META.json``.  The harness runs in a work
directory containing ``gurobi.env`` with ``Threads 1``.  A case that does not return within its
budget is recorded as ``{"hang": true}``; the harness then exits and is restarted with the
remaining cases.  See README.md for the record format.
"""
from __future__ import annotations

import argparse
import datetime as _dt
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import cases as case_spec  # noqa: E402

EXIT_HANG = 3
SPEC_KEYS = ("id", "fn", "args", "seed", "statics", "budget", "tags", "note", "nondet")
HARNESS_KEYS = ("id", "fn", "args", "seed", "statics", "budget")

_NUM = r"-?[0-9]+(?:\.[0-9]+)?(?:E-?[0-9]+)?"
WALLCLOCK_PATTERNS = [
    (re.compile(r"(found in )" + _NUM + r"( seconds)"), r"\1<t>\2"),
    (re.compile(r"(max time limit )" + _NUM), r"\1<t>"),
    (re.compile(r"(Time spent = )" + _NUM + r"( seconds)"), r"\1<t>\2"),
    (re.compile(r"(Total time spent: )" + _NUM + r"( seconds)"), r"\1<t>\2"),
    (re.compile(r"(, time: )" + _NUM), r"\1<t>"),
]


def normalize_wallclock(s: str | None) -> str | None:
    if s is None:
        return None
    for pat, rep in WALLCLOCK_PATTERNS:
        s = pat.sub(rep, s)
    return s


def clean_stderr(s: str) -> str:
    """Keep exception lines and frames of the original code; drop JDK/harness frames."""
    out = []
    for line in s.splitlines():
        if line.startswith("\tat "):
            frame = line[4:]
            if frame.startswith("projects.multipath.") and not frame.startswith("projects.multipath.ILP.Harness"):
                out.append(line)
            continue
        if re.match(r"^\t\.\.\. \d+ more$", line):
            continue
        out.append(line)
    return "\n".join(out) + ("\n" if out else "")


# ----------------------------------------------------------------------------------------------
# Harness invocation
# ----------------------------------------------------------------------------------------------

class Runner:
    def __init__(self, java: str, classpath: str, libdir: str, rundir: Path, log):
        self.java, self.classpath, self.libdir, self.rundir, self.log = java, classpath, libdir, rundir, log
        self.restarts = 0

    def run(self, cases: list[dict]) -> dict[str, dict]:
        results: dict[str, dict] = {}
        remaining = list(cases)
        env = dict(os.environ)
        env["LD_LIBRARY_PATH"] = self.libdir + (":" + env["LD_LIBRARY_PATH"] if env.get("LD_LIBRARY_PATH") else "")
        while remaining:
            cfile, ofile, pfile = (self.rundir / n for n in ("cases.jsonl", "out.jsonl", "progress.txt"))
            with open(cfile, "w") as f:
                for c in remaining:
                    f.write(json.dumps({k: c[k] for k in HARNESS_KEYS if k in c}) + "\n")
            for p in (ofile, pfile):
                if p.exists():
                    p.unlink()
            timeout = sum(c.get("budget", 120) for c in remaining) + 300
            cmd = [self.java, "--add-opens", "java.base/java.lang=ALL-UNNAMED", "-Xss64m", "-cp", self.classpath,
                   "projects.multipath.ILP.Harness", str(cfile), str(ofile), str(pfile), str(self.rundir / "work")]
            t0 = time.time()
            try:
                p = subprocess.run(cmd, cwd=self.rundir, env=env, stdout=self.log, stderr=self.log, timeout=timeout)
                code = p.returncode
            except subprocess.TimeoutExpired:
                code = -9
            done = []
            if ofile.exists():
                for line in ofile.read_text(encoding="utf-8").splitlines():
                    if line.strip():
                        r = json.loads(line)
                        results[r["id"]] = r
                        done.append(r["id"])
            print(f"  harness: {len(done)} case(s) in {time.time() - t0:.1f}s, exit {code}", file=sys.stderr)
            done_set = set(done)
            if code not in (0, EXIT_HANG):
                cur = pfile.read_text().strip() if pfile.exists() else None
                if cur and cur not in done_set:
                    results[cur] = {"id": cur, "jvm_exit": code}
                    done_set.add(cur)
                    print(f"  JVM died (exit {code}) in case {cur}", file=sys.stderr)
                elif not done:
                    raise RuntimeError(f"harness failed to start (exit {code}); see the log")
            remaining = [c for c in remaining if c["id"] not in done_set]
            if remaining:
                self.restarts += 1
        return results


# ----------------------------------------------------------------------------------------------
# Record assembly
# ----------------------------------------------------------------------------------------------

def assemble(spec: dict, out: dict) -> dict:
    rec = {k: spec[k] for k in SPEC_KEYS if k in spec}
    norm = spec.get("normalize", True)
    if "jvm_exit" in out:
        rec["jvm_exit"] = out["jvm_exit"]
        return rec
    if out.get("hang"):
        rec["hang"] = True
        rec["hang_stack"] = out.get("hang_stack", [])
    elif "exception" in out:
        exc = dict(out["exception"])
        if norm:
            exc["message"] = normalize_wallclock(exc.get("message"))
        rec["exception"] = exc
    else:
        rec["result"] = out.get("result")
    for k in ("mutated", "extra"):
        if k in out:
            rec[k] = out[k]
    stdout = out.get("stdout", "")
    rec["stdout"] = normalize_wallclock(stdout) if norm else stdout
    stderr = clean_stderr(out.get("stderr", ""))
    if stderr:
        rec["stderr"] = normalize_wallclock(stderr) if norm else stderr
    if not out.get("hang"):
        rec["rng_next"] = out.get("rng_next")
        rec["draws"] = out.get("draws")
    if "statics_after" in out:
        rec["statics_after"] = out["statics_after"]
    if "files" in out:
        files = out["files"]
        if norm and isinstance(files, dict):
            files = {n: ({**f, "text": normalize_wallclock(f["text"])} if isinstance(f, dict) and "text" in f else f)
                     for n, f in files.items()}
            # size and hash of a normalized text would be misleading
            for f in files.values():
                if isinstance(f, dict) and "text" in f and "<t>" in f["text"]:
                    f.pop("sha256", None)
                    f.pop("size", None)
        rec["files"] = files
    return rec


def strip_for_compare(rec: dict) -> dict:
    r = {k: v for k, v in rec.items() if k not in ("budget", "hang_stack")}  # hang_stack: informational
    for k in rec.get("nondet", []):
        r.pop(k, None)
    return r


def git_rev(path: Path) -> str | None:
    try:
        return subprocess.run(["git", "-C", str(path), "rev-parse", "HEAD"], capture_output=True, text=True,
                              check=True).stdout.strip()
    except Exception:
        return None


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--original-src", required=True, help="the 'source' directory of the original repository")
    ap.add_argument("--gurobi-jar", default=os.path.join(os.environ.get("GUROBI_HOME", ""), "lib", "gurobi.jar"))
    ap.add_argument("--jdk", help="JDK home (bin/java, bin/javac); default: java/javac on PATH")
    ap.add_argument("--work", help="work directory (default: a temporary directory, removed afterwards)")
    ap.add_argument("--classes", help="use an existing harness build instead of running build.sh")
    ap.add_argument("--out", default=str(HERE.parent.parent / "tests" / "golden"))
    ap.add_argument("--only", help="comma-separated groups to record (default: all)")
    ap.add_argument("--filter", help="regular expression on case ids (phase 2 only; implies no META rewrite)")
    ap.add_argument("--check-determinism", action="store_true",
                    help="run phase 2 twice and report records that differ (excluding 'nondet' fields)")
    args = ap.parse_args(argv)

    java = os.path.join(args.jdk, "bin", "java") if args.jdk else "java"
    javac = os.path.join(args.jdk, "bin", "javac") if args.jdk else "javac"
    jar = os.path.abspath(args.gurobi_jar)
    if not os.path.isfile(jar):
        ap.error(f"gurobi.jar not found: {jar}")
    work = Path(args.work).resolve() if args.work else Path(tempfile.mkdtemp(prefix="mppflow-golden-"))
    work.mkdir(parents=True, exist_ok=True)
    rundir = work / "run"
    if rundir.exists():
        shutil.rmtree(rundir)
    rundir.mkdir()
    (rundir / "gurobi.env").write_text("Threads 1\n")
    log = open(work / "harness.log", "w")

    if args.classes:
        classes = os.path.abspath(args.classes)
    else:
        env = dict(os.environ, ORIGINAL_SRC=os.path.abspath(args.original_src), GUROBI_JAR=jar, JAVAC=javac,
                   OUT=str(work / "classes"))
        r = subprocess.run(["bash", str(HERE / "build.sh")], env=env, capture_output=True, text=True)
        sys.stderr.write(r.stderr)
        if r.returncode != 0:
            print("build failed", file=sys.stderr)
            return 1
        classes = r.stdout.strip().splitlines()[-1]
    runner = Runner(java, classes + os.pathsep + jar, os.path.dirname(jar), rundir, log)

    print("phase 1 (instances)", file=sys.stderr)
    p1 = case_spec.phase1()
    meta_case = {"id": "META", "fn": "META", "args": {}, "seed": 0}
    r1 = runner.run([meta_case] + p1)
    meta_out = r1.pop("META")
    bad = [c["id"] for c in p1 if "result" not in r1.get(c["id"], {})]
    if bad:
        print(f"phase-1 cases without a result: {bad}", file=sys.stderr)
        return 1
    inst = {c["id"]: r1[c["id"]]["result"] for c in p1}

    print("phase 2", file=sys.stderr)
    p2 = case_spec.phase2(inst)
    if args.filter:
        p2 = [c for c in p2 if re.search(args.filter, c["id"])]
    groups = case_spec.GROUPS if not args.only else args.only.split(",")
    p2 = [c for c in p2 if c["group"] in groups]
    r2 = runner.run(p2)

    if args.check_determinism:
        print("phase 2 again (determinism check)", file=sys.stderr)
        r2b = runner.run(p2)
        ndiff = 0
        for c in p2:
            a = strip_for_compare(assemble(c, r2[c["id"]]))
            b = strip_for_compare(assemble(c, r2b[c["id"]]))
            if a != b:
                ndiff += 1
                keys = sorted(k for k in set(a) | set(b) if a.get(k) != b.get(k))
                print(f"  NONDETERMINISTIC {c['id']}: {keys}", file=sys.stderr)
        print(f"  {ndiff} record(s) differ between runs", file=sys.stderr)

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    all_cases = [c for c in p1 if c["group"] in groups] + p2
    results = {**r1, **r2}
    summary = {}
    for grp in groups:
        recs = [assemble(c, results[c["id"]]) for c in all_cases if c["group"] == grp]
        if args.filter:  # merge into the existing file
            path = out_dir / f"{grp}.jsonl"
            old = []
            if path.exists():
                old = [json.loads(x) for x in path.read_text().splitlines() if x.strip()]
            merged = {r["id"]: r for r in old}
            merged.update({r["id"]: r for r in recs})
            old_ids = {o["id"] for o in old}
            order = [r["id"] for r in old] + [r["id"] for r in recs if r["id"] not in old_ids]
            recs = [merged[i] for i in order]
        with open(out_dir / f"{grp}.jsonl", "w", encoding="utf-8") as f:
            for r in recs:
                f.write(json.dumps(r, separators=(",", ":"), sort_keys=False) + "\n")
        summary[grp] = {
            "cases": len(recs),
            "results": sum("result" in r for r in recs),
            "exceptions": sum("exception" in r for r in recs),
            "hangs": sum(bool(r.get("hang")) for r in recs),
            "jvm_exits": sum("jvm_exit" in r for r in recs),
        }
    if not args.filter and not args.only:
        meta = {
            "generator": "tools/golden/record.py",
            "recorded": _dt.date.today().isoformat(),
            "original_repository": "https://github.com/arc-l/optimal-mrppg-journal",
            "original_commit": git_rev(Path(args.original_src)),
            "gurobi_version": meta_out["result"]["gurobi_version"],
            "gurobi_threads": meta_out["result"]["gurobi_threads_param"],
            "java_version": meta_out["result"]["java_version"],
            "java_vendor": meta_out["result"]["java_vendor"],
            "java_vm": meta_out["result"]["java_vm"],
            "os": meta_out["result"]["os"],
            "harness_restarts": runner.restarts,
            "groups": summary,
        }
        (out_dir / "META.json").write_text(json.dumps(meta, indent=2) + "\n")
    total = sum(p.stat().st_size for p in out_dir.glob("*.json*"))
    for grp, s in summary.items():
        print(f"  {grp:11s} {s}", file=sys.stderr)
    print(f"  total size {total / 1e6:.3f} MB in {out_dir}", file=sys.stderr)
    log.close()
    if not args.work:
        shutil.rmtree(work, ignore_errors=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
