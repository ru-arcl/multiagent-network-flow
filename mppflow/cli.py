"""Command line interface: ``python -m mppflow <command> ...`` (also the ``mppflow``
script).

Commands
--------
``solve``
    Plan paths for one instance, read from a file (``.dat`` written by the original or by
    :func:`mppflow.problem.write_dat`, the text format of :func:`mppflow.problem.write_txt`,
    or JSON ``{"graph": {"adj": [[1], [0, 2], [1]]}, "starts": [0], "goals": [2]}``, see
    :func:`mppflow.experiments.problem_from_json`) or generated:
    ``--grid 8x8 --obstacles 0.2 --robots 20 --seed 1`` (``create_grid_problem(8, 8, 0.2,
    20)``: the original's convention, the first number is the row count; ``--8-connected``
    for the 8-connected grid) or ``--puzzle 3`` (a random 3x3 N^2-puzzle). Options:
    ``--objective makespan|total_time|total_distance|max_distance``, ``--split k`` (the
    k-way split, ``k`` = 1, 2, 4, 8, 16, ...), ``--backend highs|gurobi|builtin``,
    ``--horizon search|single`` (total time / distance / max distance: Algorithm 1's horizon
    search, or one solve at the optimal makespan), ``--threads`` (Gurobi), ``--time-limit``
    (seconds; with ``--split k`` per piece, the pieces running one after another), ``--gap``
    (the MIP gap of total time, total distance and max distance; makespan ignores it),
    ``--compat fixed|original``,
    ``--json`` (machine-readable output), ``--no-paths`` (leave the paths out of the text
    output, and the ``"paths"`` key out of the JSON), ``--log`` (the solvers' messages),
    ``--save-problem FILE`` (``.dat``, ``.txt`` or ``.json``). Uses :func:`mppflow.planner.plan`.
    Exit status 0 when a plan was found, 1 otherwise.
``generate``
    Regenerate the benchmark data set (``Problem.main``) with a seed:
    :func:`mppflow.experiments.generate`.
``experiment``
    Run one of ``Main``'s drivers (:data:`mppflow.experiments.DRIVERS`; a key such as
    ``crowded-8x8``, the Java name or a figure key of :data:`~mppflow.experiments.FIGURES`)
    on a data folder, optionally reduced (``--instances``, ``--max-agents``,
    ``--obstacles``, ``--sections``, ``--time-limit``). Positional parameters follow the
    driver name (e.g. ``experiment crowded-8x8 10 30 60 1 false``). Prints the log and the
    result rows.
``case``
    ``Main.main``: one of the demo cases 0..6 (case 4 takes a file).
``figures``
    The paper figure/table -> driver mapping.
``view``
    Plan an instance as ``solve`` does (same instance options) and animate it in a
    matplotlib window (:mod:`mppflow.viewer`, needs matplotlib): ``view --puzzle 3``,
    ``view --grid 8x8 --robots 20 --split 2``. ``--split 1,2,4`` plans every listed k (each
    exactly as ``solve --split k`` with the same seed); ``k`` in the window cycles them.
    ``--gif FILE`` writes an animated GIF instead of opening a window.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from typing import Sequence

from . import __version__
from ._java import MathRandom

__all__ = ["main", "build_parser"]


def _params(args) -> dict | None:
    params = {}
    if getattr(args, "threads", None) is not None:
        if args.backend == "gurobi":
            params["Threads"] = args.threads
        else:
            print(f"note: --threads is ignored by the {args.backend} backend", file=sys.stderr)
    return params or None


def _grid(s: str) -> tuple[int, int]:
    try:
        a, b = s.lower().split("x")
        return int(a), int(b)
    except ValueError:
        raise argparse.ArgumentTypeError(f"expected ROWSxCOLS, e.g. 8x8, not {s!r}") from None


def _make_problem(args, rng: MathRandom):
    from . import problem as P
    from .experiments import load_problem

    given = [x for x in (args.input, args.grid, args.puzzle) if x is not None]
    if len(given) != 1:
        raise SystemExit(f"{args.command}: give exactly one of INPUT, --grid or --puzzle")
    if args.input is not None:
        return load_problem(args.input, args.compat)
    if args.puzzle is not None:
        return P.create_n2_puzzle(args.puzzle, rng, args.compat)
    a, b = args.grid
    if args.robots is None:
        raise SystemExit(f"{args.command}: --grid needs --robots")
    make = P.create_grid_problem_8connected if args.eight_connected else P.create_grid_problem
    return make(a, b, args.obstacles, args.robots, rng, args.compat)


def _save_problem(p, path: str, compat: str) -> None:
    from . import problem as P
    from .experiments import problem_to_json

    ext = os.path.splitext(path)[1].lower()
    if ext == ".json":
        with open(path, "w", encoding="utf-8") as f:
            json.dump(problem_to_json(p), f)
    elif ext == ".txt":
        P.write_txt(p, path, compat)
    else:
        P.write_dat(p, path, compat)


def _cmd_solve(args) -> int:
    from .experiments import problem_to_json
    from .planner import plan, print_paths

    rng = MathRandom(args.seed)
    p = _make_problem(args, rng)
    if args.save_problem:
        _save_problem(p, args.save_problem, args.compat)
    res = plan(p, args.objective, args.split, backend=args.backend, time_limit=args.time_limit, compat=args.compat,
               gap=args.gap, rng=rng, params=_params(args), horizon=args.horizon, debug=args.log,
               echo=args.log and not args.json)
    if args.json:
        out = {
            "problem": problem_to_json(p) if args.include_problem else
            {"vertices": p.graph.n, "robots": len(p.starts), "starts": p.starts, "goals": p.goals},
            "status": res.status, "objective": res.objective, "value": res.value,
            "metrics": res.metrics, "lower_bounds": res.lower_bounds, "split": res.split,
            "intermediate_goals": res.intermediate_goals, "time": res.time, "stats": res.stats,
            "compat": res.compat, "paths": res.paths, "extra": {k: v for k, v in res.extra.items()
                                                                  if isinstance(v, (int, float, str, list, dict))},
        }
        if args.no_paths:
            del out["paths"]
        if args.log:
            out["log"] = res.log
        json.dump(out, sys.stdout)
        sys.stdout.write("\n")
    else:
        print(f"instance: {p.graph.n} vertices, {len(p.starts)} robots")
        print(f"status: {res.status}  objective: {res.objective} = {res.value}")
        m = res.metrics
        lb = res.lower_bounds
        print(f"makespan {m['makespan']} (lb {lb['makespan']}), total time {m['total_time']} "
              f"(lb {lb['total_time']}), total distance {m['total_distance']} (lb {lb['total_distance']}), "
              f"max distance {m['max_distance']}")
        print(f"time {res.time:.3f} s, {res.stats['solves']} models solved")
        if res.extra.get("failures"):
            for f in res.extra["failures"]:
                print(f"failure: {f}")
        if res.paths is not None and not args.no_paths:
            sys.stdout.write(print_paths(p.graph, res.paths))
    return 0 if res.paths is not None else 1


def _splits(s: str) -> list[int]:
    try:
        ks = [int(x) for x in s.split(",") if x.strip()]
    except ValueError:
        raise argparse.ArgumentTypeError(f"expected k or a list such as 1,2,4, not {s!r}") from None
    if not ks:
        raise argparse.ArgumentTypeError("expected at least one k")
    return ks


def _cmd_view(args) -> int:
    from . import viewer
    from .planner import plan

    try:
        viewer._mpl()
    except ImportError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    plans, p = {}, None
    for k in args.split:
        rng = MathRandom(args.seed)  # every k sees the instance and stream of 'solve --split k'
        p = _make_problem(args, rng)
        res = plan(p, args.objective, k, backend=args.backend, time_limit=args.time_limit, compat=args.compat,
                   gap=args.gap, rng=rng, params=_params(args), horizon=args.horizon)
        label = f"{args.objective} k={k}"
        if res.paths is None:
            print(f"{label}: no plan ({res.status})", file=sys.stderr)
            continue
        print(f"{label}: {res.status}, makespan {res.makespan}, total time {res.total_time}, "
              f"total distance {res.total_distance}, max distance {res.max_distance} ({res.time:.2f} s)")
        plans[label] = res
    if not plans:
        return 1
    if args.title is not None:
        title = args.title
    elif args.puzzle is not None:
        title = f"{args.puzzle * args.puzzle}-puzzle, seed {args.seed}"
    elif args.grid is not None:
        title = (f"{args.grid[0]}x{args.grid[1]} grid, {int(round(args.obstacles * 100))}% obstacles, "
                 f"{args.robots} robots, seed {args.seed}")
    else:
        title = os.path.basename(args.input)
    if args.gif:
        viewer.save_gif(args.gif, p, plans=plans, title=title, fps=args.fps, all_plans=len(plans) > 1)
        print(f"wrote {args.gif}")
        return 0
    viewer.show(p, plans=plans, title=title)
    return 0


def _cmd_generate(args) -> int:
    from .experiments import generate
    from .problem import BENCHMARK_FAMILIES

    fams = args.families.split(",") if args.families else list(BENCHMARK_FAMILIES)
    out = args.out
    if args.compat == "original" and not out.endswith(("/", "\\")):
        out += "/"
    failed: list[str] = []
    files = generate(args.seed, out, fams, args.compat, write_text=not args.no_text, failed=failed)
    print(f"wrote {len(files)} files to {args.out}")
    if failed:
        print(f"error: could not write {len(failed)} files (first: {failed[0]})", file=sys.stderr)
        return 1
    return 0


def _parse_value(s: str):
    low = s.lower()
    if low in ("true", "false"):
        return low == "true"
    try:
        return int(s)
    except ValueError:
        return float(s)


def _experiments(args, base_path):
    from .experiments import Experiments, Scale

    scale = None
    if any(getattr(args, k, None) is not None for k in ("instances", "max_agents", "obstacle_pcts", "sections",
                                                         "time_limit")):
        scale = Scale(instances=args.instances, max_agents=args.max_agents,
                      obstacles=None if args.obstacle_pcts is None else tuple(int(x) for x in
                                                                               args.obstacle_pcts.split(",")),
                      sections=None if args.sections is None else tuple(args.sections.split(",")),
                      time_limit=args.time_limit)
    if args.compat == "original" and base_path is not None and not base_path.endswith(("/", "\\")):
        base_path += "/"
    return Experiments(base_path, compat=args.compat, backend=args.backend, params=_params(args),
                       rng=MathRandom(args.seed), echo=not args.quiet, results_dir=args.results, scale=scale)


def _cmd_experiment(args) -> int:
    from .experiments import DRIVERS, FIGURES, _driver_key

    sections = None
    name = args.driver
    if name in FIGURES:
        fig = FIGURES[name]
        if fig.driver is None:
            raise SystemExit(f"{name}: no driver in the original code ({fig.notes})")
        name = fig.driver
        sections = fig.sections
    try:
        key = _driver_key(name)
    except KeyError as e:
        raise SystemExit(str(e)) from None
    if args.sections is None and sections is not None:
        args.sections = ",".join(sections)
    ex = _experiments(args, args.data)
    params = [_parse_value(v) for v in args.params]
    if len(params) > len(DRIVERS[key]["params"]):
        raise SystemExit(f"{key} takes at most {len(DRIVERS[key]['params'])} parameters: "
                         f"{', '.join(DRIVERS[key]['params']) or 'none'}")
    try:
        ex.run_driver(key, *params)
    finally:
        if not args.quiet and ex.results:
            print("\n--- results ---")
        if ex.results:
            sys.stdout.write(ex.results)
    return 0


def _cmd_case(args) -> int:
    ex = _experiments(args, args.data)
    argv = [str(args.case)] + ([args.file] if args.file else [])
    ex.main(argv)
    return 0


def _cmd_figures(args) -> int:
    from dataclasses import asdict

    from .experiments import FIGURES, figures_text

    if args.json:
        json.dump([asdict(f) for f in FIGURES.values()], sys.stdout, indent=1)
        sys.stdout.write("\n")
    else:
        sys.stdout.write(figures_text())
    return 0


def _common(p: argparse.ArgumentParser) -> None:
    p.add_argument("--backend", default="highs", choices=["highs", "gurobi", "builtin"],
                   help="LP/MILP backend (default highs)")
    p.add_argument("--threads", type=int, default=None, help="Gurobi Threads parameter (1 = deterministic)")
    p.add_argument("--compat", default="fixed", choices=["fixed", "original"],
                   help="fixed (default, corrected) or original (reproduce the Java code)")
    p.add_argument("--seed", type=int, default=1, help="seed of the Math.random() stream (default 1)")


def _scale_opts(p: argparse.ArgumentParser) -> None:
    p.add_argument("--instances", type=int, default=None, help="instances per row (default: the original's)")
    p.add_argument("--max-agents", type=int, default=None, help="largest robot count of every sweep")
    p.add_argument("--obstacles", dest="obstacle_pcts", default=None,
                   help="comma-separated obstacle percentages (24x18 drivers)")
    p.add_argument("--sections", default=None, help="comma-separated sections (see 'figures' / DRIVERS)")
    p.add_argument("--time-limit", type=float, default=None, help="replaces every time limit (seconds)")
    p.add_argument("--results", default=None, help="folder of the result files (default: the data folder)")
    p.add_argument("--quiet", action="store_true", help="do not print the log")


_GAP_HELP = "MIP gap of total time / total distance / max distance (default 0; ignored for --objective makespan)"
_HORIZON_HELP = ("total time / total distance / max distance: 'search' (default in fixed mode; Algorithm 1, "
                 "optimal over all horizons) or 'single' (one solve at the optimal makespan; status 'feasible' "
                 "unless the plan meets the lower bound)")
_TIME_LIMIT_HELP = ("seconds, > 0 (default: no limit; 0, negative or nan is an error in fixed mode); with "
                    "--split k it applies to each piece, and the pieces run one after another (wall time up to "
                    "k times the limit)")


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(prog="mppflow", description="Optimal multi-robot path planning on graphs "
                                 "(network flow / ILP), a Python port of our original Java code.")
    ap.add_argument("--version", action="version", version=f"%(prog)s {__version__}")
    sub = ap.add_subparsers(dest="command")

    s = sub.add_parser("solve", help="plan paths for one instance")
    s.add_argument("input", nargs="?", default=None, help="problem file (.dat, .txt or .json)")
    s.add_argument("--grid", type=_grid, default=None, help="generate a grid problem, ROWSxCOLS (e.g. 8x8)")
    s.add_argument("--obstacles", type=float, default=0.0, help="obstacle fraction of --grid (default 0)")
    s.add_argument("--robots", type=int, default=None, help="number of robots of --grid")
    s.add_argument("--8-connected", dest="eight_connected", action="store_true", help="8-connected --grid")
    s.add_argument("--puzzle", type=int, default=None, help="generate a random N^2-puzzle of side N")
    s.add_argument("--objective", default="makespan",
                   choices=["makespan", "total_time", "total_distance", "max_distance"])
    s.add_argument("--split", type=int, default=1, help="k-way split (1, 2, 4, 8, ..., at most 1024)")
    s.add_argument("--horizon", default=None, choices=["search", "single"], help=_HORIZON_HELP)
    s.add_argument("--time-limit", type=float, default=None, help=_TIME_LIMIT_HELP)
    s.add_argument("--gap", type=float, default=0.0, help=_GAP_HELP)
    s.add_argument("--json", action="store_true", help="print the result as JSON")
    s.add_argument("--include-problem", action="store_true",
                   help="--json: include the whole problem (the output can be solved again)")
    s.add_argument("--log", action="store_true", help="print the solvers' messages")
    s.add_argument("--no-paths", action="store_true", help="do not print the paths (with --json: no \"paths\" key)")
    s.add_argument("--save-problem", default=None, help="write the instance (.dat, .txt or .json)")
    _common(s)
    s.set_defaults(func=_cmd_solve)

    g = sub.add_parser("generate", help="regenerate the benchmark data set (Problem.main)")
    g.add_argument("--out", required=True, help="output folder")
    g.add_argument("--seed", type=int, default=1, help="seed of the Math.random() stream (default 1)")
    g.add_argument("--families", default=None,
                   help="comma-separated: n2puzzle, crowded, 32x32, 32x32-8c, 24x18, crowded-8c (default: all "
                        "five of Problem.main, in its order). The families share one random stream in the "
                        "order given, so a family's instances depend on the families before it: only the "
                        "default list reproduces the files of a full run with the same seed")
    g.add_argument("--compat", default="fixed", choices=["fixed", "original"])
    g.add_argument("--no-text", action="store_true", help="write only the .dat files")
    g.set_defaults(func=_cmd_generate)

    e = sub.add_parser("experiment", help="run one of Main's experiment drivers")
    e.add_argument("driver", help="driver key, Java name or figure key ('figures' lists them)")
    e.add_argument("params", nargs="*", help="the driver's positional parameters")
    e.add_argument("--data", required=True, help="data folder (Main.basePath; see 'generate')")
    _common(e)
    _scale_opts(e)
    e.set_defaults(func=_cmd_experiment)

    c = sub.add_parser("case", help="Main.main: demo case 0..6")
    c.add_argument("case", help="the case number (Main's argv[0])")
    c.add_argument("file", nargs="?", default=None, help="case 4: the problem file")
    c.add_argument("--data", default=None, help="data folder (case 0)")
    _common(c)
    _scale_opts(c)
    c.set_defaults(func=_cmd_case)

    v = sub.add_parser("view", help="plan an instance and animate it (needs matplotlib)")
    v.add_argument("input", nargs="?", default=None, help="problem file (.dat, .txt or .json)")
    v.add_argument("--grid", type=_grid, default=None, help="generate a grid problem, ROWSxCOLS (e.g. 8x8)")
    v.add_argument("--obstacles", type=float, default=0.0, help="obstacle fraction of --grid (default 0)")
    v.add_argument("--robots", type=int, default=None, help="number of robots of --grid")
    v.add_argument("--8-connected", dest="eight_connected", action="store_true", help="8-connected --grid")
    v.add_argument("--puzzle", type=int, default=None, help="generate a random N^2-puzzle of side N")
    v.add_argument("--objective", default="makespan",
                   choices=["makespan", "total_time", "total_distance", "max_distance"])
    v.add_argument("--split", type=_splits, default=[1], help="k-way split, or a list (1,2,4) to cycle with 'k'")
    v.add_argument("--horizon", default=None, choices=["search", "single"], help=_HORIZON_HELP)
    v.add_argument("--time-limit", type=float, default=None, help=_TIME_LIMIT_HELP + "; per plan of each k")
    v.add_argument("--gap", type=float, default=0.0, help=_GAP_HELP)
    v.add_argument("--title", default=None, help="window / GIF title")
    v.add_argument("--gif", default=None, help="write an animated GIF to this file instead of opening a window")
    v.add_argument("--fps", type=int, default=12, help="--gif frames per second (default 12)")
    _common(v)
    v.set_defaults(func=_cmd_view)

    f = sub.add_parser("figures", help="paper figure/table -> driver mapping")
    f.add_argument("--json", action="store_true")
    f.set_defaults(func=_cmd_figures)
    return ap


def main(argv: Sequence[str] | None = None) -> int:
    ap = build_parser()
    args = ap.parse_args(argv)
    if getattr(args, "func", None) is None:
        ap.print_help()
        return 2
    from .errors import MPPError

    try:
        return args.func(args)
    except (MPPError, FileNotFoundError, ValueError) as e:
        name = getattr(e, "java_class", type(e).__name__)
        print(f"error: {name}: {e}", file=sys.stderr)
        return 1
    except ImportError as e:  # a missing optional dependency (gurobipy, matplotlib), as in 'view'
        print(f"error: {e}", file=sys.stderr)
        return 1
    except Exception as e:
        # A Gurobi license problem (pip's size-limited license on a large model, an expired
        # license) is an error line too, not a traceback. Matched by module so that gurobipy
        # is not imported here.
        if type(e).__name__ != "GurobiError" or not type(e).__module__.startswith("gurobipy"):
            raise
        print(f"error: GurobiError: {e} (a Gurobi license problem? try --backend highs)", file=sys.stderr)
        return 1


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
