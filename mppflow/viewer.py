"""A small matplotlib viewer for plans (optional dependency: ``pip install ".[viewer]"``
from a checkout of the repository).

Importing this module does not import matplotlib; it is loaded the first time a figure is
made.

* :func:`show` opens a window: the graph (grid coordinates when every vertex has them,
  else a deterministic spring layout), the robots as labeled discs, and a timeline strip
  under the graph. Following the paper's convention (TOR Fig. 1), start locations are
  shaded discs and goal locations unshaded ones; the moving robots are the shaded discs
  with the robot's number (``1..n``, robot ``i`` of the plan is labeled ``i + 1``).

  The viewer follows the mouse: moving it horizontally over the timeline scrubs the time
  continuously (robots interpolate along the edges they traverse); hovering a robot (or its
  goal marker) highlights its whole path and shows its start, goal, arrival time and
  distance. Keys: ``space`` play / pause, ``left`` / ``right`` step to the previous / next
  integer time, ``home`` / ``end`` jump, ``k`` (``K``) cycles the precomputed plans (e.g.
  the k-way split plans for k = 1, 2, 4, ...), ``g`` toggles the start / goal markers,
  ``up`` / ``down`` change the speed, ``q`` closes the window.
* :func:`render_frames` and :func:`save_gif` draw the same picture off screen (Agg) for
  documentation, the GIF through matplotlib's ``PillowWriter`` (needs Pillow).

Every function takes the instance as a :class:`~mppflow.problem.Problem`, a
``(graph, starts, goals)`` tuple, or a :class:`~mppflow.graph.Graph` followed by ``starts``
and ``goals``. A plan is ``paths[i][t]`` (all rows of the same length), a
:class:`~mppflow.planner.PlanResult`, or a mapping with a ``"paths"`` entry. ``plans`` is a
mapping ``label -> plan`` or a sequence of plans.

Example::

    import mppflow, mppflow.viewer
    p = mppflow.create_n2_puzzle(3, mppflow.MathRandom(1))
    plans = {f"k={k}": mppflow.plan(p, "makespan", k) for k in (1, 2)}
    mppflow.viewer.show(p, plans=plans, title="9-puzzle")

Command line: ``python -m mppflow view --puzzle 3`` or
``python -m mppflow view --grid 8x8 --robots 20 --split 1,2`` (``--gif FILE`` writes a GIF
instead of opening a window).
"""
from __future__ import annotations

import math
import time
from collections.abc import Mapping

__all__ = ["Viewer", "show", "make_viewer", "render_frames", "save_gif", "layout", "spring_layout", "plan_from"]

# Paper-like palette: cyan shaded discs (TOR Fig. 1), unshaded goals.
COLORS = {
    "background": "#ffffff",
    "edge": "#b9c0c7",
    "vertex": "#8a939c",
    "robot": "#8fd6df",
    "robot_edge": "#1f5560",
    "arrived": "#c9ecf0",
    "start": "#8fd6df",
    "goal_face": "#ffffff",
    "goal_edge": "#7a8691",
    "label": "#10262b",
    "goal_label": "#7a8691",
    "highlight": "#e8762d",
    "highlight_face": "#f6b17f",
    "bars": "#cfd6dc",
    "cursor": "#1f5560",
    "boundary": "#7a8691",
    "text": "#24292f",
    "muted": "#5f6b76",
    "error": "#c62828",
}

DISC = 0.33          # disc radius, in units of the typical edge length
SPEED = 2.0          # time steps per second when playing
TICK_MS = 40         # animation timer interval


def _mpl():
    try:
        import matplotlib  # noqa: F401
    except ImportError as e:  # pragma: no cover - depends on the environment
        raise ImportError("mppflow.viewer needs matplotlib: pip install matplotlib, or "
                          "pip install \".[viewer]\" from a checkout of the repository") from e
    import matplotlib
    return matplotlib


# ----------------------------------------------------------------------------- inputs
def _instance(problem_or_graph, starts=None, goals=None):
    if isinstance(problem_or_graph, tuple):
        graph, s, g = problem_or_graph
    elif hasattr(problem_or_graph, "graph") and hasattr(problem_or_graph, "starts"):
        graph, s, g = problem_or_graph.graph, problem_or_graph.starts, problem_or_graph.goals
    else:
        graph, s, g = problem_or_graph, None, None
    s = starts if starts is not None else s
    g = goals if goals is not None else g
    if graph is None or s is None or g is None:
        raise ValueError("give a Problem, a (graph, starts, goals) tuple, or a graph with starts and goals")
    if len(s) != len(g):
        raise ValueError(f"{len(s)} starts but {len(g)} goals")
    return graph, list(s), list(g)


def plan_from(obj, label=None):
    """``(label, paths, info)`` for a plan given as paths, a ``PlanResult`` or a mapping
    with ``"paths"``; ``info`` keeps ``intermediate_goals`` / ``status`` when known."""
    info = {}
    if hasattr(obj, "paths") and hasattr(obj, "objective"):
        paths = obj.paths
        info = {"status": obj.status, "intermediate_goals": obj.intermediate_goals,
                "objective": obj.objective, "split": obj.split}
        label = label or f"{obj.objective} k={obj.split}"
    elif isinstance(obj, Mapping):
        paths = obj.get("paths")
        info = {k: obj[k] for k in ("status", "intermediate_goals", "note", "backend") if k in obj}
    else:
        paths = obj
    if paths is None:
        raise ValueError(f"plan {label!r} has no paths")
    paths = [list(map(int, row)) for row in paths]
    return label, paths, info


def _plans(paths, plans, n_robots):
    out = []
    if plans is not None:
        items = plans.items() if isinstance(plans, Mapping) else [(None, p) for p in plans]
        for i, (label, obj) in enumerate(items):
            lab, pp, info = plan_from(obj, label)
            out.append((lab if lab is not None else f"plan {i + 1}", pp, info))
    start = 0
    if paths is not None:
        lab, pp, info = plan_from(paths, None)
        match = [i for i, q in enumerate(out) if q[1] == pp]
        if match:
            start = match[0]
        else:
            out.insert(0, (lab or "plan", pp, info))
    for lab, pp, _ in out:
        if len(pp) != n_robots:
            raise ValueError(f"plan {lab!r} has {len(pp)} paths for {n_robots} robots")
        if pp and len({len(r) for r in pp}) != 1:
            raise ValueError(f"plan {lab!r}: paths of different lengths")
        if pp and not pp[0]:
            raise ValueError(f"plan {lab!r}: empty paths")
    return out, start


# ----------------------------------------------------------------------------- layout
def _edges(graph, verts):
    seen = set()
    vs = set(verts)
    out = []
    for u in verts:
        for v in graph.adj.get(u, ()):
            if v == u or v not in vs:
                continue
            key = (min(u, v), max(u, v))
            if key not in seen:
                seen.add(key)
                out.append(key)
    return out


def spring_layout(n, edges, iterations=300):
    """A deterministic Fruchterman-Reingold layout of vertices ``0..n-1`` (no random
    draws: golden-angle spiral start, fixed cooling). Scaled so that the median edge has
    length 1. Returns a list of ``(x, y)``."""
    import numpy as np

    if n == 0:
        return []
    if n == 1:
        return [(0.0, 0.0)]
    i = np.arange(n, dtype=float)
    r = np.sqrt(i + 0.5)
    th = i * 2.399963229728653
    pos = np.column_stack([r * np.cos(th), r * np.sin(th)])
    pos /= max(np.abs(pos).max(), 1e-9)
    k = math.sqrt(4.0 / n)
    e = np.array(edges, dtype=int).reshape(-1, 2)
    temp = 0.1
    for it in range(iterations):
        d = pos[:, None, :] - pos[None, :, :]
        dist = np.sqrt((d ** 2).sum(-1))
        np.fill_diagonal(dist, 1.0)
        dist = np.maximum(dist, 1e-3)
        disp = ((d / dist[..., None]) * (k * k / dist)[..., None]).sum(1)
        if len(e):
            de = pos[e[:, 0]] - pos[e[:, 1]]
            le = np.maximum(np.sqrt((de ** 2).sum(-1)), 1e-3)
            f = (de / le[:, None]) * (le * le / k)[:, None]
            np.add.at(disp, e[:, 0], -f)
            np.add.at(disp, e[:, 1], f)
        ln = np.maximum(np.sqrt((disp ** 2).sum(-1)), 1e-9)
        pos += disp / ln[:, None] * np.minimum(ln, temp)[:, None]
        temp = 0.1 * (1.0 - (it + 1) / iterations) + 1e-3
    if len(e):
        de = pos[e[:, 0]] - pos[e[:, 1]]
        scale = float(np.median(np.sqrt((de ** 2).sum(-1))))
    else:
        scale = k
    pos = (pos - pos.mean(0)) / max(scale, 1e-9)
    return [(float(x), float(y)) for x, y in pos]


def layout(graph):
    """``(verts, pos, edges, grid)``: the vertex ids (ascending), ``pos[v] = (x, y)`` (grid
    coordinates ``x`` = column, ``y`` = row when every vertex has them, else
    :func:`spring_layout`), the undirected edges and whether grid coordinates were used."""
    verts = sorted(graph.adj)
    edges = _edges(graph, verts)
    pts = {}
    for v in verts:
        p = graph.point(v) if hasattr(graph, "point") else getattr(graph, "xy", {}).get(v)
        if p is None:
            pts = None
            break
        pts[v] = (float(p[0]), float(p[1]))
    if pts is not None and len(set(pts.values())) == len(pts):
        return verts, pts, edges, True
    index = {v: i for i, v in enumerate(verts)}
    sl = spring_layout(len(verts), [(index[u], index[v]) for u, v in edges])
    return verts, {v: sl[index[v]] for v in verts}, edges, False


def _arrival(path, goal):
    if not path or path[-1] != goal:
        return None
    t = len(path) - 1
    while t > 0 and path[t - 1] == goal:
        t -= 1
    return t


def _boundaries(paths, info):
    """The times at which a split plan reaches its intermediate configurations."""
    mids = info.get("intermediate_goals") or []
    out, t0 = [], 0
    if not paths:
        return out
    T = len(paths[0]) - 1
    for cfg in mids:
        if len(cfg) != len(paths):
            return []
        for t in range(t0 + 1, T):
            if all(paths[i][t] == cfg[i] for i in range(len(paths))):
                out.append(t)
                t0 = t
                break
    return out


# ----------------------------------------------------------------------------- viewer
class Viewer:
    """The figure and its interaction state. Build it with :func:`show` (window) or
    :func:`render_frames` / :func:`save_gif` (off screen); ``Viewer(fig, ...)`` draws into
    an existing, empty figure.

    State: ``time`` (float, ``0..T``), ``plan_index``, ``highlight`` (robot index or
    ``None``), ``playing``. Methods: :meth:`set_time`, :meth:`step`, :meth:`set_plan`,
    :meth:`cycle_plan`, :meth:`set_highlight`, :meth:`toggle_play`, :meth:`advance`,
    :meth:`positions`. Mouse and key events are connected to the figure's canvas.
    """

    def __init__(self, fig, graph, starts, goals, plans, *, start_plan=0, title=None, colors=None,
                 interactive=True):
        import numpy as np

        self._np = np
        self.fig = fig
        self.graph, self.starts, self.goals = graph, starts, goals
        self.plans = plans
        self.title = title
        self.c = dict(COLORS, **(colors or {}))
        self.n = len(starts)
        self.verts, self.pos, self.edges, self.is_grid = layout(graph)
        missing = [v for v in list(starts) + list(goals) if v not in self.pos]
        for _, pp, _ in plans:
            missing += [v for row in pp for v in row if v not in self.pos]
        if missing:
            raise ValueError(f"vertex {missing[0]} is not in the graph")
        self.time = 0.0
        self.plan_index = start_plan
        self.highlight = None
        self.playing = False
        self.speed = SPEED
        self.show_markers = True
        self._timer = None
        self._cids = []
        self.live = interactive
        self._radius()
        self._build()
        self.set_plan(start_plan)
        if interactive:
            cv = fig.canvas
            self._cids = [cv.mpl_connect("motion_notify_event", self.on_motion),
                          cv.mpl_connect("button_press_event", self.on_motion),
                          cv.mpl_connect("key_press_event", self.on_key),
                          cv.mpl_connect("resize_event", lambda e: self._fit_fonts())]

    # -- geometry
    def _radius(self):
        np = self._np
        if self.edges:
            le = [math.dist(self.pos[u], self.pos[v]) for u, v in self.edges]
            unit = float(np.median(le))
        else:
            unit = 1.0
        r = DISC * unit
        pts = np.array([self.pos[v] for v in self.verts])
        if len(pts) > 1 and len(pts) <= 3000:
            d = np.sqrt(((pts[:, None, :] - pts[None, :, :]) ** 2).sum(-1))
            np.fill_diagonal(d, np.inf)
            r = min(r, 0.45 * float(d.min()))
        self.r = r if r > 0 else DISC

    @property
    def paths(self):
        return self.plans[self.plan_index][1]

    @property
    def label(self):
        return self.plans[self.plan_index][0]

    @property
    def T(self):
        return len(self.paths[0]) - 1 if self.paths else 0

    def positions(self, t=None):
        """Robot positions (an ``n x 2`` array) at the continuous time ``t`` (default the
        current time): linear interpolation between ``paths[i][floor(t)]`` and the next
        vertex."""
        np = self._np
        t = self.time if t is None else min(max(float(t), 0.0), self.T)
        if not self.paths:
            return np.array([self.pos[s] for s in self.starts]).reshape(-1, 2)
        i = min(int(math.floor(t)), max(self.T - 1, 0))
        f = t - i if self.T > 0 else 0.0
        a = np.array([self.pos[p[i]] for p in self.paths])
        b = np.array([self.pos[p[min(i + 1, self.T)]] for p in self.paths])
        return a + (b - a) * f

    # -- drawing
    def _circles(self, ax, offsets, **kw):
        from matplotlib.collections import EllipseCollection

        np = self._np
        m = len(offsets)
        d = np.full(m, 2 * self.r)
        try:
            col = EllipseCollection(d, d, np.zeros(m), units="xy", offsets=offsets,
                                    offset_transform=ax.transData, **kw)
        except TypeError:  # matplotlib < 3.6
            col = EllipseCollection(d, d, np.zeros(m), units="xy", offsets=offsets,
                                    transOffset=ax.transData, **kw)
        ax.add_collection(col)
        return col

    def _build(self):
        from matplotlib.collections import LineCollection

        np = self._np
        fig, c = self.fig, self.c
        fig.patch.set_facecolor(c["background"])
        xs = [p[0] for p in self.pos.values()] or [0.0]
        ys = [p[1] for p in self.pos.values()] or [0.0]
        pad = 1.6 * self.r
        x0, x1, y0, y1 = min(xs) - pad, max(xs) + pad, min(ys) - pad, max(ys) + pad
        aspect = (x1 - x0) / max(y1 - y0, 1e-9)
        W = fig.get_figwidth()
        ha = min(max(W * 0.94 / aspect, 2.4), 8.0)
        top, bottom = 0.75, 1.15
        H = ha + top + bottom
        fig.set_size_inches(W, H, forward=True)
        self.ax = ax = fig.add_axes([0.03, bottom / H, 0.94, ha / H])
        ax.set_aspect("equal", adjustable="datalim")
        ax.set_xlim(x0, x1)
        ax.set_ylim(y1, y0)  # rows grow downwards, as in the paper's figures
        ax.axis("off")
        self.tl = tl = fig.add_axes([0.07, 0.42 / H, 0.86, 0.36 / H])
        tl.set_facecolor(c["background"])
        tl.set_yticks([])
        tl.tick_params(axis="x", labelsize=8, colors=c["muted"], length=2)
        for side in ("left", "right", "top"):
            tl.spines[side].set_visible(False)
        tl.spines["bottom"].set_color(c["edge"])
        tl.set_ylim(0, 1.05)

        segs = [(self.pos[u], self.pos[v]) for u, v in self.edges]
        ax.add_collection(LineCollection(segs, colors=c["edge"], linewidths=1.2, zorder=1))
        vp = np.array([self.pos[v] for v in self.verts]).reshape(-1, 2)
        ax.scatter(vp[:, 0], vp[:, 1], s=6, color=c["vertex"], zorder=2, linewidths=0)

        sp = np.array([self.pos[v] for v in self.starts]).reshape(-1, 2)
        gp = np.array([self.pos[v] for v in self.goals]).reshape(-1, 2)
        self.start_col = self._circles(ax, sp, facecolors=c["start"], edgecolors="none", alpha=0.28, zorder=3)
        self.goal_col = self._circles(ax, gp, facecolors=c["goal_face"], edgecolors=c["goal_edge"],
                                      linewidths=1.0, linestyles="--", zorder=3)
        self.goal_labels = [ax.text(x, y, str(i + 1), ha="center", va="center", color=c["goal_label"], zorder=4)
                            for i, (x, y) in enumerate(gp)]
        (self.hl_line,) = ax.plot([], [], color=c["highlight"], lw=3, alpha=0.75, zorder=5,
                                  solid_capstyle="round", solid_joinstyle="round")
        self.hl_goal = self._circles(ax, np.zeros((1, 2)), facecolors="none", edgecolors=c["highlight"],
                                     linewidths=2.0, zorder=6)
        self.hl_goal.set_visible(False)
        self.robot_col = self._circles(ax, sp, facecolors=c["robot"], edgecolors=c["robot_edge"],
                                       linewidths=1.0, zorder=7)
        self.labels = [ax.text(x, y, str(i + 1), ha="center", va="center", color=c["label"], zorder=8,
                               fontweight="bold") for i, (x, y) in enumerate(sp)]

        self.title_text = fig.text(0.5, 1 - 0.2 / H, self.title or "", ha="center", va="center",
                                   fontsize=12, color=c["text"])
        self.status_text = fig.text(0.5, 1 - 0.5 / H, "", ha="center", va="center", fontsize=8.5, linespacing=1.4,
                                    color=c["muted"])
        self.info_text = fig.text(0.07, 0.95 / H, "", ha="left", va="center", fontsize=9, color=c["text"])
        self.time_text = fig.text(0.93, 0.95 / H, "", ha="right", va="center", fontsize=9, color=c["text"],
                                  family="monospace")
        self.help_text = fig.text(0.5, 0.12 / H, "", ha="center", va="center", fontsize=7.5, color=c["muted"])
        self.cursor = tl.axvline(0, color=c["cursor"], lw=2, zorder=5)
        self.hl_arrival = tl.axvline(0, color=c["highlight"], lw=1.5, ls="--", zorder=4)
        self.hl_arrival.set_visible(False)
        self._tl_artists = []
        self._fit_fonts()

    def _fit_fonts(self):
        """Label sizes that fit the discs at the current axes size."""
        try:
            self.ax.apply_aspect()
        except Exception:  # pragma: no cover
            pass
        tr = self.ax.transData
        px = abs(tr.transform((self.r, 0))[0] - tr.transform((0, 0))[0])
        pts = px * 72.0 / self.fig.dpi
        digits = len(str(max(self.n, 1)))
        size = pts * (0.9 if digits == 1 else 0.78 if digits == 2 else 0.58)
        vis = size >= 3.5
        for t in self.labels:
            t.set_fontsize(max(size, 1.0))
            t.set_visible(vis)
        for t in self.goal_labels:
            t.set_fontsize(max(size * 0.85, 1.0))
            t.set_visible(vis and self.show_markers)

    def _timeline(self):
        tl, c = self.tl, self.c
        for a in self._tl_artists:
            a.remove()
        self._tl_artists = []
        T = self.T
        tl.set_xlim(0, max(T, 1))
        from matplotlib.ticker import MaxNLocator

        tl.xaxis.set_major_locator(MaxNLocator(integer=True, nbins=12))
        if T > 0 and self.n:
            moving = [sum(1 for p in self.paths if p[t] != p[t + 1]) / self.n for t in range(T)]
            bars = tl.bar(range(T), moving, width=1.0, align="edge", color=c["bars"], edgecolor=c["background"],
                          linewidth=0.5, zorder=1)
            self._tl_artists.extend(bars.patches)
        for b in self._boundaries:
            self._tl_artists.append(tl.axvline(b, color=c["boundary"], lw=1, ls=":", zorder=3))

    def _status(self):
        from .planner import path_set_error, plan_metrics

        paths = self.paths
        m = plan_metrics(paths, self.goals) if paths else {"makespan": 0, "total_time": 0,
                                                            "total_distance": 0, "max_distance": 0}
        err = path_set_error(paths, self.graph, self.starts, self.goals) if paths else None
        self.metrics, self.error = m, err
        tt = "-" if m["total_time"] is None else m["total_time"]
        parts = []
        if len(self.plans) > 1:
            parts.append(f"{self.label} ({self.plan_index + 1}/{len(self.plans)})")
        elif self.label != "plan":
            parts.append(self.label)
        parts.append(f"{self.n} robots, {len(self.verts)} vertices")
        metrics = (f"makespan {m['makespan']}  ·  total time {tt}  ·  total distance {m['total_distance']}  ·  "
                   f"max distance {m['max_distance']}")
        self.status_text.set_text("  |  ".join(parts) + "\n" + metrics)
        self.status_text.set_color(self.c["muted"])
        if err:
            msg = f"INVALID PLAN: {err}"
            self.status_text.set_text(msg if len(msg) <= 100 else msg[:97] + "...")
            self.status_text.set_color(self.c["error"])
        keys = "move over the timeline to scrub  ·  hover a robot  ·  space play/pause  ·  ←/→ step"
        if len(self.plans) > 1:
            keys += "  ·  k next plan"
        self.help_text.set_text(keys)

    def _arrivals(self):
        return [_arrival(p, g) for p, g in zip(self.paths, self.goals)] if self.paths else [0] * self.n

    def _distances(self):
        return [sum(1 for i in range(len(p) - 1) if p[i] != p[i + 1]) for p in self.paths] if self.paths \
            else [0] * self.n

    def _draw_idle(self):
        cv = getattr(self.fig, "canvas", None)
        if cv is not None and self.live:  # off-screen viewers draw when a frame is taken
            cv.draw_idle()

    def _update(self):
        np = self._np
        c = self.c
        P = self.positions()
        self.robot_col.set_offsets(P)
        for lab, (x, y) in zip(self.labels, P):
            lab.set_position((x, y))
        faces = []
        arr = self._arr
        for i in range(self.n):
            if i == self.highlight:
                faces.append(c["highlight_face"])
            elif arr[i] is not None and self.time >= arr[i]:
                faces.append(c["arrived"])
            else:
                faces.append(c["robot"])
        self.robot_col.set_facecolors(faces)
        edges = [c["highlight"] if i == self.highlight else c["robot_edge"] for i in range(self.n)]
        self.robot_col.set_edgecolors(edges)
        lw = [2.0 if i == self.highlight else 1.0 for i in range(self.n)]
        self.robot_col.set_linewidths(lw)
        self.cursor.set_xdata([self.time, self.time])
        self.time_text.set_text(f"t = {self.time:5.2f} / {self.T}")
        h = self.highlight
        if h is None:
            self.hl_line.set_data([], [])
            self.hl_goal.set_visible(False)
            self.hl_arrival.set_visible(False)
            self.info_text.set_text("")
        else:
            path = self.paths[h] if self.paths else [self.starts[h]]
            pts = [self.pos[path[0]]]
            for v in path[1:]:
                if self.pos[v] != pts[-1]:
                    pts.append(self.pos[v])
            pts = np.array(pts)
            self.hl_line.set_data(pts[:, 0], pts[:, 1])
            self.hl_goal.set_offsets(np.array([self.pos[self.goals[h]]]))
            self.hl_goal.set_visible(True)
            a = arr[h]
            if a is not None:
                self.hl_arrival.set_xdata([a, a])
            self.hl_arrival.set_visible(a is not None)
            now = path[min(int(math.floor(self.time)), len(path) - 1)]
            self.info_text.set_text(
                f"robot {h + 1}: v{self.starts[h]} → v{self.goals[h]}, "
                f"arrival {'never' if a is None else a}, distance {self._dist[h]}, at v{now}")
        self._draw_idle()

    # -- state changes
    def set_plan(self, index):
        self.plan_index = index % len(self.plans) if self.plans else 0
        self._boundaries = _boundaries(self.paths, self.plans[self.plan_index][2]) if self.plans else []
        self._arr = self._arrivals()
        self._dist = self._distances()
        self.time = min(self.time, float(self.T))
        self._timeline()
        self._status()
        self._update()

    def cycle_plan(self, step=1):
        self.set_plan(self.plan_index + step)

    def set_time(self, t):
        self.time = min(max(float(t), 0.0), float(self.T))
        self._update()

    def step(self, direction):
        """Go to the next (``direction > 0``) or previous integer time."""
        if direction > 0:
            t = math.floor(self.time + 1e-9) + 1
        else:
            t = math.ceil(self.time - 1e-9) - 1
        self.set_time(t)

    def set_highlight(self, robot):
        if robot != self.highlight:
            self.highlight = robot
            self._update()

    def robot_at(self, x, y):
        """The robot whose disc (at the current time) or goal marker contains ``(x, y)``."""
        np = self._np
        if x is None or y is None or not self.n:
            return None
        P = self.positions()
        d = np.hypot(P[:, 0] - x, P[:, 1] - y)
        i = int(d.argmin())
        if d[i] <= self.r * 1.05:
            return i
        if self.show_markers:
            G = np.array([self.pos[g] for g in self.goals])
            d = np.hypot(G[:, 0] - x, G[:, 1] - y)
            i = int(d.argmin())
            if d[i] <= self.r * 1.05:
                return i
        return None

    def toggle_markers(self):
        self.show_markers = not self.show_markers
        for a in (self.start_col, self.goal_col):
            a.set_visible(self.show_markers)
        self._fit_fonts()
        self._draw_idle()

    def toggle_play(self):
        self.playing = not self.playing
        if self.playing:
            if self.time >= self.T:
                self.set_time(0)
            if self._timer is None:
                cv = getattr(self.fig, "canvas", None)
                if cv is not None and hasattr(cv, "new_timer"):
                    self._timer = cv.new_timer(interval=TICK_MS)
                    self._timer.add_callback(self._tick)
            if self._timer is not None:
                self._last_tick = time.perf_counter()
                self._timer.start()
        elif self._timer is not None:
            self._timer.stop()

    def advance(self, dt):
        """Advance the playing animation by ``dt`` seconds (stops at the end)."""
        if not self.playing:
            return
        t = self.time + self.speed * dt
        if t >= self.T:
            t = self.T
            self.playing = False
            if self._timer is not None:
                self._timer.stop()
        self.set_time(t)

    def _tick(self):
        now = time.perf_counter()
        dt = now - getattr(self, "_last_tick", now - TICK_MS / 1000.0)
        self._last_tick = now
        self.advance(min(max(dt, 0.0), 0.25))  # wall-clock speed, whatever the redraw cost

    def pause(self):
        if self.playing:
            self.toggle_play()

    # -- events
    def on_motion(self, event):
        if event.inaxes is self.tl and event.xdata is not None:
            self.pause()
            self.set_time(event.xdata)
        elif event.inaxes is self.ax:
            self.set_highlight(self.robot_at(event.xdata, event.ydata))

    def on_key(self, event):
        k = event.key
        if k == " ":
            self.toggle_play()
        elif k == "right":
            self.pause()
            self.step(1)
        elif k == "left":
            self.pause()
            self.step(-1)
        elif k == "home":
            self.pause()
            self.set_time(0)
        elif k == "end":
            self.pause()
            self.set_time(self.T)
        elif k in ("k", "K") and len(self.plans) > 1:
            self.cycle_plan(1 if k == "k" else -1)
        elif k == "g":
            self.toggle_markers()
        elif k == "up":
            self.speed = min(self.speed * 1.5, 30.0)
        elif k == "down":
            self.speed = max(self.speed / 1.5, 0.2)
        elif k in ("q", "escape"):
            import matplotlib.pyplot as plt

            plt.close(self.fig)

    def disconnect(self):
        for cid in self._cids:
            self.fig.canvas.mpl_disconnect(cid)
        self._cids = []


# ----------------------------------------------------------------------------- entry points
def _make(problem_or_graph, starts, goals, paths, plans, title, fig, interactive, colors=None):
    graph, s, g = _instance(problem_or_graph, starts, goals)
    pl, start = _plans(paths, plans, len(s))
    if not pl:
        pl = [("start configuration", [[v] for v in s], {})]
    return Viewer(fig, graph, s, g, pl, start_plan=start, title=title, interactive=interactive, colors=colors)


def _agg_figure(width, dpi):
    _mpl()
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.figure import Figure

    fig = Figure(figsize=(width, width), dpi=dpi)
    FigureCanvasAgg(fig)
    return fig


def make_viewer(problem_or_graph, starts=None, goals=None, paths=None, plans=None, title=None, *, width=6.0,
           dpi=100, colors=None, interactive=True):
    """An off-screen (Agg) :class:`Viewer` with its events connected (for tests and
    notebooks): drive it with its methods or by feeding events to ``viewer.fig.canvas``."""
    fig = _agg_figure(width, dpi)
    return _make(problem_or_graph, starts, goals, paths, plans, title, fig, interactive, colors)


def show(problem_or_graph, starts=None, goals=None, paths=None, plans=None, title=None, *, width=7.0,
         block=None, colors=None):
    """Open an interactive window for ``paths`` (and / or the ``plans`` to cycle with
    ``k``) and return the :class:`Viewer`. See the module docstring for the controls."""
    _mpl()
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(width, width))
    v = _make(problem_or_graph, starts, goals, paths, plans, title, fig, True, colors)
    mgr = getattr(fig.canvas, "manager", None)
    if mgr is not None:
        hid = getattr(mgr, "key_press_handler_id", None)
        if hid is not None:  # the default keymap ('k' = log x, 'g' = grid, arrows = history)
            fig.canvas.mpl_disconnect(hid)
        try:
            mgr.set_window_title(title or "mppflow viewer")
        except Exception:  # pragma: no cover - backend specific
            pass
    fig._mppflow_viewer = v  # keep a reference while the window is open
    plt.show(block=block)
    return v


def _times(T, frames_per_step, hold):
    n = max(int(T * frames_per_step), 0)
    ts = [T * i / n for i in range(n + 1)] if n else [0.0]
    return ts + [float(T)] * max(int(hold), 0)


def render_frames(problem_or_graph, starts=None, goals=None, paths=None, plans=None, title=None, *,
                  plan=None, times=None, frames_per_step=4, highlight=None, width=6.0, dpi=80, colors=None):
    """Render the plan off screen (Agg) and return the frames as ``H x W x 4`` ``uint8``
    arrays. ``times`` (default: ``frames_per_step`` frames per time step, from 0 to the
    makespan) are continuous times; ``plan`` picks one of ``plans`` (index or label);
    ``highlight`` a robot whose path is drawn."""
    import numpy as np

    v = make_viewer(problem_or_graph, starts, goals, paths, plans, title, width=width, dpi=dpi, colors=colors,
               interactive=False)
    _pick(v, plan)
    if highlight is not None:
        v.set_highlight(highlight)
    out = []
    for t in (times if times is not None else _times(v.T, frames_per_step, 0)):
        v.set_time(t)
        v.fig.canvas.draw()
        out.append(np.asarray(v.fig.canvas.buffer_rgba()).copy())
    return out


def _pick(v, plan):
    if plan is None:
        return
    if isinstance(plan, int):
        v.set_plan(plan)
        return
    for i, (lab, _, _) in enumerate(v.plans):
        if lab == plan:
            v.set_plan(i)
            return
    raise ValueError(f"no plan labeled {plan!r}")


def _compact_writer(fps, colors):
    """A ``PillowWriter`` that quantizes every frame to ``colors`` colors without dithering
    (flat diagrams compress far better) and lets Pillow drop unchanged pixels."""
    from matplotlib.animation import PillowWriter

    class _Writer(PillowWriter):
        def finish(self):
            if not colors:
                return super().finish()
            from PIL import Image

            dither = getattr(Image, "Dither", Image).NONE
            rgb = [f.convert("RGB") for f in self._frames]
            # One palette for all frames, so that Pillow only stores the changed regions.
            sample = rgb[:: max(1, len(rgb) // 12)][:12] + [rgb[-1]]
            w, h = sample[0].size
            sheet = Image.new("RGB", (w, h * len(sample)))
            for i, f in enumerate(sample):
                sheet.paste(f, (0, h * i))
            pal = sheet.quantize(colors=colors, dither=dither)
            # Keep the background exact: median cut averages it with nearby shades, and
            # Pillow's palette lookup is approximate, so snap every near-background entry.
            entries = pal.getpalette()
            bg = list(sheet.getpixel((0, 0)))
            for k in range(len(entries) // 3):
                if max(abs(entries[3 * k + j] - bg[j]) for j in range(3)) <= 8:
                    entries[3 * k:3 * k + 3] = bg
            pal.putpalette(entries)
            frames = [f.quantize(palette=pal, dither=dither) for f in rgb]
            frames[0].save(self.outfile, save_all=True, append_images=frames[1:],
                           duration=int(1000 / self.fps), loop=0, optimize=True)

    return _Writer(fps=fps)


def save_gif(path, problem_or_graph, starts=None, goals=None, paths=None, plans=None, title=None, *,
             plan=None, fps=12, frames_per_step=6, hold=None, highlight=None, width=6.0, dpi=80, colors=None,
             all_plans=False, palette=64):
    """Write an animated GIF of the plan with matplotlib's ``PillowWriter`` and return
    ``path``. ``hold`` frames (default ``fps``) repeat the final configuration;
    ``all_plans=True`` plays every plan of ``plans`` one after the other. Frames are
    quantized to ``palette`` colors (``0``: Pillow's default conversion, larger files)."""
    v = make_viewer(problem_or_graph, starts, goals, paths, plans, title, width=width, dpi=dpi, colors=colors,
               interactive=False)
    hold = fps if hold is None else hold
    order = range(len(v.plans)) if all_plans else [None]
    writer = _compact_writer(fps, palette)
    with writer.saving(v.fig, str(path), dpi):
        for idx in order:
            if idx is None:
                _pick(v, plan)
            else:
                v.set_plan(idx)
            if highlight is not None:
                v.set_highlight(highlight)
            for t in _times(v.T, frames_per_step, hold):
                v.set_time(t)
                writer.grab_frame()
    return path
