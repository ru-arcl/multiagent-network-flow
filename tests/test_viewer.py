"""The matplotlib viewer (mppflow.viewer), off screen with the Agg backend: frames, mouse
scrubbing on the timeline, hover highlight, keys, plan cycling, GIF output and the CLI
``view`` command."""
from __future__ import annotations

import math

import pytest

matplotlib = pytest.importorskip("matplotlib")
matplotlib.use("Agg")

from matplotlib.backend_bases import KeyEvent, MouseEvent  # noqa: E402

from mppflow import cli, viewer as V  # noqa: E402
from mppflow._java import MathRandom  # noqa: E402
from mppflow.graph import Graph  # noqa: E402
from mppflow.problem import Problem, create_n2_puzzle  # noqa: E402


# 2x2 grid 0 1 / 2 3; the four robots rotate along the cycle 0 -> 1 -> 3 -> 2 -> 0 twice.
XY = [(0, 0), (1, 0), (0, 1), (1, 1)]
ADJ = [[1, 2], [0, 3], [0, 3], [1, 2]]
FAST = [[0, 1, 3], [1, 3, 2], [2, 0, 1], [3, 2, 0]]
SLOW = [[0, 0, 1, 3], [1, 1, 3, 2], [2, 2, 0, 1], [3, 3, 2, 0]]


def square():
    g = Graph.from_adjacency(ADJ, XY)
    return Problem(g, [0, 1, 2, 3], [3, 2, 1, 0])


def mouse(v, ax, x, y, name="motion_notify_event"):
    """Send a mouse event at the data point ``(x, y)`` of ``ax``."""
    v.fig.canvas.draw()
    px, py = ax.transData.transform((x, y))
    ev = MouseEvent(name, v.fig.canvas, px, py)
    assert ev.inaxes is ax
    v.fig.canvas.callbacks.process(name, ev)
    return ev


def key(v, k):
    ev = KeyEvent("key_press_event", v.fig.canvas, k)
    v.fig.canvas.callbacks.process("key_press_event", ev)


def test_layout_grid_and_spring():
    verts, pos, edges, grid = V.layout(square().graph)
    assert grid and verts == [0, 1, 2, 3] and pos[3] == (1.0, 1.0)
    assert sorted(edges) == [(0, 1), (0, 2), (1, 3), (2, 3)]
    tree = Graph.from_adjacency([[1], [0, 2, 3], [1], [1, 4, 5], [3], [3]])
    a = V.layout(tree)
    b = V.layout(tree)
    assert not a[3] and a[1] == b[1]  # no coordinates -> deterministic spring layout
    lens = sorted(math.dist(a[1][u], a[1][v]) for u, v in a[2])
    assert lens[len(lens) // 2] == pytest.approx(1.0)
    assert len(set(a[1].values())) == 6


def test_positions_interpolate_along_edges():
    v = V.make_viewer(square(), paths=FAST)
    assert v.T == 2
    P = v.positions(0.5)
    assert P[0].tolist() == [0.5, 0.0] and P[1].tolist() == [1.0, 0.5]
    assert v.positions(2).tolist() == [[1, 1], [0, 1], [1, 0], [0, 0]]
    assert v.positions(99).tolist() == v.positions(2).tolist()


def test_render_frames():
    frames = V.render_frames(square(), paths=FAST, title="square", frames_per_step=3, dpi=50)
    assert len(frames) == 2 * 3 + 1
    h, w, c = frames[0].shape
    assert c == 4 and h > 100 and w > 100
    assert all(f.shape == frames[0].shape for f in frames)
    assert (frames[0] != frames[3]).any() and (frames[3] != frames[-1]).any()
    again = V.render_frames(square(), paths=FAST, title="square", times=[1.0], dpi=50)
    assert (again[0] == frames[3]).all()  # deterministic


def test_timeline_scrubs_time():
    v = V.make_viewer(square(), paths=SLOW)
    assert v.time == 0
    seen = []
    for x in (0.25, 1.4, 2.5, 2.9):
        mouse(v, v.tl, x, 0.5)
        seen.append(v.time)
        assert v.cursor.get_xdata()[0] == pytest.approx(v.time)
    assert seen == pytest.approx([0.25, 1.4, 2.5, 2.9], abs=0.02)
    # robot 0 moves 1 -> 3, i.e. (1, 0) -> (1, 1), during [2, 3]
    assert v.positions()[0] == pytest.approx([1.0, v.time - 2.0], abs=1e-9)
    assert "t =  2.9" in v.time_text.get_text()
    # moving over the graph does not change the time; leaving the strip keeps it
    mouse(v, v.ax, 0.5, 0.5)
    assert v.time == pytest.approx(seen[-1])
    # playing stops when the mouse scrubs
    v.toggle_play()
    assert v.playing
    mouse(v, v.tl, 1.0, 0.5)
    assert not v.playing and v.time == pytest.approx(1.0, abs=0.02)


def test_hover_highlights_robot():
    v = V.make_viewer(square(), paths=SLOW)
    v.set_time(2.5)
    x, y = v.positions()[2]  # robot 3 (index 2), on the edge 0 -> 1
    mouse(v, v.ax, x, y)
    assert v.highlight == 2
    info = v.info_text.get_text()
    assert info.startswith("robot 3:") and "arrival 3" in info and "distance 2" in info
    xs, ys = v.hl_line.get_data()
    assert list(zip(xs, ys)) == [(0, 1), (0, 0), (1, 0)]
    assert v.hl_arrival.get_visible() and v.hl_arrival.get_xdata()[0] == 3
    assert v.robot_col.get_facecolors()[2].tolist() != v.robot_col.get_facecolors()[0].tolist()
    mouse(v, v.ax, 0.5, 0.5)  # centre of the square: nothing there
    assert v.highlight is None and v.info_text.get_text() == ""
    assert len(v.hl_line.get_data()[0]) == 0 and not v.hl_goal.get_visible()
    # hovering a goal marker highlights its robot
    p = Problem(square().graph, [0, 2], [3, 2])
    v = V.make_viewer(p, paths=[[0, 1, 3], [2, 2, 2]])
    mouse(v, v.ax, 1.0 + 0.5 * v.r, 1.0)  # vertex 3, empty at t = 0
    assert v.highlight == 0 and v.hl_goal.get_visible()
    assert "arrival 2, distance 2" in v.info_text.get_text()


def test_keys_and_plan_cycling():
    v = V.make_viewer(square(), plans={"slow": SLOW, "fast": FAST})
    assert v.label == "slow" and v.T == 3
    key(v, "right")
    assert v.time == 1
    v.set_time(1.4)
    key(v, "right")
    assert v.time == 2
    key(v, "left")
    key(v, "left")
    assert v.time == 0
    key(v, "end")
    assert v.time == 3
    key(v, " ")
    assert v.playing and v.time == 0  # restarts from the beginning
    v.advance(0.25)
    assert v.time == pytest.approx(0.25 * V.SPEED)
    key(v, " ")
    assert not v.playing
    v.playing = True
    v.advance(100)
    assert v.time == 3 and not v.playing
    key(v, "k")
    assert v.label == "fast" and v.T == 2 and v.time == 2
    assert "fast (2/2)" in v.status_text.get_text()
    key(v, "k")
    assert v.label == "slow"
    key(v, "K")
    assert v.label == "fast"
    key(v, "g")
    assert not v.goal_col.get_visible()
    key(v, "g")
    assert v.goal_col.get_visible()


def test_plans_with_paths_and_plan_results():
    p = create_n2_puzzle(3, MathRandom(1))
    from mppflow.planner import plan

    r = plan(p, "makespan", 1)
    v = V.make_viewer(p, paths=r.paths, plans=[r])
    assert len(v.plans) == 1 and v.label == "makespan k=1"
    m = v.metrics
    assert m["makespan"] == r.makespan and m["total_distance"] == r.total_distance
    assert "makespan 4" in v.status_text.get_text() and v.error is None


def test_split_boundaries_and_invalid_plan():
    p = square()
    info = {"intermediate_goals": [[1, 3, 0, 2]], "paths": SLOW}
    v = V.make_viewer(p, plans={"split": info})
    assert v._boundaries == [2]
    bad = [row[:] for row in FAST]
    bad[1] = [1, 1, 2]  # jumps along a diagonal
    v = V.make_viewer(p, paths=bad)
    assert v.error and v.status_text.get_text().startswith("INVALID PLAN")


def test_input_errors():
    p = square()
    with pytest.raises(ValueError):
        V.make_viewer(p, paths=FAST[:3])
    with pytest.raises(ValueError):
        V.make_viewer(p, paths=[[0, 1], [1, 3, 2], [2, 0, 1], [3, 2, 0]])
    with pytest.raises(ValueError):
        V.make_viewer(p.graph, paths=FAST)  # no starts / goals
    with pytest.raises(ValueError):
        V.make_viewer(p, paths=[[0, 7, 3], [1, 3, 2], [2, 0, 1], [3, 2, 0]])  # vertex 7
    v = V.make_viewer(p.graph, p.starts, p.goals)  # no plan: the start configuration
    assert v.T == 0 and v.positions().tolist() == [list(map(float, XY[s])) for s in p.starts]


def test_save_gif(tmp_path):
    from PIL import Image

    out = tmp_path / "square.gif"
    V.save_gif(out, square(), plans={"slow": SLOW, "fast": FAST}, all_plans=True, fps=10, frames_per_step=2,
               hold=2, dpi=40)
    with Image.open(out) as im:
        assert im.format == "GIF" and im.n_frames > 3
    assert out.stat().st_size < 400_000


def test_show_disconnects_default_keymap(monkeypatch):
    import matplotlib.pyplot as plt

    monkeypatch.setattr(plt, "show", lambda *a, **k: None)
    v = V.show(square(), paths=FAST, title="square")
    try:
        key(v, "k")  # the default keymap would set a log x scale
        key(v, "right")
        assert v.ax.get_xscale() == "linear" and v.time == 1
    finally:
        plt.close(v.fig)


def test_cli_view(monkeypatch, tmp_path, capsys):
    shown = {}

    def fake_show(problem, starts=None, goals=None, paths=None, plans=None, title=None, **kw):
        shown.update(problem=problem, plans=plans, title=title)

    monkeypatch.setattr(V, "show", fake_show)
    assert cli.main(["view", "--grid", "8x8", "--robots", "20", "--split", "2"]) == 0
    assert list(shown["plans"]) == ["makespan k=2"] and "8x8 grid" in shown["title"]
    res = shown["plans"]["makespan k=2"]
    assert res.split == 2 and len(res.paths) == 20
    # identical to 'solve --split 2' with the same seed
    capsys.readouterr()
    cli.main(["solve", "--grid", "8x8", "--robots", "20", "--split", "2", "--json"])
    import json

    assert json.loads(capsys.readouterr().out)["paths"] == res.paths
    gif = tmp_path / "p.gif"
    assert cli.main(["view", "--puzzle", "3", "--gif", str(gif)]) == 0
    gif2 = tmp_path / "g.gif"
    assert cli.main(["view", "--grid", "4x4", "--robots", "4", "--split", "1,2", "--gif", str(gif2)]) == 0
    assert gif2.exists()
    assert gif.exists() and "wrote" in capsys.readouterr().out
    with pytest.raises(SystemExit):
        cli.main(["view", "--puzzle", "3", "--split", "x"])
    # --title, --8-connected and --fps reach the viewer
    assert cli.main(["view", "--grid", "3x3", "--robots", "2", "--8-connected", "--title", "T"]) == 0
    assert shown["title"] == "T" and shown["problem"].graph.neighbors(4) == list(range(9))
    saved = {}
    monkeypatch.setattr(V, "save_gif", lambda path, problem, **kw: saved.update(kw))
    assert cli.main(["view", "--puzzle", "3", "--gif", str(tmp_path / "f.gif"), "--fps", "7"]) == 0
    assert saved["fps"] == 7 and saved["title"] == "9-puzzle, seed 1"
