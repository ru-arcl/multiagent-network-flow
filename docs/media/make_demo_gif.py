#!/usr/bin/env python3
"""Assemble docs/media/demo.gif from the frames that make_demo_gif.mjs captured.

Every picture in the GIF is a crop of a real screenshot of docs/index.html: the page's title and
subtitle, its drawing of the graph and the robots, its time slider and time label, and its
time-expanded network with its legend. This script only crops, scales and arranges them on an
800-pixel-wide canvas. It draws no robots and no text; the only thing it draws is the rounded tile,
in the view's own colour, behind each cropped graph view. Each view is cropped with one fixed box
per scene (the union of the drawn content over all frames), so the tiles do not move.

    node docs/media/make_demo_gif.mjs --out FRAMES
    python3 docs/media/make_demo_gif.py FRAMES [-o docs/media/demo.gif]

Needs Pillow and NumPy. The GIF uses one palette for all frames (96 colours by default, no
dithering, so the discs and labels stay crisp and frames do not flicker). It loops forever. Pixels
that do not change are transparent in the delta frames, and each run of identical frames is stored
once with a longer duration.
"""

from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np
from PIL import Image, ImageChops, ImageDraw

W, H = 800, 640  # canvas (pixels)
M = 14  # outer margin
GAP = 10
WHITE = (255, 255, 255)
VIEW_BORDER = (214, 220, 227)


class Clip:
    """One captured PNG and the element boxes inside it (CSS pixels relative to the page)."""

    def __init__(self, folder: str, scene: dict, key: str, frame: dict):
        self.img = Image.open(os.path.join(folder, frame["files"][key])).convert("RGB")
        self.origin = scene["clips"][key]
        self.boxes = scene["boxes"]
        self.scale = self.img.width / self.origin["width"]

    def crop(self, sel: str, pad: float = 0) -> Image.Image:
        b, o, s = self.boxes[sel], self.origin, self.scale
        x0 = round((b["x"] - o["x"] - pad) * s)
        y0 = round((b["y"] - o["y"] - pad) * s)
        x1 = round((b["x"] + b["width"] - o["x"] + pad) * s)
        y1 = round((b["y"] + b["height"] - o["y"] + pad) * s)
        return self.img.crop((max(0, x0), max(0, y0), min(self.img.width, x1), min(self.img.height, y1)))


def content_box(img: Image.Image, pad: int) -> tuple[int, int, int, int]:
    """The box of everything drawn in a view crop that is not its background colour."""
    bg = img.getpixel((img.width // 2, 8))
    e = 8  # skip the view's border, which the box of a wide view touches
    inner = img.crop((e, e, img.width - e, img.height - e))
    diff = ImageChops.difference(inner, Image.new("RGB", inner.size, bg)).convert("L").point(lambda v: 255 if v > 24 else 0)
    x0, y0, x1, y1 = diff.getbbox() or (0, 0, inner.width, inner.height)
    x0, y0, x1, y1 = x0 + e, y0 + e, x1 + e, y1 + e
    return max(0, x0 - pad), max(0, y0 - pad), min(img.width, x1 + pad), min(img.height, y1 + pad)


def fit(img: Image.Image, max_w: float, max_h: float, max_scale: float = 1.0) -> Image.Image:
    s = min(max_w / img.width, max_h / img.height, max_scale)
    size = (max(1, round(img.width * s)), max(1, round(img.height * s)))
    return img if size == img.size else img.resize(size, Image.LANCZOS)


def css(img: Image.Image, clip: Clip, factor: float = 1.0) -> Image.Image:
    """Scale a crop to `factor` times its CSS size (the screenshots are taken at 2x)."""
    s = factor / clip.scale
    return img.resize((max(1, round(img.width * s)), max(1, round(img.height * s))), Image.LANCZOS)


def panel(canvas: Image.Image, img: Image.Image, x: int, y: int, bg) -> None:
    """Paste a view crop on a rounded tile of the view's own background, as the page frames it."""
    d = ImageDraw.Draw(canvas)
    d.rounded_rectangle((x - 6, y - 6, x + img.width + 5, y + img.height + 5), radius=8, fill=bg, outline=VIEW_BORDER)
    canvas.paste(img, (x, y))


def view_content(clip: Clip, box: tuple[int, int, int, int]) -> tuple[Image.Image, tuple]:
    view = clip.crop("#view-svg rect.bg")
    return view.crop(box), view.getpixel((view.width // 2, 8))


def scene_box(folder: str, scene: dict, pad_px: int) -> tuple[int, int, int, int]:
    """One crop box for the graph view of a whole scene: the union of the drawn content of every frame."""
    boxes = []
    for fr in scene["frames"]:
        c = Clip(folder, scene, "stage", fr)
        boxes.append(content_box(c.crop("#view-svg rect.bg"), round(pad_px * c.scale)))
    return (min(b[0] for b in boxes), min(b[1] for b in boxes), max(b[2] for b in boxes), max(b[3] for b in boxes))


def time_bar(c: Clip, width: int) -> Image.Image:
    """The page's time slider and its "t = ... / T" label in one row (the play buttons are left out)."""
    scrub = css(c.crop(".pb-scrub", 1), c, 1.0)
    status = css(c.crop(".pb-status", 1), c, 1.0)
    scrub = fit(scrub, width - status.width - 16, 100, 1.0)
    h = max(scrub.height, status.height)
    bar = Image.new("RGB", (scrub.width + 16 + status.width, h), WHITE)
    bar.paste(scrub, (0, (h - scrub.height) // 2))
    bar.paste(status, (scrub.width + 16, (h - status.height) // 2))
    return bar


def compose_single(folder: str, scene: dict, frame: dict, crops: dict) -> Image.Image:
    """The 9-puzzle and the tree: the stage heading, the drawing (with the goal configuration on the
    right, or the time-expanded network below) and the time slider."""
    c = Clip(folder, scene, "stage", frame)
    canvas = Image.new("RGB", (W, H), WHITE)
    head = css(c.crop(".stage-head"), c, (W - 2 * M) / c.boxes[".stage-head"]["width"])
    canvas.paste(head, (M, M))
    bar = time_bar(c, W - 2 * M - 40)
    canvas.paste(bar, ((W - bar.width) // 2, H - M - bar.height))
    top, bottom = M + head.height + 14, H - M - bar.height - 14
    view, bg = view_content(c, crops.setdefault("box", scene_box(folder, scene, 14)))
    if scene["name"] == "puzzle":
        right = crops.setdefault("goal", (c.crop("#goal-svg", 4), c.crop("#goal-inset figcaption strong", 2)))
        goal = css(right[0], c, 1.3)
        cap = css(right[1], c, 1.0)
        rw = max(goal.width, cap.width)
        v = fit(view, 400, min(400, bottom - top - 12))
        x = M + 6 + (W - 2 * M - rw - 40 - v.width) // 2
        panel(canvas, v, x, top + 6 + (bottom - top - 12 - v.height) // 2, bg)
        gx = W - M - rw
        gy = top + (bottom - top - goal.height - cap.height - 8) // 2
        canvas.paste(cap, (gx + (rw - cap.width) // 2, gy))
        canvas.paste(goal, (gx + (rw - goal.width) // 2, gy + cap.height + 8))
    else:
        # the network fills the width at the bottom; the tree, the network's title and its legend above it
        n = Clip(folder, scene, "net", frame)
        nh = Clip(folder, scene, "nethead", frame)
        lg = Clip(folder, scene, "legend", frame)
        net = fit(n.img, W - 2 * M, bottom - top - 160, 2.0)
        ny = bottom - net.height
        canvas.paste(net, ((W - net.width) // 2, ny))
        v = fit(view, 300, ny - 14 - top - 12, 2.0)
        vx = M + 40
        panel(canvas, v, vx, top + 6, bg)
        title = css(nh.img, nh, 1.1)
        legend = css(lg.img, lg, 1.1)
        rx = vx + v.width + 6 + 48
        ry = top + 6 + (v.height - title.height - 14 - legend.height) // 2
        canvas.paste(title, (rx, ry))
        canvas.paste(legend, (rx, ry + title.height + 14))
    return canvas


def compose_split(folder: str, scene: dict, i: int, crops: dict) -> Image.Image:
    """The 24 x 18 grid: the exact plan (k = 1) and the 4-way split (k = 4) side by side, at the same time."""
    canvas = Image.new("RGB", (W, H), WHITE)
    pw = (W - 2 * M - 2 * GAP) // 2
    clips = [Clip(folder, p, "stage", p["frames"][i]) for p in scene["panels"]]
    title = crops.setdefault("title", css(clips[0].crop("#view-title", 2), clips[0], 1.0))
    canvas.paste(title, ((W - title.width) // 2, M))
    y0 = M + title.height + 10
    boxes = crops.setdefault("boxes", [scene_box(folder, p, 4) for p in scene["panels"]])
    for j, c in enumerate(clips):
        x = M + j * (pw + 2 * GAP)
        sub = css(c.crop("#view-sub", 1), c, 1.0)
        status = css(c.crop(".pb-status", 1), c, 1.0)
        scrub = fit(c.crop(".pb-scrub", 1), pw, 100, 1.0)
        view, bg = view_content(c, boxes[j])
        avail = H - M - y0 - sub.height - 8 - scrub.height - status.height - 6 - 14
        v = fit(view, pw - 12, avail)
        canvas.paste(sub, (x + (pw - sub.width) // 2, y0))
        vy = y0 + sub.height + 14
        panel(canvas, v, x + (pw - v.width) // 2, vy, bg)
        sy = vy + v.height + 14
        canvas.paste(scrub, (x + (pw - scrub.width) // 2, sy))
        canvas.paste(status, (x + (pw - status.width) // 2, sy + scrub.height + 4))
    return canvas


def palette(scenes: list[list[Image.Image]], colors: int, exact: int = 24) -> Image.Image:
    """One palette for the whole GIF. First the `exact` most frequent colours of each scene, as they are (the
    page's flat fills, accent colours and flow lines keep their exact colour), then a median-cut palette of
    all frames for the anti-aliased edges. At most 255 colours, so that Pillow has a free index for the
    transparent pixels of its delta frames."""
    chosen: list[tuple[int, int, int]] = []

    def add(c, tol):
        if len(chosen) < colors and all(sum(abs(x - y) for x, y in zip(c, d)) > tol for d in chosen):
            chosen.append(tuple(c))

    def stack(frames):
        strip = Image.new("RGB", (W, H * len(frames)))
        for k, im in enumerate(frames):
            strip.paste(im, (0, H * k))
        return strip

    for frames in scenes:
        top = len(chosen) + exact
        for _, c in sorted(stack(frames).getcolors(1 << 24), reverse=True):
            if len(chosen) >= top:
                break
            add(c, 12)
    every = stack([im for frames in scenes for im in frames])
    mc = every.quantize(colors=colors, method=Image.Quantize.MEDIANCUT, dither=Image.Dither.NONE).getpalette()
    for k in range(colors):
        add(mc[3 * k: 3 * k + 3], 6)
    pal = Image.new("P", (1, 1))
    pal.putpalette([v for c in chosen for v in c])
    return pal


def to_palette(img: Image.Image, pal: Image.Image) -> Image.Image:
    """Map every pixel to its nearest palette colour, exactly. (Image.quantize(palette=...) goes through a
    colour cache that can map white to a near-white entry even when white is in the palette.)"""
    colors = np.array(pal.getpalette(), dtype=np.int32).reshape(-1, 3)
    rgb = np.asarray(img.convert("RGB"), dtype=np.int32)
    keys = (rgb[..., 0] << 16) | (rgb[..., 1] << 8) | rgb[..., 2]
    uniq, inverse = np.unique(keys.ravel(), return_inverse=True)
    u = np.stack([uniq >> 16, (uniq >> 8) & 255, uniq & 255], axis=1)
    nearest = ((u[:, None, :] - colors[None, :, :]) ** 2).sum(axis=2).argmin(axis=1).astype(np.uint8)
    out = Image.fromarray(nearest[inverse].reshape(keys.shape), "P")
    out.putpalette(colors.ravel().astype(np.uint8).tolist())
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("frames", help="the --out directory of make_demo_gif.mjs")
    ap.add_argument("-o", "--output", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "demo.gif"))
    ap.add_argument("--max-bytes", type=int, default=1_800_000, help="fail if the GIF is larger (default 1.8 MB)")
    ap.add_argument("--colors", type=int, default=96, help="palette size (default 96)")
    ap.add_argument("--keep-png", help="also write every composed frame as a PNG into this directory")
    a = ap.parse_args(argv)
    with open(os.path.join(a.frames, "frames.json"), encoding="utf-8") as f:
        man = json.load(f)

    frames, durations, groups = [], [], []
    for scene in man["scenes"]:
        groups.append([])
        crops: dict = {}
        for i, fr in enumerate(scene["frames"]):
            if scene.get("layout") == "split":
                img = compose_split(a.frames, scene, i, crops)
            else:
                img = compose_single(a.frames, scene, fr, crops)
            frames.append(img)
            durations.append(fr["duration"])
            if i % 2 == 0:
                groups[-1].append(img)

    pal = palette(groups, min(255, a.colors))
    out = [to_palette(im, pal) for im in frames]
    if a.keep_png:
        os.makedirs(a.keep_png, exist_ok=True)
        for k, im in enumerate(frames):
            im.save(os.path.join(a.keep_png, "frame-%04d.png" % k))

    out[0].save(a.output, save_all=True, append_images=out[1:], duration=durations, loop=0,
                optimize=True, disposal=1)
    size = os.path.getsize(a.output)
    total = sum(durations) / 1000
    print("%s: %d frames, %.1f s, %dx%d, %.2f MB" % (a.output, len(out), total, W, H, size / 1e6))
    if size > a.max_bytes:
        print("error: larger than %d bytes" % a.max_bytes, file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
