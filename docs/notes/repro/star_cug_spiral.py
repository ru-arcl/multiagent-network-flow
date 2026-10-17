"""STAR 2013 Sec. 2: "any graph with unit edge length and no acute angles between adjacent
edges is a CUG" is false.

A collision-free unit-distance graph (CUG) needs every two distinct edges (u1,v1), (u2,v2)
with u1 != u2 and v1 != v2 to be traversable at the same time by discs of radius r <
sqrt(2)/4 without contact, i.e. the centres stay >= 2r apart. Angles between adjacent edges
say nothing about edges that are far apart in the graph but close in the plane. A
polygonal spiral with unit edges, r(phi) = 6 - 0.3 phi / (2 pi), two turns, has only
obtuse angles (about 170 degrees) but successive turns only 0.3 apart.
"""
import math

R0, PITCH, TURNS = 6.0, 0.3, 2


def p(phi):
    r = R0 - PITCH * phi / (2 * math.pi)
    return (r * math.cos(phi), r * math.sin(phi))


pts, phi = [p(0.0)], 0.0
while phi < TURNS * 2 * math.pi:
    lo, hi = phi, phi + 1.0
    for _ in range(100):  # the next vertex at chord length exactly 1
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if math.dist(p(mid), pts[-1]) < 1 else (lo, mid)
    phi = (lo + hi) / 2
    pts.append(p(phi))
edges = [(i, i + 1) for i in range(len(pts) - 1)]


def angle(i):
    (ax, ay), (bx, by), (cx, cy) = pts[i - 1], pts[i], pts[i + 1]
    u, v = (ax - bx, ay - by), (cx - bx, cy - by)
    return math.degrees(math.acos((u[0] * v[0] + u[1] * v[1]) / (math.hypot(*u) * math.hypot(*v))))


def min_dist(u1, v1, u2, v2):  # both discs leave u1, u2 at time 0 at unit speed
    a, b, c, d = pts[u1], pts[v1], pts[u2], pts[v2]
    dx0, dy0 = a[0] - c[0], a[1] - c[1]
    dx1, dy1 = (b[0] - a[0]) - (d[0] - c[0]), (b[1] - a[1]) - (d[1] - c[1])
    den = dx1 * dx1 + dy1 * dy1
    s = 0.0 if den == 0 else min(1.0, max(0.0, -(dx0 * dx1 + dy0 * dy1) / den))
    return math.hypot(dx0 + s * dx1, dy0 + s * dy1)


lengths = [math.dist(pts[i], pts[j]) for i, j in edges]
angles = [angle(i) for i in range(1, len(pts) - 1)]
best = (math.inf, None)
for e1 in edges:
    for e2 in edges:
        if e1 == e2:
            continue
        for (u1, v1) in (e1, e1[::-1]):
            for (u2, v2) in (e2, e2[::-1]):
                if u1 != u2 and v1 != v2:
                    best = min(best, (min_dist(u1, v1, u2, v2), (u1, v1, u2, v2)))
print(f"{len(pts)} vertices, {len(edges)} edges, edge length {min(lengths):.6f}..{max(lengths):.6f}")
print(f"angle between adjacent edges: {min(angles):.1f}..{max(angles):.1f} degrees (none acute)")
print(f"closest approach of two discs on qualifying edges: {best[0]:.3f} at edges {best[1]}")
print(f"required: >= 2r, i.e. any r in ({best[0] / 2:.3f}, sqrt(2)/4 = {math.sqrt(2) / 4:.3f}) collides")
