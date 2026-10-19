"""A planned route through a course for one or more laps, and the checks that keep it safe.

The route: take-off point over the start pad, then for every gate a point before its centre along the passage direction, the
centre and a point after, plus the detours of the route file; between two laps the route file's `lap_junction` points; then the
end pad at hover height. A cubic spline through these points at a constant speed, at rest at both ends.

The checks: the distance from a point to the nearest solid (gate frame bars, gate posts, poles, boxes), and the BACKWARD crossings
of a gate opening. The second check exists because of gate G7 on 2026-10-18: the path from the end of lap 1 into lap 2 went
through G7 backwards, 0.26 m from its centre, and the drone stuck on the frame. The forward counter (course.LapCounter) cannot see
that, because a backward crossing is not a crossing for it.

Sizes of the solids follow the PX4 + Gazebo world we built (private repo, px4_gazebo/race_core.py): frame bars (outer - opening) / 2
wide and DEPTH_M thick, a square post of POST_M under each frame down to the ground.
"""

from __future__ import annotations

import math

import numpy as np
from scipy.interpolate import CubicSpline

from .course import Course

DEPTH_M = 0.2   # thickness of a frame along the passage direction, m (not scaled)
POST_M = 0.3    # side of the square post under a frame, m (not scaled)


def waypoints(course: Course, laps: int = 1, hover_z: float = 0.8, before: float = 0.35, after: float = 0.35) -> np.ndarray:
    """Route points in the course frame (already scaled). hover_z, before and after are at scale 1, like the route file."""
    spec, s = course.spec, course.scale
    det = spec.get("waypoints_hint", {}).get("detours", {})
    junction = spec.get("lap_junction", {}).get("points", [])
    if laps > 1 and not junction:
        raise ValueError("the route file has no lap_junction; a second lap needs one")
    sx, sy = spec.get("start_pad", [0.0, 0.0, 0.0])[:2]
    pts = [[sx, sy, hover_z]]
    for lap in range(laps):
        if lap > 0:
            pts += [list(q) for q in junction]
        for g in spec["gates"]:
            c = np.array(g["center"], float)
            d = np.array([math.cos(math.radians(g["yaw_deg"])), math.sin(math.radians(g["yaw_deg"])), 0.0])
            pts += [list(c - before * d), list(c), list(c + after * d)]
            if f"after_{g['id']}" in det:
                pts.append(list(det[f"after_{g['id']}"]))
    ex, ey = spec["end_pad"][:2]
    pts.append([ex, ey, hover_z])
    return np.array(pts, float) * s


class Route:
    """Cubic spline through the waypoints at `speed` m/s (time = distance along the polyline / speed), at rest at both ends."""

    def __init__(self, course: Course, laps: int = 1, speed: float = 1.1, **kw):
        self.course, self.laps, self.speed = course, laps, speed
        self.wp = waypoints(course, laps, **kw)
        seg = np.linalg.norm(np.diff(self.wp, axis=0), axis=1)
        if (seg < 1e-6).any():
            raise ValueError("two route points coincide")
        self.t_knots = np.concatenate([[0.0], np.cumsum(seg)]) / speed
        self.duration = float(self.t_knots[-1])
        self.spline = CubicSpline(self.t_knots, self.wp, bc_type=((1, np.zeros(3)), (1, np.zeros(3))))

    def pos_vel(self, t: float):
        """Reference position and velocity at route time t (clamped to the route; at rest outside it)."""
        u = min(max(t, 0.0), self.duration)
        v = self.spline(u, 1) if 0.0 < t < self.duration else np.zeros(3)
        return self.spline(u), v

    def sample(self, dt: float = 0.02):
        ts = np.arange(0.0, self.duration + 1e-9, dt)
        return ts, self.spline(ts)


# ------------------------------------------------------------------ solids and distances
def _box_distance(local_p, centre, half):
    d = np.abs(np.asarray(local_p, float) - np.asarray(centre, float)) - np.asarray(half, float)
    return float(np.linalg.norm(np.maximum(d, 0.0)))


def gate_boxes(course: Course, g: int):
    """The four frame bars and the post of gate g, as (centre, half sizes) in the gate's own frame (x = passage direction)."""
    op, out = course.opening, course.outer
    bar = (out - op) / 2.0
    off = op / 2.0 + bar / 2.0
    boxes = [((0.0, 0.0, off), (DEPTH_M / 2, out / 2, bar / 2)), ((0.0, 0.0, -off), (DEPTH_M / 2, out / 2, bar / 2)),
             ((0.0, -off, 0.0), (DEPTH_M / 2, bar / 2, out / 2)), ((0.0, off, 0.0), (DEPTH_M / 2, bar / 2, out / 2))]
    z = course.gates[g, 2]
    top, low = -out / 2.0, -z
    boxes.append(((0.0, 0.0, (top + low) / 2.0), (POST_M / 2, POST_M / 2, (top - low) / 2.0)))
    return boxes


def distances(course: Course, p) -> dict:
    """Distance from point p to the nearest solid of each kind: {"gate": (d, id), "pole": (d, id), "box": (d, id)}."""
    p = np.asarray(p, float)
    s = course.scale
    out = {"gate": (math.inf, None), "pole": (math.inf, None), "box": (math.inf, None)}
    for g in range(course.n):
        lp = course.local(g, p)
        for c, h in gate_boxes(course, g):
            d = _box_distance(lp, c, h)
            if d < out["gate"][0]:
                out["gate"] = (d, course.ids[g])
    for o in course.spec.get("obstacles", []):
        xy = np.asarray(o["center_xy"], float) * s
        if o["kind"] == "vertical pole":
            dxy = math.hypot(p[0] - xy[0], p[1] - xy[1]) - o["radius"] * s
            dz = max(p[2] - o["height"] * s, -p[2], 0.0)
            d, kind = math.hypot(max(dxy, 0.0), dz), "pole"
        elif o["kind"].startswith("box"):
            half, hz = o["size_xy"] * s / 2.0, o["height"] * s / 2.0
            d, kind = _box_distance(p, (xy[0], xy[1], hz), (half, half, hz)), "box"
        else:
            continue
        if d < out[kind][0]:
            out[kind] = (d, o["id"])
    return out


def backward_crossings(course: Course, P, margin: float = 0.0):
    """Crossings of a gate plane from the front to the back within the outer frame (+ margin): [(sample index, gate id, ly, lz)]."""
    P = np.asarray(P, float)
    lim = course.outer / 2.0 + margin
    found = []
    for g in range(course.n):
        L = np.array([course.local(g, p) for p in P])
        for i in np.nonzero((L[:-1, 0] >= 0.0) & (L[1:, 0] < 0.0))[0]:
            a, b = L[i], L[i + 1]
            u = a[0] / (a[0] - b[0])
            ly, lz = a[1] + u * (b[1] - a[1]), a[2] + u * (b[2] - a[2])
            if abs(ly) < lim and abs(lz) < lim:
                found.append((int(i), course.ids[g], float(ly), float(lz)))
    return found
