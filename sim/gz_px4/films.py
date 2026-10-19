"""path.mp4: the top view of the flight drawn with matplotlib (gates, solids, planned route, the flown path growing over time, lap and gate
counter), piped into ffmpeg on the runner. Runs at SPEED x simulation time. matplotlib is imported inside the function."""

from __future__ import annotations

import math
import subprocess
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))
from sim.gz_px4 import analysis as AN  # noqa: E402

FPS = 15
SPEED = 2.0


def make_path_film(course, route, poses, result: dict, out_mp4: Path) -> dict:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle

    if len(poses) < 10:
        raise RuntimeError("no truth poses to draw")
    T, P, Q = poses[:, 0], poses[:, 1:4], poses[:, 4:8]
    t_route = result.get("route_start_sim_s")
    t_route = T[0] if t_route is None else t_route
    lc, counts = AN.counter_replay(course, 2, T, P, t_route)
    yaw = np.arctan2(2 * (Q[:, 0] * Q[:, 1] + Q[:, 2] * Q[:, 3]), 1 - 2 * (Q[:, 1] ** 2 + Q[:, 2] ** 2))
    _, plan = route.sample(0.05)
    s = course.scale

    fig, ax = plt.subplots(figsize=(8, 7), dpi=100)
    ax.set_aspect("equal")
    ax.set_facecolor("#e8efe4")
    ax.grid(True, color="#c9d3c4", lw=0.5)
    ax.set_xlabel("x east (m)")
    ax.set_ylabel("y north (m)")
    allx = np.concatenate([plan[:, 0], P[:, 0], course.gates[:, 0]])
    ally = np.concatenate([plan[:, 1], P[:, 1], course.gates[:, 1]])
    ax.set_xlim(allx.min() - 3, allx.max() + 3)
    ax.set_ylim(ally.min() - 3, ally.max() + 3)
    for o in course.spec.get("obstacles", []):
        x, y = (v * s for v in o["center_xy"])
        if o["kind"] == "vertical pole":
            ax.add_patch(Circle((x, y), o["radius"] * s, color="#d62728", zorder=3))
            ax.annotate(o["id"], (x, y), xytext=(4, 4), textcoords="offset points", fontsize=7, color="#8c1a1b")
        elif o["kind"].startswith("box"):
            a = o["size_xy"] * s
            ax.add_patch(Rectangle((x - a / 2, y - a / 2), a, a, color="#9b5d2e", zorder=3))
            ax.annotate(o["id"], (x, y), xytext=(4, 4), textcoords="offset points", fontsize=7, color="#5c3a1c")
    for name, xy, col in (("start", course.spec.get("start_pad", [0, 0, 0]), "#ffffff"), ("end", course.spec["end_pad"], "#ffe14d")):
        ax.add_patch(Rectangle((xy[0] * s - 1.2, xy[1] * s - 1.2), 2.4, 2.4, facecolor=col, edgecolor="#555555", zorder=2))
        ax.annotate(name, (xy[0] * s, xy[1] * s), ha="center", va="center", fontsize=7, zorder=4)
    for g in range(course.n):
        x, y, _, yw = course.gates[g]
        n = np.array([math.cos(yw), math.sin(yw)])
        lat = np.array([-math.sin(yw), math.cos(yw)])
        a, b = np.array([x, y]) - lat * course.outer / 2, np.array([x, y]) + lat * course.outer / 2
        ax.plot([a[0], b[0]], [a[1], b[1]], color="#ff7f0e", lw=5, solid_capstyle="butt", zorder=3)
        ax.annotate("", xy=(x + n[0] * 1.6, y + n[1] * 1.6), xytext=(x, y), arrowprops=dict(arrowstyle="->", color="#ff7f0e"), zorder=3)
        ax.annotate(course.ids[g], (x, y), xytext=(6, -12), textcoords="offset points", fontsize=8, weight="bold")
    ax.plot(plan[:, 0], plan[:, 1], color="#1f77b4", lw=1.2, ls="--", label="planned route", zorder=2)
    flown, = ax.plot([], [], color="#111111", lw=2.0, label="flown path (Gazebo truth)", zorder=5)
    dot, = ax.plot([], [], "o", color="#2ca02c", ms=9, zorder=6)
    head, = ax.plot([], [], "-", color="#2ca02c", lw=3, zorder=6)
    txt = ax.text(0.01, 0.99, "", transform=ax.transAxes, va="top", ha="left", fontsize=10, family="monospace", bbox=dict(fc="white", ec="#888888", alpha=0.9), zorder=10)
    ax.legend(loc="lower right", fontsize=8)
    ax.set_title(f"two laps of hard_race_v1 (scale {s:g}), top view, {SPEED:g}x speed")
    fig.tight_layout()
    fig.canvas.draw()
    w, h = fig.canvas.get_width_height()

    cmd = ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgba", "-s", f"{w}x{h}", "-r", str(FPS), "-i", "-",
           "-c:v", "libx264", "-preset", "veryfast", "-crf", "23", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(out_mp4)]
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    dt = SPEED / FPS
    n_frames = int((T[-1] - T[0]) / dt) + 1
    for k in range(n_frames):
        t = T[0] + k * dt
        i = min(int(np.searchsorted(T, t, side="right")) - 1, len(T) - 1)
        i = max(i, 0)
        flown.set_data(P[: i + 1, 0], P[: i + 1, 1])
        dot.set_data([P[i, 0]], [P[i, 1]])
        head.set_data([P[i, 0], P[i, 0] + 1.5 * math.cos(yaw[i])], [P[i, 1], P[i, 1] + 1.5 * math.sin(yaw[i])])
        c = int(counts[i])
        lap = min(c // course.n + 1, 2) if c < 2 * course.n else 2
        nxt = course.ids[c % course.n] if c < 2 * course.n else "done"
        txt.set_text(f"t = {t - T[0]:6.1f} s   z = {P[i, 2]:4.1f} m\nlap {lap} of 2   gates {c:2d} of {2 * course.n}\nnext gate: {nxt}   wrong: {sum(1 for w_ in lc.wrong if w_[0] <= t)}")
        fig.canvas.draw()
        proc.stdin.write(np.asarray(fig.canvas.buffer_rgba()).tobytes())
    proc.stdin.close()
    rc = proc.wait(timeout=600)
    plt.close(fig)
    return {"frames": n_frames, "fps": FPS, "speed": SPEED, "size": [w, h], "ffmpeg_exit": rc}
