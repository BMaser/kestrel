"""A race course that does not depend on any simulator: the gates of a route file, their frames, and a lap counter that counts
the gates passed in order over several laps and times each lap. The same rule decides "gate passed" in every simulator we use
(PX4 + Gazebo now, Project AirSim for the realism stage, and the camera-to-control loop later).

Route file format: tracks/hard_race_v1.json. Frame: x forward from the start pad, y to the left, z up, in metres at scale 1;
every coordinate and size is multiplied by `scale` (4 for the x500, about 0.65 m wide). A gate's yaw is the direction in which
the drone must pass through it (0 = +x, counter-clockwise positive). Pure numpy, so the tests run anywhere.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np


class Course:
    def __init__(self, spec: dict, scale: float = 1.0):
        self.spec, self.scale = spec, float(scale)
        self.opening = spec["gate_opening_m"] * self.scale
        self.outer = spec["gate_frame_outer_m"] * self.scale
        self.ids = [g["id"] for g in spec["gates"]]
        self.gates = np.array([[*(np.asarray(g["center"], float) * self.scale), math.radians(g["yaw_deg"])] for g in spec["gates"]])

    @classmethod
    def load(cls, path, scale: float = 1.0) -> "Course":
        return cls(json.loads(Path(path).read_text(encoding="utf-8")), scale)

    @property
    def n(self) -> int:
        return len(self.ids)

    def frame(self, g: int):
        """(centre, R): R's columns are the gate's x (passage direction), y (left) and z (up) axes in the course frame."""
        x, y, z, yaw = self.gates[g]
        c, s = math.cos(yaw), math.sin(yaw)
        return np.array([x, y, z]), np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])

    def local(self, g: int, p) -> np.ndarray:
        """Point p (course frame) in gate g's own frame."""
        c, R = self.frame(g)
        return R.T @ (np.asarray(p, float) - c)


class LapCounter:
    """Counts gate passages in order over `laps` laps.

    A passage = the drone's centre goes from the back (local x < 0) to the front (local x >= 0) of the NEXT gate of the order
    G1..Gn, with |local y| and |local z| below half the opening where it crosses the gate plane. After the last gate the next one
    is G1 of the next lap, until `laps` laps are done; later crossings are not counted.
    Any other crossing of a gate plane inside the opening, or within `near_m` of the gate centre, goes to `wrong`
    (out of order, backwards is not a crossing at all, or after the race).

    count      gates passed in order over all laps
    log        (t, count, gate id, local y, local z) per counted passage
    wrong      (t, gate id, local y, local z)
    lap_times  time of each finished lap: lap 1 from `t_start` (default: the time of the first update), each later lap from the
               end of the one before
    """

    def __init__(self, course: Course, laps: int = 1, t_start: float | None = None, start_at: int = 0, near_m: float = 1.0):
        self.c, self.laps, self.t_start, self.near_m = course, int(laps), t_start, near_m
        self.count, self.prev, self.log, self.wrong, self.lap_ends = start_at, None, [], [], []

    @property
    def done(self) -> bool:
        return self.count >= self.laps * self.c.n

    @property
    def next_gate(self):
        """Index of the gate to pass next, or None when the race is done."""
        return None if self.done else self.count % self.c.n

    @property
    def lap(self) -> int:
        """Laps finished."""
        return min(self.count // self.c.n, self.laps)

    @property
    def lap_times(self) -> list:
        if not self.lap_ends:
            return []
        ends = [self.t_start] + self.lap_ends
        return [b - a for a, b in zip(ends, ends[1:])]

    def update(self, t: float, p) -> int:
        p = np.asarray(p, float)
        if self.t_start is None:
            self.t_start = float(t)
        if self.prev is not None:
            for g in range(self.c.n):
                a, b = self.c.local(g, self.prev), self.c.local(g, p)
                if a[0] < 0.0 <= b[0]:
                    u = -a[0] / (b[0] - a[0]) if b[0] != a[0] else 0.0
                    ly, lz = a[1] + u * (b[1] - a[1]), a[2] + u * (b[2] - a[2])
                    inside = abs(ly) < self.c.opening / 2 and abs(lz) < self.c.opening / 2
                    if g == self.next_gate and inside:
                        self.count += 1
                        self.log.append((t, self.count, self.c.ids[g], ly, lz))
                        if self.count % self.c.n == 0:
                            self.lap_ends.append(float(t))
                    elif inside or math.hypot(ly, lz) < self.near_m:
                        self.wrong.append((t, self.c.ids[g], ly, lz))
        self.prev = p
        return self.count
