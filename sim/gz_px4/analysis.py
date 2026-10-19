"""Numbers of a flight, from the ground-truth path in the course frame. Pure numpy/scipy (no simulator), tested by tests/test_analysis.py.

analyse(course, route, T, P, Q, t_route_start, end_pad, landed) -> dict with the pass numbers of result.json.
  T (n,) seconds, P (n, 3) positions in the course frame, Q (n, 4) quaternions x, y, z, w.
"""

from __future__ import annotations

import math

import numpy as np
from scipy.spatial import cKDTree

from racecourse.course import Course, LapCounter
from racecourse.route import Route, backward_crossings, distances

DRONE_RADIUS = 0.37
CLEAR_MIN = DRONE_RADIUS + 0.4      # 0.77 m, the margin of the route checks
LAND_RADIUS = 1.0


def tilt_deg(Q) -> np.ndarray:
    Q = np.asarray(Q, float).reshape(-1, 4)
    return np.degrees(np.arccos(np.clip(1.0 - 2.0 * (Q[:, 0] ** 2 + Q[:, 1] ** 2), -1.0, 1.0)))


def counter_replay(course: Course, laps: int, T, P, t_start):
    """Replay a path through a fresh LapCounter. Returns (counter, count after every sample)."""
    lc = LapCounter(course, laps=laps, t_start=t_start)
    counts = np.zeros(len(T), dtype=np.int64)
    for i, (t, p) in enumerate(zip(T, P)):
        counts[i] = lc.update(float(t), p)
    return lc, counts


def analyse(course: Course, route: Route, T, P, Q, t_route_start: float, end_pad, landed: bool, laps: int = 2) -> dict:
    T, P, Q = np.asarray(T, float), np.asarray(P, float), np.asarray(Q, float)
    lc, _ = counter_replay(course, laps, T, P, t_route_start)
    back = backward_crossings(course, P, margin=CLEAR_MIN)
    mins = {"gate": (math.inf, None), "pole": (math.inf, None), "box": (math.inf, None)}
    for p in P:
        for k, v in distances(course, p).items():
            if v[0] < mins[k][0]:
                mins[k] = v
    ts, plan = route.sample(0.02)
    tree = cKDTree(plan)
    after = (T >= t_route_start) & (T <= t_route_start + route.duration)
    dev = tree.query(P[after])[0] if after.any() else np.array([math.nan])
    end = np.asarray(end_pad, float)
    final = P[-1]
    land_d = float(math.hypot(final[0] - end[0], final[1] - end[1]))
    n_total = laps * course.n
    out = {
        "gates_passed": int(lc.count), "gates_total": n_total,
        "passage_log": [{"t": float(a), "count": int(b), "gate": c, "ly": float(d), "lz": float(e)} for a, b, c, d, e in lc.log],
        "lap_times_s": [float(x) for x in lc.lap_times],
        "wrong_crossings": [{"t": float(a), "gate": b, "ly": float(c), "lz": float(d)} for a, b, c, d in lc.wrong],
        "backward_crossings": [{"sample": i, "t": float(T[i]), "gate": g, "ly": ly, "lz": lz} for i, g, ly, lz in back],
        "backward_margin_m": CLEAR_MIN,
        "min_clearance_m": {k: {"distance_m": float(v[0]), "id": v[1]} for k, v in mins.items()},
        "max_distance_from_planned_path_m": float(np.nanmax(dev)),
        "max_tilt_deg": float(tilt_deg(Q).max()),
        "landing_distance_from_end_pad_m": land_d,
        "landed": bool(landed and final[2] < 0.3),
        "final_position": [float(v) for v in final],
    }
    checks = {
        "14 of 14 gates in order": lc.count == n_total,
        "2 lap times": len(lc.lap_times) == laps,
        "0 wrong crossings": len(lc.wrong) == 0,
        "0 backward crossings": len(back) == 0,
        "min clearance to every solid > 0.77 m": all(v[0] > CLEAR_MIN for v in mins.values()),
        "landed within 1.0 m of the end pad centre": out["landed"] and land_d <= LAND_RADIUS,
    }
    out["checks"] = checks
    out["passed"] = all(checks.values())
    return out
