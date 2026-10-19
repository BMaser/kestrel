"""Checks of racecourse/course.py (no simulator): gates counted in order, over two laps, with the right lap times; nothing counted
outside the opening, backwards, out of order or after the race.
Run: python tests/test_course.py"""

import math
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
from racecourse.course import Course, LapCounter  # noqa: E402

passed = failed = 0


def check(name, ok, detail=""):
    global passed, failed
    if ok:
        passed += 1
    else:
        failed += 1
        print(f"FAIL {name} {detail}")


# Two gates, scale 4 (opening 2 m): G1 at (1, 0, 1) passed towards +x, G2 at (1, 4, 1) passed towards -x.
TWO = {"gate_opening_m": 0.5, "gate_frame_outer_m": 0.7,
       "gates": [{"id": "G1", "center": [0.25, 0.0, 0.25], "yaw_deg": 0}, {"id": "G2", "center": [0.25, 1.0, 0.25], "yaw_deg": 180}]}
RECT = [(0, 0), (2, 0), (2, 4), (0, 4), (0, 0)]  # counter-clockwise, 12 m; G1 at 1 m, G2 at 7 m along it


def fly(counter, corners, laps, y_shift=0.0, z=1.0, speed=1.0, dt=0.01, t0=0.0):
    """Fly the closed polyline `laps` times at `speed`, updating the counter every dt; returns the end time."""
    pts = np.array(corners, float)
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    t = t0
    for i in range(int(round(laps * cum[-1] / (speed * dt))) + 1):
        s = (i * speed * dt) % cum[-1]
        k = min(int(np.searchsorted(cum, s, side="right") - 1), len(seg) - 1)
        p = pts[k] + (s - cum[k]) / seg[k] * (pts[k + 1] - pts[k])
        t = t0 + i * dt
        counter.update(t, (p[0], p[1] + y_shift, z))
    return t


def main():
    two = Course(TWO, scale=4.0)
    check("scale: opening 2 m, G2 centre (1, 4, 1)", abs(two.opening - 2.0) < 1e-12 and np.allclose(two.gates[1, :3], [1, 4, 1]))

    # ---- two laps: 4 passages, lap times 7 s and 12 s, nothing wrong
    lc = LapCounter(two, laps=2, t_start=0.0)
    fly(lc, RECT, laps=2)
    check("two laps: 4 passages", lc.count == 4, lc.count)
    check("two laps: done, 2 laps", lc.done and lc.lap == 2 and lc.next_gate is None)
    lt = lc.lap_times
    check("two laps: lap times 7 s and 12 s", len(lt) == 2 and abs(lt[0] - 7.0) < 0.02 and abs(lt[1] - 12.0) < 0.02, lt)
    check("two laps: passage times 1, 7, 13, 19 s", [round(e[0]) for e in lc.log] == [1, 7, 13, 19], [e[0] for e in lc.log])
    check("two laps: no wrong crossing", lc.wrong == [], lc.wrong)

    # ---- a third lap after the race: not counted, logged as wrong
    lc3 = LapCounter(two, laps=2, t_start=0.0)
    fly(lc3, RECT, laps=3)
    check("after the race: count stays 4", lc3.count == 4, lc3.count)
    check("after the race: G1 and G2 crossings logged as wrong", [w[1] for w in lc3.wrong] == ["G1", "G2"], lc3.wrong)

    # ---- one lap = the old counter: stops at the number of gates
    lc1 = LapCounter(two, laps=1)
    fly(lc1, RECT, laps=2)
    check("one lap: count stops at 2", lc1.count == 2 and lc1.lap == 1, lc1.count)
    check("one lap: lap 1 timed from the first update", len(lc1.lap_times) == 1 and abs(lc1.lap_times[0] - 7.0) < 0.02, lc1.lap_times)

    # ---- outside the opening (1.1 m beside the centre, half opening 1 m): nothing counted
    lco = LapCounter(two, laps=1)
    fly(lco, RECT, laps=1, z=2.1)
    check("outside the opening: 0 passages", lco.count == 0, lco.count)

    # ---- backwards (clockwise): G1 is crossed towards -x, which is not a crossing
    lcb = LapCounter(two, laps=1)
    fly(lcb, RECT[::-1], laps=1)
    check("backwards: 0 passages, nothing wrong", lcb.count == 0 and lcb.wrong == [], (lcb.count, lcb.wrong))

    # ---- out of order: G1 missed (bottom edge 1.5 m to the side), G2 flown through -> logged as wrong, not counted
    lcm = LapCounter(two, laps=1)
    shifted = [(0, -1.5), (2, -1.5), (2, 4), (0, 4), (0, -1.5)]
    fly(lcm, shifted, laps=1)
    check("out of order: 0 passages", lcm.count == 0, lcm.count)
    check("out of order: G2 logged as wrong", [w[1] for w in lcm.wrong] == ["G2"], lcm.wrong)

    # ---- the real route file at scale 4, two laps through points 1 m before and after every gate
    hr = Course.load(REPO / "tracks" / "hard_race_v1.json", scale=4.0)
    check("hard_race_v1: 7 gates, G1 at (6, 0, 3.2)", hr.n == 7 and np.allclose(hr.gates[0, :3], [6.0, 0.0, 3.2]))
    pts = []
    for _ in range(2):
        for g in range(hr.n):
            c, R = hr.frame(g)
            pts += [c - 1.0 * R[:, 0], c + 1.0 * R[:, 0]]
    lch = LapCounter(hr, laps=2, t_start=0.0)
    t = 0.0
    for a, b in zip(pts, pts[1:]):
        for u in np.linspace(0.0, 1.0, 50, endpoint=False):
            lch.update(t, a + u * (b - a))
            t += 0.01
    lch.update(t, pts[-1])
    check("hard_race_v1: 14 passages in order over two laps", lch.count == 14 and [e[2] for e in lch.log] == hr.ids * 2, [e[2] for e in lch.log])
    check("hard_race_v1: two lap times that add up to the end of lap 2",
          len(lch.lap_times) == 2 and math.isclose(sum(lch.lap_times), lch.lap_ends[-1] - lch.t_start), lch.lap_times)

    print(f"{passed} of {passed + failed} passed")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
