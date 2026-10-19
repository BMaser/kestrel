"""Checks of racecourse/route.py (no simulator) on tracks/hard_race_v1.json at scale 4 (the x500):
one lap and two laps pass every gate in order, keep the drone radius plus 0.4 m from every solid, never cross a gate backwards near
its frame, and stay gentle; a lap junction that goes straight from G7 to G1 (the 2026-10-18 mistake) must FAIL these checks.
Run: python tests/test_route.py"""

import copy
import math
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
from racecourse.course import Course, LapCounter  # noqa: E402
from racecourse.route import Route, backward_crossings, distances  # noqa: E402

DRONE_RADIUS = 0.37   # x500: motor 0.25 m from the centre plus a 0.12 m propeller
CLEAR = DRONE_RADIUS + 0.4
passed = failed = 0


def check(name, ok, detail=""):
    global passed, failed
    if ok:
        passed += 1
    else:
        failed += 1
        print(f"FAIL {name} {detail}")


def survey(course, laps):
    r = Route(course, laps=laps)
    ts, P = r.sample(0.02)
    lc = LapCounter(course, laps=laps, t_start=0.0)
    mins = {"gate": (9e9, None), "pole": (9e9, None), "box": (9e9, None)}
    for t, p in zip(ts, P):
        lc.update(t, p)
        for k, v in distances(course, p).items():
            if v[0] < mins[k][0]:
                mins[k] = v
    back = backward_crossings(course, P, margin=CLEAR)
    acc = float(np.linalg.norm(r.spline(ts, 2), axis=1).max())
    return r, P, lc, mins, back, acc


def main():
    hr = Course.load(REPO / "tracks" / "hard_race_v1.json", scale=4.0)
    for laps in (1, 2):
        r, P, lc, mins, back, acc = survey(hr, laps)
        tag = f"{laps} lap(s)"
        check(f"{tag}: {7 * laps} passages in order", lc.count == 7 * laps and [e[2] for e in lc.log] == hr.ids * laps, [e[2] for e in lc.log])
        check(f"{tag}: no wrong crossing", lc.wrong == [], lc.wrong[:3])
        check(f"{tag}: {laps} lap time(s)", len(lc.lap_times) == laps, lc.lap_times)
        check(f"{tag}: no backward crossing within the frame plus {CLEAR:.2f} m", back == [], back[:3])
        for k, (d, who) in mins.items():
            check(f"{tag}: clearance to {k} above the drone radius plus 0.4 m", d > CLEAR, f"{d:.3f} at {who}")
        check(f"{tag}: heights 2 to 7 m", P[:, 2].min() > 2.0 and P[:, 2].max() < 7.0, f"{P[:, 2].min():.2f} {P[:, 2].max():.2f}")
        check(f"{tag}: acceleration below 2.5 m/s^2", acc < 2.5, f"{acc:.2f}")
        end = np.asarray(hr.spec["end_pad"][:2]) * hr.scale
        check(f"{tag}: ends over the end pad", math.hypot(P[-1, 0] - end[0], P[-1, 1] - end[1]) < 0.05)
        p0, v0 = r.pos_vel(-1.0)
        p1, v1 = r.pos_vel(r.duration + 1.0)
        check(f"{tag}: at rest outside the route", not v0.any() and not v1.any() and np.allclose(p1, r.wp[-1]))
        print(f"{tag}: route {r.duration:.1f} s, lap times {[round(x, 1) for x in lc.lap_times]}, "
              f"clearance gate {mins['gate'][0]:.2f} ({mins['gate'][1]}), pole {mins['pole'][0]:.2f}, box {mins['box'][0]:.2f}, acc {acc:.2f}")

    # ---- the mutant: straight from the point after G7 to the point before G1 (only its midpoint as the junction)
    bad = copy.deepcopy(hr.spec)
    bad["lap_junction"]["points"] = [[0.65, 0.77, 0.85]]
    _, _, lcb, minsb, backb, _ = survey(Course(bad, 4.0), 2)
    caught = bool(backb) or minsb["gate"][0] <= CLEAR or lcb.count != 14
    check("mutant (straight junction through G7): the checks catch it", caught,
          f"back {backb[:2]} gate {minsb['gate']} count {lcb.count}")
    print(f"mutant: backward {backb[:2]}, gate clearance {minsb['gate'][0]:.2f} ({minsb['gate'][1]}), passages {lcb.count}")

    # ---- a second lap needs a junction
    nojunction = copy.deepcopy(hr.spec)
    del nojunction["lap_junction"]
    try:
        Route(Course(nojunction, 4.0), laps=2)
        check("no junction: two laps refused", False)
    except ValueError:
        check("no junction: two laps refused", True)

    print(f"{passed} of {passed + failed} passed")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
