"""Offline check of sim/gz_px4/analysis.py: the planned two-lap route, flown perfectly and landed on the end pad, passes every rule;
a landing 3 m off the pad and a flight that stays in the air fail the landing rule; deviation and tilt are measured.
Run: python tests/test_analysis.py"""

import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
from racecourse.course import Course  # noqa: E402
from racecourse.route import Route  # noqa: E402
from sim.gz_px4.analysis import analyse  # noqa: E402

passed = failed = 0


def check(name, ok, detail=""):
    global passed, failed
    if ok:
        passed += 1
    else:
        failed += 1
        print(f"FAIL {name} {detail}")


def main():
    course = Course.load(REPO / "tracks" / "hard_race_v1.json", 4.0)
    route = Route(course, laps=2)
    ts, P = route.sample(0.02)
    end = np.array(course.spec["end_pad"][:2]) * course.scale
    down = np.linspace(1.0, 0.0, 20)[:, None]
    tail = np.hstack([np.tile(P[-1, :2], (20, 1)), (P[-1, 2] * down)])
    Pf = np.vstack([P, tail])
    Tf = np.concatenate([ts, ts[-1] + 0.02 * np.arange(1, 21)])
    Q = np.tile([0.0, 0.0, 0.0, 1.0], (len(Tf), 1))
    r = analyse(course, route, Tf, Pf, Q, 0.0, end, True)
    check("perfect flight: gates 14", r["gates_passed"] == 14, r["gates_passed"])
    check("perfect flight: 2 lap times", len(r["lap_times_s"]) == 2, r["lap_times_s"])
    check("perfect flight: 0 wrong, 0 backward", not r["wrong_crossings"] and not r["backward_crossings"], (r["wrong_crossings"][:2], r["backward_crossings"][:2]))
    check("perfect flight: deviation ~ 0", r["max_distance_from_planned_path_m"] < 1e-6, r["max_distance_from_planned_path_m"])
    check("perfect flight: lands on the pad", r["landing_distance_from_end_pad_m"] < 1e-6 and r["landed"])
    check("perfect flight: passed", r["passed"], r["checks"])
    check("perfect flight: tilt 0", r["max_tilt_deg"] < 1e-6)

    # a landing 3 m off the pad
    P2 = Pf.copy()
    P2[-20:, 0] += 3.0
    r2 = analyse(course, route, Tf, P2, Q, 0.0, end, True)
    check("landing 3 m off fails", not r2["passed"] and not r2["checks"]["landed within 1.0 m of the end pad centre"], r2["landing_distance_from_end_pad_m"])

    # not landed (still in the air)
    r3 = analyse(course, route, Tf, Pf[:-20], Q[:-20], 0.0, end, False)
    check("not landed fails", not r3["passed"] and not r3["landed"])

    # a flight that flies the path a metre away from the planned path but still passes: deviation is measured
    P4 = Pf.copy()
    P4[:, 2] += 0.5
    r4 = analyse(course, route, Tf, P4, Q, 0.0, end, True)
    check("deviation measured", 0.4 < r4["max_distance_from_planned_path_m"] < 0.6, r4["max_distance_from_planned_path_m"])

    # tilt
    Q5 = Q.copy()
    Q5[10] = [np.sin(np.radians(20)), 0.0, 0.0, np.cos(np.radians(20))]   # 40 degree roll
    r5 = analyse(course, route, Tf, Pf, Q5, 0.0, end, True)
    check("tilt 40 deg", abs(r5["max_tilt_deg"] - 40.0) < 1e-6, r5["max_tilt_deg"])

    print(f"{passed} of {passed + failed} passed")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
