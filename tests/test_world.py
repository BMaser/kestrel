"""Offline check of sim/gz_px4/world.py: the SDF is well-formed XML and has 7 gates x 5 boxes (4 frame bars + 1 post), the poles and the box,
two pads and a ground; every gate sits at its course position. Run: python tests/test_world.py"""

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
from racecourse.course import Course  # noqa: E402
from sim.gz_px4.world import world_sdf  # noqa: E402

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
    text = world_sdf(course)
    root = ET.fromstring(text)
    world = root.find("world")
    check("one world", world is not None and world.get("name") == "baylands")
    check("no plugin in the world file (PX4 server.config must load)", world.find("plugin") is None)
    models = {m.get("name"): m for m in world.findall("model")}
    gates = [m for n, m in models.items() if n.startswith("gate_")]
    check("7 gates", len(gates) == 7, len(gates))
    for g in gates:
        link = g.find("link")
        check(f"{g.get('name')} has 5 boxes with collision", len(link.findall("collision")) == 5 and len(link.findall("visual")) == 5)
        check(f"{g.get('name')} is static", g.find("static").text == "true")
    for i, gid in enumerate(course.ids):
        pose = [float(v) for v in models[f"gate_{gid}"].find("pose").text.split()]
        want = [course.gates[i, 0], course.gates[i, 1], course.gates[i, 2], 0.0, 0.0, course.gates[i, 3]]
        check(f"{gid} pose", all(abs(a - b) < 1e-4 for a, b in zip(pose, want)), pose)
    poles = [m for n, m in models.items() if n.startswith("pole_")]
    check("2 poles", len(poles) == 2, len(poles))
    for p in poles:
        r = float(p.find("link/collision/geometry/cylinder/radius").text)
        check(f"{p.get('name')} radius 0.05 x 4", abs(r - 0.2) < 1e-6, r)
    check("1 box", "box_1" in models and len(models["box_1"].findall("link/collision")) == 1)
    check("start and end pad", "start_pad" in models and "end_pad" in models)
    ep = [float(v) for v in models["end_pad"].find("pose").text.split()[:2]]
    check("end pad position", abs(ep[0] + 1.2) < 1e-4 and abs(ep[1] - 8.0) < 1e-4, ep)
    check("ground collision", "ground_collision" in models)
    print(f"{passed} of {passed + failed} passed")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
