"""SDF world of a race course for PX4 + Gazebo Harmonic: ground, gates (frame bars + post, plain colours), poles, a box, start and end pads.

Pure python (no numpy beyond what racecourse needs, no Gazebo import), so tests/test_world.py runs anywhere.
Frame: the world IS the course frame F (x east, y north, z up; origin = start pad = the spawn point; the ground top is z = 0).

The world has NO plugin on purpose: PX4's server.config (physics, sensors, scene broadcaster, ...) is only loaded when the world file
brings none. The runner writes this text over PX4's Tools/simulation/gz/worlds/baylands.sdf, so the make target
`gz_x500_mono_cam_baylands` (which exists after the normal build) starts it; the world keeps the name "baylands".

Solids use racecourse.route.gate_boxes, the same boxes the route checks use (frame bars DEPTH_M thick, a post POST_M square to the ground).
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))
from racecourse.course import Course  # noqa: E402
from racecourse.route import gate_boxes  # noqa: E402

WORLD = "baylands"
GATE_COLORS = ["1 0.45 0 1", "0.1 0.55 1 1", "0.9 0.1 0.6 1", "1 0.85 0 1", "0.2 0.8 0.3 1", "0.6 0.3 1 1", "0.1 0.9 0.9 1"]
POST_COLOR = "0.35 0.35 0.4 1"
POLE_COLOR = "0.9 0.1 0.1 1"
BOX_COLOR = "0.6 0.35 0.15 1"
START_PAD_COLOR = "1 1 1 1"
END_PAD_COLOR = "1 0.9 0.1 1"
GROUND_COLOR = "0.30 0.45 0.25 1"
LINE_COLOR = "0.80 0.80 0.75 1"
PAD_SIDE_M = 2.4          # side of a pad, m (the x500 is 0.65 m wide, its legs reach about 0.4 m from the centre)
LINE_STEP_M, LINE_HALF_M = 4.0, 40.0


def _material(color: str) -> str:
    return f"<material><ambient>{color}</ambient><diffuse>{color}</diffuse><emissive>{color}</emissive></material>"


def _box(name: str, centre, size, color: str, collision: bool = True) -> str:
    pose = f"<pose>{centre[0]:.5f} {centre[1]:.5f} {centre[2]:.5f} 0 0 0</pose>"
    geo = f"<geometry><box><size>{size[0]:.5f} {size[1]:.5f} {size[2]:.5f}</size></box></geometry>"
    s = f'<visual name="{name}_v">{pose}{geo}{_material(color)}</visual>'
    if collision:
        s += f'<collision name="{name}_c">{pose}{geo}</collision>'
    return s


def gate_model(course: Course, g: int) -> str:
    cx, cy, cz, yaw = course.gates[g]
    parts = []
    for k, (c, h) in enumerate(gate_boxes(course, g)):
        post = k == 4
        parts.append(_box("post" if post else f"bar{k}", c, [2 * v for v in h], POST_COLOR if post else GATE_COLORS[g % len(GATE_COLORS)]))
    return (f'<model name="gate_{course.ids[g]}"><static>true</static><pose>{cx:.5f} {cy:.5f} {cz:.5f} 0 0 {yaw:.6f}</pose>'
            f'<link name="link">{"".join(parts)}</link></model>')


def obstacle_models(course: Course) -> str:
    s, out = course.scale, []
    for o in course.spec.get("obstacles", []):
        x, y = (v * s for v in o["center_xy"])
        if o["kind"] == "vertical pole":
            r, h = o["radius"] * s, o["height"] * s
            geo = f"<geometry><cylinder><radius>{r:.5f}</radius><length>{h:.5f}</length></cylinder></geometry>"
            pose = f"<pose>0 0 {h / 2:.5f} 0 0 0</pose>"
            body = f'<visual name="v">{pose}{geo}{_material(POLE_COLOR)}</visual><collision name="c">{pose}{geo}</collision>'
        elif o["kind"].startswith("box"):
            a, h = o["size_xy"] * s, o["height"] * s
            body = _box("b", (0.0, 0.0, h / 2), (a, a, h), BOX_COLOR)
        else:
            continue
        out.append(f'<model name="{o["id"]}"><static>true</static><pose>{x:.5f} {y:.5f} 0 0 0 0</pose><link name="link">{body}</link></model>')
    return "".join(out)


def pad_models(course: Course) -> str:
    s = course.scale
    out = []
    for name, xy, color in (("start_pad", course.spec.get("start_pad", [0.0, 0.0, 0.0]), START_PAD_COLOR), ("end_pad", course.spec["end_pad"], END_PAD_COLOR)):
        x, y = xy[0] * s, xy[1] * s
        out.append(f'<model name="{name}"><static>true</static><pose>{x:.5f} {y:.5f} 0.01 0 0 0</pose><link name="link">'
                   f'{_box("p", (0, 0, 0), (PAD_SIDE_M, PAD_SIDE_M, 0.02), color, collision=False)}</link></model>')
    return "".join(out)


def ground_models() -> str:
    """Collision box (top at z = 0), a green visual on it and thin light lines every LINE_STEP_M so that the camera sees the motion."""
    side = 2 * LINE_HALF_M + 20.0
    parts = ['<model name="ground_collision"><static>true</static><pose>0 0 -1 0 0 0</pose><link name="link">'
             '<collision name="c"><geometry><box><size>6000 6000 2</size></box></geometry><surface><friction><ode/></friction></surface></collision></link></model>',
             f'<model name="ground_look"><static>true</static><pose>0 0 -0.005 0 0 0</pose><link name="link">'
             f'{_box("g", (0, 0, 0), (side, side, 0.01), GROUND_COLOR, collision=False)}</link></model>']
    lines, n = [], int(LINE_HALF_M // LINE_STEP_M)
    for k in range(-n, n + 1):
        v = k * LINE_STEP_M
        lines.append(_box(f"lx{k + n}", (v, 0, 0.002), (0.06, 2 * LINE_HALF_M, 0.004), LINE_COLOR, collision=False))
        lines.append(_box(f"ly{k + n}", (0, v, 0.002), (2 * LINE_HALF_M, 0.06, 0.004), LINE_COLOR, collision=False))
    parts.append(f'<model name="ground_lines"><static>true</static><pose>0 0 0 0 0 0</pose><link name="link">{"".join(lines)}</link></model>')
    return "".join(parts)


def _light(name: str, direction, diffuse) -> str:
    return (f"<light name='{name}' type='directional'><pose>0 0 500 0 0 0</pose><cast_shadows>false</cast_shadows><intensity>1</intensity>"
            f"<direction>{direction}</direction><diffuse>{diffuse} 1</diffuse><specular>0.3 0.3 0.3 1</specular>"
            "<attenuation><range>2000</range><linear>0</linear><constant>1</constant><quadratic>0</quadratic></attenuation></light>")


def world_sdf(course: Course, name: str = WORLD) -> str:
    models = ground_models() + "".join(gate_model(course, g) for g in range(course.n)) + obstacle_models(course) + pad_models(course)
    return ("<?xml version='1.0'?><sdf version='1.9'><world name='" + name + "'>"
            "<physics type='ode'><max_step_size>0.004</max_step_size><real_time_factor>1.0</real_time_factor><real_time_update_rate>250</real_time_update_rate></physics>"
            "<gravity>0 0 -9.8</gravity><magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field><atmosphere type='adiabatic'/>"
            "<scene><ambient>0.35 0.36 0.40 1</ambient><background>0.55 0.72 0.95 1</background><grid>false</grid><shadows>0</shadows></scene>"
            "<wind><linear_velocity>0 0 0</linear_velocity></wind>"
            + _light("sun", "-0.4 0.3 -0.85", "1 0.95 0.85") + _light("fill", "0.55 0.25 -0.8", "0.45 0.5 0.6")
            + models +
            "<spherical_coordinates><surface_model>EARTH_WGS84</surface_model><world_frame_orientation>ENU</world_frame_orientation>"
            "<latitude_deg>47.397971</latitude_deg><longitude_deg>8.546164</longitude_deg><elevation>0</elevation></spherical_coordinates>"
            "</world></sdf>")


if __name__ == "__main__":  # python -m sim.gz_px4.world tracks/hard_race_v1.json out.sdf
    c = Course.load(sys.argv[1], 4.0)
    Path(sys.argv[2]).write_text(world_sdf(c), encoding="utf-8")
