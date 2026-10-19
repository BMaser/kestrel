"""Two laps of the hard race course in PX4 SITL + Gazebo Harmonic (x500 with ONE forward nose camera), flown with MAVSDK offboard
position + velocity setpoints, with the live lap counter, the films and result.json.

  python -m sim.gz_px4.fly --px4 px4 --out results          # from the repository root, on the GitHub runner

Needs (see .github/workflows/gz-two-laps.yml): the PX4-Autopilot checkout built with `make px4_sitl_default`, Gazebo Harmonic with its python
bindings, xvfb, Mesa, ffmpeg, mavsdk (python), numpy, scipy, matplotlib. NOTHING of this runs on the owner's PC.

Frame F = course frame = Gazebo world (x east, y north, z up, origin = spawn point = start pad). PX4 local NED from F: north = y, east = x,
down = -z; PX4 yaw (deg from north, clockwise) = 90 - heading in F. PX4's local origin can sit a few cm off the Gazebo origin (its
estimate starts at the spawn height of the flight controller): the offset between PX4's position and the Gazebo truth is measured once in the
hover before the route and applied to every setpoint and to the live counter. Time is Gazebo simulation time everywhere.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import os
import re
import signal
import subprocess
import sys
import threading
import time
import traceback
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))
from racecourse.course import Course, LapCounter  # noqa: E402
from racecourse.route import Route, distances  # noqa: E402
from sim.gz_px4 import analysis as AN  # noqa: E402
from sim.gz_px4 import nose_yaw as ny  # noqa: E402
from sim.gz_px4 import world as W  # noqa: E402

TRACK = REPO / "tracks" / "hard_race_v1.json"
SCALE = 4.0
LAPS = 2
MODEL = "x500_mono_cam_0"
PX4_WORLD = "Tools/simulation/gz/worlds/baylands.sdf"
PX4_MODELS = "Tools/simulation/gz/models"
WIDTH, HEIGHT = 640, 480
FILM_FPS = 15
SENSOR_WIDTH_FROM, SENSOR_HEIGHT_FROM = "1280", "960"
CHASE_BACK_M, CHASE_UP_M = 2.5, 1.0
STACK_START_TIMEOUT_S = 12 * 60
FLIGHT_TIMEOUT_S = 28 * 60
SIM_DEAD_WALL_S = 90.0
HOLD_START_S, HOLD_END_S = 2.0, 3.0
CLIMB_RATE, DESCENT_RATE, LAND_HANDOVER_M = 0.8, 0.6, 1.0
YAW_RATE_MAX, V_MIN, PLAN_DT = 1.0, 0.3, 0.05
POSE_DT = 0.02

CHASE_LINK = f"""
    <link name="chase_cam_link">
      <pose relative_to="base_link">-{CHASE_BACK_M} 0 {CHASE_UP_M} 0 {math.atan2(CHASE_UP_M, CHASE_BACK_M):.4f} 0</pose>
      <gravity>false</gravity>
      <inertial><mass>0.001</mass><inertia><ixx>1e-7</ixx><iyy>1e-7</iyy><izz>1e-7</izz></inertia></inertial>
      <sensor name="chase_cam" type="camera">
        <gz_frame_id>chase_cam_link</gz_frame_id>
        <camera>
          <horizontal_fov>1.4</horizontal_fov>
          <image><width>{WIDTH}</width><height>{HEIGHT}</height></image>
          <clip><near>0.1</near><far>3000</far></clip>
        </camera>
        <always_on>1</always_on>
        <update_rate>30</update_rate>
        <visualize>false</visualize>
      </sensor>
    </link>
    <joint name="ChaseCamJoint" type="fixed"><parent>base_link</parent><child>chase_cam_link</child></joint>
"""


def sh(cmd, timeout=30):
    try:
        p = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=timeout)
        return p.stdout + p.stderr
    except subprocess.TimeoutExpired:
        return ""


def log(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


# ------------------------------------------------------------------------------------------------ files in the PX4 checkout
def prepare_files(px4: Path, out: Path, course: Course) -> dict:
    """Our world over PX4's baylands.sdf (its make target exists after the normal build), the nose camera at 640 x 480, a chase camera."""
    info = {}
    wp = px4 / PX4_WORLD
    orig = wp.with_suffix(".sdf.orig")
    if not orig.exists():
        orig.write_text(wp.read_text(encoding="utf-8"), encoding="utf-8")
    text = W.world_sdf(course)
    wp.write_text(text, encoding="utf-8")
    (out / "world_used.sdf").write_text(text, encoding="utf-8")
    info["world_bytes"] = len(text)
    models = px4 / PX4_MODELS
    mono, x500 = models / "mono_cam" / "model.sdf", models / "x500_mono_cam" / "model.sdf"
    t = mono.read_text()
    t2 = t.replace(f"<width>{SENSOR_WIDTH_FROM}</width>", f"<width>{WIDTH}</width>").replace(f"<height>{SENSOR_HEIGHT_FROM}</height>", f"<height>{HEIGHT}</height>")
    info["mono_cam_changed"] = t2 != t
    mono.write_text(t2)
    t = x500.read_text()
    if "chase_cam_link" not in t:
        k = t.rindex("</model>")
        t = t[:k] + CHASE_LINK + t[k:]
    x500.write_text(t)
    info["x500_mono_cam_has_chase"] = "chase_cam_link" in x500.read_text()
    log("prepared files: " + json.dumps(info))
    return info


# ------------------------------------------------------------------------------------------------ shared state, gz transport, films
class Shared:
    def __init__(self):
        self.lock = threading.Lock()
        self.sim_t = None
        self.stats_t = None
        self.pose_n = 0
        self.stamp_zero = 0
        self.pose = None             # latest (x, y, z, qx, qy, qz, qw), world = F
        self.poses = []              # (t, x, y, z, qx, qy, qz, qw) every POSE_DT while recording
        self.recording = False
        self.films = {}
        self.node = None
        self.cam_topics = {}
        self.image_problems = []

    def now(self):
        with self.lock:
            return self.sim_t


class FilmPipe:
    """Camera pictures straight into an ffmpeg process (libx264). Slots of 1/FILM_FPS simulated seconds: a slot with no new picture repeats
    the last one, so the film runs in simulation time. Counts every picture it gets (n_seen) to give the achieved camera rate."""

    def __init__(self, path: Path):
        self.path = path
        cmd = ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{WIDTH}x{HEIGHT}", "-r", str(FILM_FPS), "-i", "-",
               "-c:v", "libx264", "-preset", "ultrafast", "-crf", "23", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(path)]
        self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
        self.lock = threading.Lock()
        self.active, self.broken = False, False
        self.n_seen = self.n_seen_active = self.n_written = 0
        self.t_first = self.t_last = self.next_t = self.cur = None

    def _write(self, data):
        if self.broken:
            return
        try:
            self.proc.stdin.write(data)
            self.n_written += 1
        except (BrokenPipeError, OSError, ValueError):
            self.broken = True

    def add(self, t, data: bytes):
        with self.lock:
            self.n_seen += 1
            if not self.active:
                return
            self.n_seen_active += 1
            if self.t_first is None:
                self.t_first, self.next_t = t, t
            self.t_last = t
            k = 0
            while self.cur is not None and self.next_t < t - 1e-9 and k < 900:
                self._write(self.cur)
                self.next_t += 1.0 / FILM_FPS
                k += 1
            if self.cur is None or k >= 900:
                self.next_t = t
            self.cur = data

    def close(self) -> dict:
        with self.lock:
            self.active = False
            if self.cur is not None:
                self._write(self.cur)
            try:
                self.proc.stdin.close()
            except OSError:
                pass
        try:
            rc = self.proc.wait(timeout=300)
        except subprocess.TimeoutExpired:
            self.proc.kill()
            rc = -9
        span = (self.t_last - self.t_first) if self.t_first is not None and self.t_last is not None else 0.0
        return {"film": self.path.name, "pictures_seen_while_recording": self.n_seen_active, "pictures_seen_total": self.n_seen, "frames_written": self.n_written,
                "span_sim_s": span, "camera_fps_achieved": (self.n_seen_active - 1) / span if span > 0 and self.n_seen_active > 1 else None,
                "ffmpeg_exit": rc, "pipe_broken": self.broken}


def stamp_seconds(header, fallback):
    t = header.stamp.sec + header.stamp.nsec * 1e-9
    return t if t > 0 else fallback


def setup_gz(S: Shared, world: str):
    from gz.msgs10.image_pb2 import Image
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.msgs10.world_stats_pb2 import WorldStatistics
    from gz.transport13 import Node

    S.Image = Image
    node = Node()
    S.node = node

    def on_stats(msg):
        with S.lock:
            S.stats_t = msg.sim_time.sec + msg.sim_time.nsec * 1e-9

    def on_pose(msg):
        t = stamp_seconds(msg.header, S.stats_t)
        if t is None:
            return
        for p in msg.pose:
            if p.name == MODEL:
                row = (t, p.position.x, p.position.y, p.position.z, p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
                with S.lock:
                    S.pose_n += 1
                    if msg.header.stamp.sec == 0 and msg.header.stamp.nsec == 0:
                        S.stamp_zero += 1
                    S.sim_t = t
                    S.pose = row[1:]
                    if S.recording and (not S.poses or t - S.poses[-1][0] >= POSE_DT - 1e-9):
                        S.poses.append(row)
                break

    ok = [node.subscribe(WorldStatistics, f"/world/{world}/stats", on_stats), node.subscribe(Pose_V, f"/world/{world}/dynamic_pose/info", on_pose)]
    log(f"subscribed to stats and dynamic_pose: {ok}")
    return node


def subscribe_camera(S: Shared, name: str, topic: str):
    rec = S.films[name]

    def on_image(msg):
        if msg.width != WIDTH or msg.height != HEIGHT or len(msg.data) != WIDTH * HEIGHT * 3:
            if len(S.image_problems) < 5:
                S.image_problems.append(f"{name}: {msg.width}x{msg.height}, {len(msg.data)} bytes, format {msg.pixel_format_type}")
            return
        t = stamp_seconds(msg.header, S.now())
        if t is None:
            return
        rec.add(t, msg.data)

    return S.node.subscribe(S.Image, topic, on_image)


# ------------------------------------------------------------------------------------------------ start and stop the simulator
def start_stack(out: Path, px4: Path):
    procs = {}
    env = {**os.environ, "DISPLAY": ":99", "LIBGL_ALWAYS_SOFTWARE": "1", "HEADLESS": "1", "GZ_VERBOSE": "2"}
    procs["xvfb"] = subprocess.Popen(["Xvfb", ":99", "-screen", "0", "1280x1024x24", "+extension", "GLX", "+render", "-noreset"],
                                     stdout=open(out / "xvfb.log", "w"), stderr=subprocess.STDOUT, start_new_session=True)
    time.sleep(3)
    os.environ.update(env)
    cmd = f'cd "{px4}" && sleep 4000 | setsid make px4_sitl gz_x500_mono_cam_baylands > "{out / "sim.log"}" 2>&1'
    procs["px4"] = subprocess.Popen(["bash", "-c", cmd], env=env, start_new_session=True)
    return procs


def stop_stack(procs):
    for pattern in ("build/px4_sitl_default/bin/px4", "gz sim", "ruby"):
        sh(f'pkill -f "{pattern}"', 10)
    for k in ("px4", "xvfb"):
        p = procs.get(k)
        if p is not None:
            try:
                os.killpg(p.pid, signal.SIGTERM)
            except (ProcessLookupError, PermissionError):
                pass


def wait_for(pattern: str, timeout_s: float):
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        out = sh("gz topic -l")
        m = re.search(pattern, out, re.M)
        if m:
            return m, out
        time.sleep(3)
    return None, sh("gz topic -l")


# ------------------------------------------------------------------------------------------------ the flight
def yaw_plan(route: Route):
    taus = np.arange(0.0, route.duration + HOLD_END_S + 1e-9, PLAN_DT)
    vel = np.array([route.pos_vel(t)[1][:2] for t in taus])
    return taus, ny.plan_yaw(vel, PLAN_DT, YAW_RATE_MAX, V_MIN, 0.0)


def ned_yaw_deg(heading_rad):
    return 90.0 - math.degrees(heading_rad)


def tilt_of(q):
    return math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (q[0] ** 2 + q[1] ** 2)))))


SAMPLE_COLUMNS = ["t", "tau", "phase", "px", "py", "pz", "vx", "vy", "vz", "roll_deg", "pitch_deg", "yaw_deg", "truth_x", "truth_y", "truth_z",
                  "sp_x", "sp_y", "sp_z", "sp_vx", "sp_vy", "sp_vz", "sp_yaw_deg", "gates_passed"]


async def fly(S: Shared, child: dict, course: Course, route: Route, samples: list, live: dict):
    from mavsdk import System
    from mavsdk.offboard import OffboardError, PositionNedYaw, VelocityNedYaw

    taus, yaws = yaw_plan(route)

    def ref_heading(tau):
        return float(np.interp(tau, taus, yaws))

    end_pad = np.array(course.spec["end_pad"][:2], float) * course.scale
    hover_z = float(route.wp[0, 2])
    d = System()
    await d.connect(system_address="udpin://0.0.0.0:14540")
    log("waiting for the connection to PX4")

    async def connected():
        async for st in d.core.connection_state():
            if st.is_connected:
                return

    await asyncio.wait_for(connected(), 300)
    log("connected; waiting for a good position estimate and home")

    async def healthy():
        async for h in d.telemetry.health():
            if h.is_global_position_ok and h.is_home_position_ok:
                return h

    await asyncio.wait_for(healthy(), 900)
    log("healthy")
    for fn, hz in (("set_rate_position_velocity_ned", 50.0), ("set_rate_attitude_euler", 50.0)):
        try:
            await getattr(d.telemetry, fn)(hz)
        except Exception as e:  # noqa: BLE001
            child.setdefault("telemetry_rate_errors", []).append(f"{fn}: {e!r}"[:200])

    state = {"pv": None, "att": None, "armed": None, "pv_n": 0}

    async def watch_pv():
        async for pv in d.telemetry.position_velocity_ned():
            state["pv"] = pv
            state["pv_n"] += 1

    async def watch_att():
        async for a in d.telemetry.attitude_euler():
            state["att"] = a

    async def watch_armed():
        async for a in d.telemetry.armed():
            state["armed"] = a

    tasks = [asyncio.ensure_future(f()) for f in (watch_pv, watch_att, watch_armed)]
    t_wait = time.monotonic()
    while state["pv"] is None and time.monotonic() - t_wait < 60:
        await asyncio.sleep(0.1)
    if state["pv"] is None:
        raise RuntimeError("no position_velocity_ned telemetry from PX4")

    def px4_f():
        """PX4 position and velocity in F (before the offset)."""
        pv = state["pv"]
        return (np.array([pv.position.east_m, pv.position.north_m, -pv.position.down_m]),
                np.array([pv.velocity.east_m_s, pv.velocity.north_m_s, -pv.velocity.down_m_s]))

    # the cameras draw only while somebody listens, so start the recording (and the listening) here
    with S.lock:
        S.poses = []
        S.recording = True
    for name in S.films:
        S.films[name].active = True
        child.setdefault("camera_subscribe", {})[name] = bool(subscribe_camera(S, name, S.cam_topics[name]))
    wall0 = time.monotonic()
    child["record_start_sim_s"] = S.now()
    yaw0 = ned_yaw_deg(0.0)
    await d.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, yaw0))
    await d.action.arm()
    t_arm = S.now()
    log(f"armed at sim time {t_arm}")
    try:
        await d.offboard.start()
    except OffboardError as e:
        child["offboard_error"] = str(e)
        raise
    log("offboard started")

    phase, tau0, tau1, end_reason, settled_since = "climb", None, None, None, None
    off = np.zeros(3)
    counter = None
    live_min = {"gate": (math.inf, None), "pole": (math.inf, None), "box": (math.inf, None)}
    n_sp_route, t_land, t_end = 0, None, None
    last_t_seen, last_adv_wall, last_log_wall = None, time.monotonic(), 0.0
    landed_flag = False
    while True:
        await asyncio.sleep(0.02)
        t = S.now()
        if t is None:
            continue
        if t != last_t_seen:
            last_t_seen, last_adv_wall = t, time.monotonic()
        elif time.monotonic() - last_adv_wall > SIM_DEAD_WALL_S:
            end_reason = "sim_dead"
            break
        if time.monotonic() - wall0 > FLIGHT_TIMEOUT_S:
            end_reason = "time"
            break
        if t_end is not None:
            if t >= t_end:
                break
        with S.lock:
            truth = S.pose
        if truth is not None and tilt_of(truth[3:]) > 75.0 and t - t_arm > 3 and t_end is None:
            end_reason = "crash"
            child["crash_sim_s"] = t
            t_end = t + 2.0
            continue
        pf, vf = px4_f()
        att = state["att"]
        sp = None          # (position F in the PX4 frame, velocity, yaw deg) of this step
        if phase == "climb":
            zsp = min(hover_z, CLIMB_RATE * max(0.0, t - t_arm))
            vz = CLIMB_RATE if zsp < hover_z else 0.0
            sp = (np.array([0.0, 0.0, zsp]), np.array([0.0, 0.0, vz]), yaw0)
            if zsp >= hover_z:
                near = math.dist(pf, (0.0, 0.0, hover_z)) < 0.4
                settled_since = (settled_since if settled_since is not None else t) if near else None
                if settled_since is not None and t - settled_since > HOLD_START_S and truth is not None:
                    off = np.array(truth[:3]) - pf
                    child["px4_to_truth_offset_m"] = [float(v) for v in off]
                    phase, tau0 = "route", t
                    child["route_start_sim_s"] = t
                    counter = LapCounter(course, laps=LAPS, t_start=t)
                    log(f"route starts at sim time {t}; offset truth - PX4 = {off}")
            if t - t_arm > 90.0 and phase == "climb":
                child["climb_failed"] = True
                end_reason = "time"
                break
        elif phase == "route":
            tau = t - tau0
            if tau > route.duration + HOLD_END_S:
                phase, tau1 = "descend", t
                child["route_end_sim_s"] = t
                log(f"route finished at sim time {t}; descending over the end pad")
                continue
            p, v = route.pos_vel(tau)
            sp = (p - off, v, ned_yaw_deg(ref_heading(tau)))
            n_sp_route += 1
        elif phase == "descend":
            z = max(LAND_HANDOVER_M, hover_z - DESCENT_RATE * (t - tau1))
            vz = -DESCENT_RATE if z > LAND_HANDOVER_M else 0.0
            sp = (np.array([end_pad[0], end_pad[1], z]) - off, np.array([0.0, 0.0, vz]), ned_yaw_deg(ref_heading(route.duration)))
            if z <= LAND_HANDOVER_M + 1e-6 and abs(pf[2] + off[2] - LAND_HANDOVER_M) < 0.35:
                try:
                    await d.action.land()
                    t_land = t
                    child["land_cmd_sim_s"] = t
                    log(f"land command at sim time {t}")
                except Exception as e:  # noqa: BLE001
                    child["land_error"] = repr(e)
                    end_reason = "land_error"
                    t_end = t + 2.0
                phase = "landing"
                sp = None
            if t - tau1 > 60:
                child["descend_timeout"] = True
                end_reason = "time"
                break
        elif phase == "landing":
            if state["armed"] is False:
                child["disarm_sim_s"] = t
                end_reason = "landed"
                landed_flag = True
                t_end = t + 2.0
                phase = "landed"
                log(f"disarmed at sim time {t}; 2 s more")
            elif t - t_land > 70:
                child["landing_timeout"] = True
                end_reason = "time"
                t_end = t + 2.0
        if sp is not None:
            pos, vel, yaw = sp
            await d.offboard.set_position_velocity_ned(PositionNedYaw(float(pos[1]), float(pos[0]), float(-pos[2]), float(yaw)),
                                                      VelocityNedYaw(float(vel[1]), float(vel[0]), float(-vel[2]), float(yaw)))
        live_p = pf + off
        if counter is not None:
            counter.update(t, live_p)
            for k, v in distances(course, live_p).items():
                if v[0] < live_min[k][0]:
                    live_min[k] = v
        samples.append([round(t, 4), None if tau0 is None else round(t - tau0, 4), phase, *(round(float(x), 4) for x in live_p), *(round(float(x), 4) for x in vf),
                        None if att is None else round(att.roll_deg, 3), None if att is None else round(att.pitch_deg, 3), None if att is None else round(att.yaw_deg, 3),
                        *( [round(float(x), 4) for x in truth[:3]] if truth is not None else [None] * 3),
                        *( [round(float(x), 4) for x in (sp[0] + off)] if sp is not None else [None] * 3),
                        *( [round(float(x), 4) for x in sp[1]] if sp is not None else [None] * 3),
                        None if sp is None else round(float(sp[2]), 3), 0 if counter is None else counter.count])
        if time.monotonic() - last_log_wall > 15:
            last_log_wall = time.monotonic()
            el = max(1e-6, last_log_wall - wall0)
            log(f"phase {phase} sim {t:.1f} rtf {(t - child['record_start_sim_s']) / el:.2f} gates {0 if counter is None else counter.count} truth {None if truth is None else [round(v, 2) for v in truth[:3]]}")
    child["flight_end_reason"] = end_reason
    child["record_wall_s"] = time.monotonic() - wall0
    with S.lock:
        S.recording = False
        child["record_end_sim_s"] = S.sim_t
    for name in S.films:
        S.films[name].active = False
        try:
            S.node.unsubscribe(S.cam_topics[name])
        except Exception:  # noqa: BLE001
            pass
    for t_ in tasks:
        t_.cancel()
    child["landed_flag"] = landed_flag
    child["setpoints_route"] = n_sp_route
    child["px4_position_messages"] = state["pv_n"]
    if counter is not None:
        live.update(gates_passed=counter.count, lap_times_s=[float(x) for x in counter.lap_times], wrong_crossings=len(counter.wrong),
                    passage_log=[[float(a), int(b), c, float(d_), float(e)] for a, b, c, d_, e in counter.log],
                    min_clearance_m={k: {"distance_m": float(v[0]), "id": v[1]} for k, v in live_min.items()})


# ------------------------------------------------------------------------------------------------ the whole run
def versions(px4: Path) -> dict:
    v = {"px4_commit": sh(f'git -C "{px4}" rev-parse HEAD').strip(), "px4_describe": sh(f'git -C "{px4}" describe --tags --always').strip(),
         "gz_sim": sh("gz sim --versions").strip()}
    try:
        from importlib.metadata import version
        v["mavsdk_python"] = version("mavsdk")
    except Exception:  # noqa: BLE001
        v["mavsdk_python"] = None
    return v


def run_stack(out: Path, px4: Path) -> dict:
    course = Course.load(TRACK, SCALE)
    route = Route(course, laps=LAPS)
    child = {"tag": "kestrel_gz_two_laps", "laps": LAPS, "scale": SCALE, "route_duration_s": route.duration, "route_speed_mps": route.speed,
             "versions": versions(px4)}
    S = Shared()
    samples, live = [], {}
    procs = None
    try:
        child["files"] = prepare_files(px4, out, course)
        for name in ("nose", "outside"):
            S.films[name] = FilmPipe(out / f"{name}.mp4")
        procs = start_stack(out, px4)
        t_start = time.time()
        m, topics = wait_for(r"^/world/([^/]+)/clock$", STACK_START_TIMEOUT_S)
        (out / "gz_topics.txt").write_text(topics)
        if not m:
            raise RuntimeError("no Gazebo world clock topic: Gazebo did not start (see sim.log)")
        world = m.group(1)
        child["world"] = world
        log(f"world {world} after {time.time() - t_start:.0f} s")
        cams = {}
        for name, sensor in (("nose", r"link/camera_link/sensor/imager/image"), ("outside", r"link/chase_cam_link/sensor/chase_cam/image")):
            mm, topics = wait_for(r"^(/world/%s/model/%s/%s)$" % (re.escape(world), re.escape(MODEL), sensor), 300)
            (out / "gz_topics.txt").write_text(topics)
            if not mm:
                raise RuntimeError(f"camera topic for {name} did not appear")
            cams[name] = mm.group(1)
        S.cam_topics = cams
        child["camera_topics"] = cams
        setup_gz(S, world)
        t0 = time.time()
        while S.pose is None and time.time() - t0 < 300:
            time.sleep(1)
        if S.pose is None:
            raise RuntimeError("no pose of the drone model in dynamic_pose/info (topics in gz_topics.txt)")
        time.sleep(8)  # let the drone settle on the ground
        with S.lock:
            rest = list(S.pose)
        child["rest_pose_world"] = rest
        if math.hypot(rest[0], rest[1]) > 0.3:
            child["spawn_warning"] = "the drone did not spawn at the world origin: gates are NOT where the route expects them"
            log(child["spawn_warning"])
        child["gz_models"] = sh("gz model --list", 20)[:2000]
        asyncio.run(fly(S, child, course, route, samples, live))
        child["pose_messages"] = S.pose_n
        child["pose_messages_without_stamp"] = S.stamp_zero
        child["image_problems"] = S.image_problems
    except Exception:  # noqa: BLE001
        child["traceback"] = traceback.format_exc()
        log(child["traceback"])
    finally:
        film_stats = {}
        for name, rec in S.films.items():
            try:
                film_stats[name] = rec.close()
            except Exception:  # noqa: BLE001
                film_stats[name] = {"error": traceback.format_exc()[-400:]}
        child["films"] = film_stats
        if procs is not None:
            stop_stack(procs)
    # samples and the truth path are saved before the analysis, so a bug in the analysis cannot lose them
    (out / "samples.json").write_text(json.dumps({"columns": SAMPLE_COLUMNS, "rows": samples}, separators=(",", ":")))
    poses = np.array(S.poses, dtype=float) if S.poses else np.zeros((0, 8))
    np.save(out / "truth_poses.npy", poses)
    child["samples"] = len(samples)
    child["truth_poses"] = int(len(poses))
    return {"child": child, "live": live, "poses": poses, "course": course, "route": route}


def write_result(out: Path, run: dict) -> dict:
    child, live, poses, course, route = run["child"], run["live"], run["poses"], run["course"], run["route"]
    res = {"passed": False, "versions": child.get("versions"), "flight_end_reason": child.get("flight_end_reason"), "route_duration_s": route.duration,
           "route_start_sim_s": child.get("route_start_sim_s"), "errors": []}
    if "traceback" in child:
        res["errors"].append(child["traceback"][-1500:])
    rec_wall, rec_sim = child.get("record_wall_s"), None
    if child.get("record_end_sim_s") is not None and child.get("record_start_sim_s") is not None:
        rec_sim = child["record_end_sim_s"] - child["record_start_sim_s"]
    res["real_time_factor"] = (rec_sim / rec_wall) if rec_sim and rec_wall else None
    res["sim_seconds_recorded"], res["wall_seconds_recorded"] = rec_sim, rec_wall
    res["camera"] = {"nose": child.get("films", {}).get("nose"), "outside": child.get("films", {}).get("outside"), "image_problems": child.get("image_problems")}
    res["camera_fps"] = (res["camera"]["nose"] or {}).get("camera_fps_achieved")
    sp_span = None
    if child.get("route_start_sim_s") is not None and child.get("route_end_sim_s") is not None:
        sp_span = child["route_end_sim_s"] - child["route_start_sim_s"]
    res["setpoint_rate_hz_sim"] = (child.get("setpoints_route") / sp_span) if sp_span and child.get("setpoints_route") else None
    res["live"] = live
    res["notes"] = {k: child[k] for k in ("spawn_warning", "px4_to_truth_offset_m", "climb_failed", "land_error", "landing_timeout", "descend_timeout", "crash_sim_s",
                                         "offboard_error", "telemetry_rate_errors", "rest_pose_world", "disarm_sim_s") if k in child}
    try:
        if len(poses) > 10 and child.get("route_start_sim_s") is not None:
            end_pad = np.array(course.spec["end_pad"][:2], float) * course.scale
            a = AN.analyse(course, route, poses[:, 0], poses[:, 1:4], poses[:, 4:8], child["route_start_sim_s"], end_pad, bool(child.get("landed_flag")), LAPS)
            res.update(a)
        else:
            res["errors"].append(f"no analysis: {len(poses)} truth poses, route start {child.get('route_start_sim_s')}")
    except Exception:  # noqa: BLE001
        res["errors"].append("analysis: " + traceback.format_exc()[-1500:])
    (out / "result.json").write_text(json.dumps(res, indent=2, default=str))
    (out / "child.json").write_text(json.dumps(child, indent=2, default=str))
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--px4", default="px4")
    ap.add_argument("--out", default="results")
    a = ap.parse_args()
    out = Path(a.out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    run = run_stack(out, Path(a.px4).resolve())
    res = write_result(out, run)
    try:
        from sim.gz_px4 import films
        stats = films.make_path_film(run["course"], run["route"], run["poses"], res, out / "path.mp4")
        res["path_film"] = stats
    except Exception:  # noqa: BLE001
        res.setdefault("errors", []).append("path film: " + traceback.format_exc()[-1500:])
    (out / "result.json").write_text(json.dumps(res, indent=2, default=str))
    log(f"RESULT passed={res.get('passed')} gates={res.get('gates_passed')} checks={res.get('checks')} errors={len(res.get('errors', []))}")
    sys.exit(0 if res.get("passed") else 1)


if __name__ == "__main__":
    main()
