# Kestrel UI notebook, step 0: names, paths and helpers. Run with exec() inside the notebook, so later steps share these names.
# The flow is the one of the PX4 + Gazebo Colab notebook that ran on 2026-10-17/18 (private repo, colab_px4/notebook_source.py).
import datetime
import json
import os
import re
import secrets
import shutil
import subprocess
import threading
import time
from pathlib import Path

STAMP = datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%d_%H%M")
WORK = Path("/content")
KESTREL = WORK / "kestrel"                 # this repository, cloned by the notebook's first cell
PX4 = WORK / "px4"                         # PX4-Autopilot, built here
PX4_TAG = "v1.17.0"                        # the version of the GitHub two-lap pass (run 39204204269)
LOCAL = WORK / "kestrel_results" / STAMP   # results while the run goes on; copied to Drive every 20 s
DRIVE_RUN_ROOT = Path("/content/drive/MyDrive/github/Comprehansive_cv/coursera_mirror/mirrors/Autonomous_Racing_Drone_Path/runs/2026-10-19_kestrel_ui")
DRIVE = None
DISPLAY = ":1"                             # the desktop that the browser shows
SCREEN = "1600x900x24"
VNC_PORT, WEB_PORT = 5901, 6080
LOCAL.mkdir(parents=True, exist_ok=True)


def say(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


def sh(cmd, log, timeout=None, env=None):
    with open(log, "ab") as f:
        f.write(f"\n$ {cmd}\n".encode())
        f.flush()
        try:
            return subprocess.run(["bash", "-c", cmd], stdout=f, stderr=subprocess.STDOUT, timeout=timeout, env=env).returncode
        except subprocess.TimeoutExpired:
            return 124


def out(cmd, timeout=120):
    try:
        r = subprocess.run(["bash", "-c", cmd], capture_output=True, text=True, timeout=timeout)
        return r.returncode, r.stdout + r.stderr
    except subprocess.TimeoutExpired:
        return 124, "timeout"


def apt(pkgs, log):
    """Install packages; if the group fails (one unknown name fails all), one by one. Returns the names that failed."""
    if sh("DEBIAN_FRONTEND=noninteractive apt-get install -y -qq --no-install-recommends " + " ".join(pkgs), log) == 0:
        return []
    return [p for p in pkgs if sh(f"DEBIAN_FRONTEND=noninteractive apt-get install -y -qq --no-install-recommends {p}", log) != 0]


def tail(p, n=30):
    try:
        return "".join(Path(p).read_text(errors="replace").splitlines(keepends=True)[-n:])
    except Exception as e:  # noqa: BLE001
        return f"(cannot read {p}: {e!r})"


def background(cmd, log, env=None):
    """Start a long-running program in its own session; its output goes to log."""
    return subprocess.Popen(["bash", "-c", cmd], stdout=open(log, "ab"), stderr=subprocess.STDOUT, start_new_session=True, env=env)


_synced = {}


def sync():
    if DRIVE is None:
        return
    for p in LOCAL.rglob("*"):
        try:
            if not p.is_file() or p.suffix in (".raw", ".npy"):
                continue
            st = p.stat()
            key = (st.st_size, int(st.st_mtime))
            if _synced.get(p) == key:
                continue
            d = DRIVE / p.relative_to(LOCAL)
            d.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(p, d)
            _synced[p] = key
        except Exception:  # noqa: BLE001
            pass


def _sync_loop():
    while True:
        time.sleep(20)
        sync()


say("Kestrel UI, results (local):", LOCAL)
