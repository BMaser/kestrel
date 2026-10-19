# Kestrel UI notebook, step 2: the desktop programs, PX4 v1.17.0, Gazebo Harmonic, MAVSDK, and the PX4 build (30 to 40 minutes).
# PX4 part copied from the Colab notebook that worked on 2026-10-17/18 (private repo, colab_px4/notebook_source.py, step 4).


def step_install():
    log = LOCAL / "install.log"
    say("desktop programs (virtual screen, window manager, VNC, noVNC) ...")
    sh("apt-get update -qq", log)
    failed = apt(["xvfb", "xauth", "x11vnc", "fluxbox", "xterm", "novnc", "websockify", "mesa-utils", "ffmpeg", "git", "wget", "curl",
                  "libfuse2", "gstreamer1.0-plugins-bad", "gstreamer1.0-libav", "gstreamer1.0-gl", "libxcb-xinerama0",
                  "libxkbcommon-x11-0", "libxcb-cursor0", "libxcb-cursor-dev", "libegl1", "libgl1", "libpulse-mainloop-glib0"], log)
    say("desktop packages that failed:", failed or "none")
    if shutil.which("sudo") is None:                       # Colab runs as root; PX4's set-up script calls sudo
        Path("/usr/local/bin/sudo").write_text('#!/bin/sh\nexec "$@"\n')
        os.chmod("/usr/local/bin/sudo", 0o755)
    if not (PX4 / "Makefile").exists():
        rc = sh(f"git clone --branch {PX4_TAG} --depth 1 --recurse-submodules --shallow-submodules https://github.com/PX4/PX4-Autopilot.git {PX4}", log)
        if rc:
            raise SystemExit("git clone of PX4 failed; see install.log")
    (LOCAL / "px4_version.txt").write_text(out(f"git -C {PX4} rev-parse HEAD")[1])
    say("PX4 set-up script (Gazebo Harmonic and the build tools) ...")
    rc = sh(f"DEBIAN_FRONTEND=noninteractive bash {PX4}/Tools/setup/ubuntu.sh --no-nuttx", log, timeout=3600)
    say("set-up script exit code", rc)
    failed = apt(["python3-gz-transport13", "python3-gz-msgs10", "python3-numpy", "python3-scipy", "python3-matplotlib", "python3-venv",
                  "python3.12-venv", "python3.10-venv"], log)
    say("apt names that failed (one of python3.12-venv / python3.10-venv is expected):", failed or "none")
    # the Gazebo Python bindings are built for Ubuntu's own Python (3.12 on 24.04, 3.10 on 22.04), not for Colab's Python
    py, tried = None, []
    for p in ("/usr/bin/python3.12", "/usr/bin/python3.10", "/usr/bin/python3"):
        if not Path(p).exists():
            continue
        rc_, msg = out(f"{p} -c 'import sys; from gz.transport13 import Node; print(sys.version)'")
        tried.append(f"{p}: exit {rc_}: {msg.strip()[-300:]}")
        if rc_ == 0:
            py = p
            break
    with open(log, "a") as f:
        f.write("\nPython candidates for the Gazebo bindings:\n" + "\n".join(tried) + "\n")
    if py is None:
        raise SystemExit("no Python finds the Gazebo bindings (gz.transport13):\n" + "\n".join(tried))
    say("Python for the Gazebo bindings:", py)
    # numpy, scipy and matplotlib go into the venv together (GitHub run 39203502427: the system scipy broke on the venv's numpy 2)
    sh(f"{py} -m venv --system-site-packages {WORK}/venv && {WORK}/venv/bin/pip install -q 'mavsdk<4' numpy scipy matplotlib", log)
    _, v = out(f"{WORK}/venv/bin/python -c \"import mavsdk, numpy, scipy, matplotlib; from gz.transport13 import Node; print('python ok')\"; gz sim --versions")
    (LOCAL / "versions.txt").write_text(v)
    say("versions:", v.strip().replace("\n", " | "))
    say("building PX4 SITL (the longest part) ...")
    rc = sh(f"cd {PX4} && make px4_sitl_default", LOCAL / "build.log", timeout=5400)
    if rc:
        print(tail(LOCAL / "build.log", 60))
        raise SystemExit("the PX4 build failed; see build.log")
    say("PX4 built.", tail(LOCAL / "build.log", 2))


step_install()
sync()
