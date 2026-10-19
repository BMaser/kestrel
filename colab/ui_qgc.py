# Kestrel UI notebook, step 4: QGroundControl, PX4's own ground station, on the desktop. It listens on UDP 14550, where PX4 SITL
# sends its MAVLink stream, so it connects to the drone by itself once the flight starts. QGroundControl refuses to run as root
# (Colab runs as root), so it runs as a plain user "pilot".


def step_qgc():
    log = LOCAL / "qgc.log"
    home = Path("/home/pilot")
    app = home / "qgc" / "squashfs-root" / "AppRun"
    if not app.exists():
        _, rel = out("curl -s https://api.github.com/repos/mavlink/qgroundcontrol/releases/latest")
        urls = re.findall(r'"browser_download_url":\s*"([^"]+\.AppImage)"', rel)
        url = next((u for u in urls if "x86_64" in u or "amd64" in u), urls[0] if urls else None)
        if url is None:
            raise SystemExit("no QGroundControl AppImage in the latest release; see qgc.log\n" + rel[:500])
        say("QGroundControl:", url)
        sh(f"id pilot || useradd -m pilot; mkdir -p {home}/qgc && cd {home}/qgc && wget -q -O QGC.AppImage '{url}' && chmod +x QGC.AppImage"
           f" && ./QGC.AppImage --appimage-extract > /dev/null && chown -R pilot:pilot {home}", log)
        (LOCAL / "qgc_version.txt").write_text(url + "\n")
    if not app.exists():
        raise SystemExit("QGroundControl did not unpack; see qgc.log")
    env = {**os.environ, "DISPLAY": DISPLAY, "HOME": str(home), "LIBGL_ALWAYS_SOFTWARE": "1", "QT_QPA_PLATFORM": "xcb"}
    background(f"exec runuser -u pilot -- env DISPLAY={DISPLAY} HOME={home} LIBGL_ALWAYS_SOFTWARE=1 QT_QPA_PLATFORM=xcb {app}",
               LOCAL / "qgc_run.log", env=env)
    time.sleep(15)
    rc, ps = out("pgrep -fa QGroundControl | head -n 3")
    say("QGroundControl running:" if rc == 0 else "QGroundControl is NOT running; last lines of qgc_run.log:\n" + tail(LOCAL / "qgc_run.log", 25), ps.strip())


step_qgc()
sync()
