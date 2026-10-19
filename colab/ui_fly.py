# Kestrel UI notebook, step 5: the two-lap flight of the GitHub test (sim/gz_px4/fly.py), with Gazebo's own window on the desktop.
# Software drawing on Colab's 2 processor cores is slower than GitHub's 4, so the wall-clock limit of the flight is 90 minutes.


def step_fly():
    sh("pkill -f 'sim.gz_px4.fly'; pkill -f 'gz sim'; pkill -f 'px4_sitl_default/bin/px4'; pkill -f mavsdk_server; sleep 2; true", LOCAL / "fly_cleanup.log")
    env = {**os.environ, "PYTHONPATH": str(KESTREL)}
    env.pop("HEADLESS", None)
    proc = background(f"cd {KESTREL} && exec {WORK}/venv/bin/python -m sim.gz_px4.fly --px4 {PX4} --out {LOCAL}"
                      f" --gui-display {DISPLAY} --flight-timeout-min 90 > {LOCAL}/run.log 2>&1", LOCAL / "fly_launch.log", env=env)
    say("flight started; watch it on the desktop. Progress every 30 s:")
    last = ""
    while proc.poll() is None:
        time.sleep(30)
        lines = [l for l in tail(LOCAL / "run.log", 40).splitlines() if l.strip()]
        now = lines[-1] if lines else "(no output yet)"
        if now != last:
            say(now[:200])
            last = now
        sync()
    say("flight program ended, exit code", proc.returncode)
    print(tail(LOCAL / "run.log", 15))
    sync()


step_fly()
