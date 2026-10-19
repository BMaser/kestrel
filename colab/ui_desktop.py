# Kestrel UI notebook, step 3: a virtual desktop (Xvfb + fluxbox), shared by VNC with a password, shown in the browser by noVNC.
# Two ways in: Colab's own port link (only your Google account can open it) and, as a fallback, a Cloudflare quick tunnel
# (a random public address; the VNC password keeps others out).
from IPython.display import HTML, display  # noqa: E402


def step_desktop():
    log = LOCAL / "desktop.log"
    sh("pkill -f 'Xvfb :1'; pkill -f x11vnc; pkill -f websockify; pkill -f fluxbox; pkill -f cloudflared; sleep 1; true", log)
    background(f"exec Xvfb {DISPLAY} -screen 0 {SCREEN} -ac +extension GLX +render -noreset", LOCAL / "xvfb.log")
    time.sleep(3)
    env = {**os.environ, "DISPLAY": DISPLAY}
    background("exec fluxbox", LOCAL / "fluxbox.log", env=env)
    global VNC_PASSWORD
    VNC_PASSWORD = secrets.token_hex(4)                    # VNC passwords are at most 8 characters
    sh(f"mkdir -p /root/.vnc && x11vnc -storepasswd {VNC_PASSWORD} /root/.vnc/passwd", log)
    background(f"exec x11vnc -display {DISPLAY} -rfbauth /root/.vnc/passwd -forever -shared -rfbport {VNC_PORT} -noxdamage", LOCAL / "x11vnc.log")
    web = "/usr/share/novnc"
    background(f"exec websockify --web {web} {WEB_PORT} localhost:{VNC_PORT}", LOCAL / "websockify.log")
    time.sleep(3)
    background(f"exec xterm -geometry 100x15+0+0 -T 'Kestrel log' -e 'tail -F {LOCAL}/run.log'",LOCAL / "xterm.log", env=env)
    page = "vnc.html?autoconnect=1&resize=scale&reconnect=1"
    links = []
    try:
        from google.colab.output import eval_js
        base = eval_js(f"google.colab.kernel.proxyPort({WEB_PORT})")
        links.append(("Colab link (your Google account only)", base.rstrip("/") + "/" + page))
    except Exception as e:  # noqa: BLE001
        say("Colab port link not available:", repr(e))
    if not Path("/usr/local/bin/cloudflared").exists():
        sh("wget -q -O /usr/local/bin/cloudflared https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-amd64"
           " && chmod +x /usr/local/bin/cloudflared", log)
    background(f"exec cloudflared tunnel --no-autoupdate --url http://localhost:{WEB_PORT}", LOCAL / "cloudflared.log")
    url = None
    for _ in range(30):
        time.sleep(2)
        m = re.search(r"https://[a-z0-9-]+\.trycloudflare\.com", tail(LOCAL / "cloudflared.log", 200))
        if m:
            url = m.group(0)
            break
    if url:
        links.append(("Cloudflare link (fallback)", url + "/" + page))
    rows = "".join(f'<li><a href="{u}" target="_blank">{n}</a></li>' for n, u in links)
    display(HTML(f"<h3>Your desktop</h3><ol>{rows}</ol><p>VNC password: <b>{VNC_PASSWORD}</b> (new every run)</p>"
                 "<p>Try link 1 first. If the page stays grey or says it cannot connect, try link 2.</p>"))
    (LOCAL / "desktop_links.txt").write_text("\n".join(n for n, _ in links) + "\n")   # names only: the links and password stay in this notebook


step_desktop()
sync()
