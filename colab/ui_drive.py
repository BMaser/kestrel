# Kestrel UI notebook, step 1: Google Drive, where the results are copied every 20 s.
from google.colab import drive  # noqa: E402

drive.mount("/content/drive")
DRIVE = DRIVE_RUN_ROOT / STAMP
DRIVE.mkdir(parents=True, exist_ok=True)
threading.Thread(target=_sync_loop, daemon=True).start()
(LOCAL / "kestrel_commit.txt").write_text(out(f"git -C {KESTREL} rev-parse HEAD")[1])
say("results on Drive:", DRIVE)
