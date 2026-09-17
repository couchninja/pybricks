"""Build ``navigator/web_viewer/dist/viewer.js`` (requires pixi ``nodejs`` / ``npm`` on PATH)."""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

_WEB_VIEWER_DIR = Path(__file__).resolve().parent / "web_viewer"


def build_web_viewer_if_ready() -> None:
    if shutil.which("npm") is None:
        raise RuntimeError(
            "npm not found on PATH; use pixi run (nodejs is a pixi dependency)"
        )
    if not (_WEB_VIEWER_DIR / "node_modules").is_dir():
        print("Installing navigator web viewer npm dependencies...")
        subprocess.run(
            ["npm", "install"],
            cwd=_WEB_VIEWER_DIR,
            check=True,
        )
    print("Building navigator web viewer (viewer.js)...")
    subprocess.run(
        ["npm", "run", "build"],
        cwd=_WEB_VIEWER_DIR,
        check=True,
    )
