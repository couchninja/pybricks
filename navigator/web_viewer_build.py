"""Build ``navigator/web_viewer/dist/viewer.js`` when npm dependencies are present."""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

_WEB_VIEWER_DIR = Path(__file__).resolve().parent / "web_viewer"


def build_web_viewer_if_ready() -> None:
    if shutil.which("npm") is None:
        return
    if not (_WEB_VIEWER_DIR / "node_modules").is_dir():
        return
    print("Building navigator web viewer (viewer.js)...")
    subprocess.run(
        ["npm", "run", "build"],
        cwd=_WEB_VIEWER_DIR,
        check=True,
    )
