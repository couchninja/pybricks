from pathlib import Path
from unittest.mock import patch

from navigator.web_viewer_build import build_web_viewer_if_ready


def test_build_skips_without_npm() -> None:
    with patch("navigator.web_viewer_build.shutil.which", return_value=None):
        build_web_viewer_if_ready()


def test_build_runs_npm_when_ready(tmp_path: Path) -> None:
    viewer_dir = tmp_path / "web_viewer"
    viewer_dir.mkdir()
    (viewer_dir / "node_modules").mkdir()
    with (
        patch("navigator.web_viewer_build.shutil.which", return_value="/usr/bin/npm"),
        patch("navigator.web_viewer_build._WEB_VIEWER_DIR", viewer_dir),
        patch("navigator.web_viewer_build.subprocess.run") as run,
    ):
        build_web_viewer_if_ready()
    run.assert_called_once()
    assert run.call_args.args[0] == ["npm", "run", "build"]
    assert run.call_args.kwargs["cwd"] == viewer_dir
