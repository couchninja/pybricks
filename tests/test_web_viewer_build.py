import subprocess
from pathlib import Path
from unittest.mock import patch

import pytest

from navigator.web_viewer_build import build_web_viewer_if_ready

_WEB_VIEWER_DIR = Path(__file__).resolve().parents[1] / "navigator" / "web_viewer"


def test_build_raises_without_npm() -> None:
    with (
        patch("navigator.web_viewer_build.shutil.which", return_value=None),
        pytest.raises(RuntimeError, match="npm not found"),
    ):
        build_web_viewer_if_ready()


def test_build_runs_npm_install_when_node_modules_missing(tmp_path: Path) -> None:
    viewer_dir = tmp_path / "web_viewer"
    viewer_dir.mkdir()
    with (
        patch("navigator.web_viewer_build.shutil.which", return_value="/usr/bin/npm"),
        patch("navigator.web_viewer_build._WEB_VIEWER_DIR", viewer_dir),
        patch("navigator.web_viewer_build.subprocess.run") as run,
    ):
        build_web_viewer_if_ready()
    assert run.call_count == 2
    assert run.call_args_list[0].args[0] == ["npm", "install"]
    assert run.call_args_list[1].args[0] == ["npm", "run", "build"]
    assert run.call_args_list[0].kwargs["cwd"] == viewer_dir


def test_build_runs_npm_install_even_when_node_modules_exists(tmp_path: Path) -> None:
    viewer_dir = tmp_path / "web_viewer"
    viewer_dir.mkdir()
    (viewer_dir / "node_modules").mkdir()
    with (
        patch("navigator.web_viewer_build.shutil.which", return_value="/usr/bin/npm"),
        patch("navigator.web_viewer_build._WEB_VIEWER_DIR", viewer_dir),
        patch("navigator.web_viewer_build.subprocess.run") as run,
    ):
        build_web_viewer_if_ready()
    assert run.call_count == 2
    assert run.call_args_list[0].args[0] == ["npm", "install"]
    assert run.call_args_list[1].args[0] == ["npm", "run", "build"]
    assert run.call_args_list[0].kwargs["cwd"] == viewer_dir


def test_flat_arrow_orientation() -> None:
    subprocess.run(["npm", "test"], cwd=_WEB_VIEWER_DIR, check=True)
