from watchfiles import Change

from navigator.nav_watch_filter import NavigatorWatchFilter


def test_nav_watch_filter_ignores_viewer_build_artifacts() -> None:
    watch_filter = NavigatorWatchFilter()
    root = "/Users/example/code/navigator"
    assert not watch_filter(Change.modified, f"{root}/navigator/web_viewer/dist/viewer.js")
    assert not watch_filter(Change.modified, f"{root}/navigator/web_viewer/.tsbuildinfo")
    assert watch_filter(Change.modified, f"{root}/navigator/web_viewer/src/main.ts")
    assert watch_filter(Change.modified, f"{root}/navigator/web_ui.py")
