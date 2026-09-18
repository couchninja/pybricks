"""watchfiles filter for ``pixi run nav-watch``."""

from watchfiles.filters import DefaultFilter


class NavigatorWatchFilter(DefaultFilter):
    """Ignore web viewer build artifacts (watchfiles reports absolute paths)."""

    ignore_dirs = (
        *DefaultFilter.ignore_dirs,
        "dist",
    )
    ignore_entity_patterns = (
        *DefaultFilter.ignore_entity_patterns,
        r"^\.tsbuildinfo$",
    )
