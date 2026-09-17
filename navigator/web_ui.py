from __future__ import annotations

import asyncio
import json
import mimetypes
import sys
import uuid
from collections import deque
from collections.abc import AsyncIterator, Iterator
from contextlib import asynccontextmanager, contextmanager, suppress
from pathlib import Path
from typing import TYPE_CHECKING

from astropy import units as u

from simulate.astronomy.constants import PointingTarget
from simulate.astronomy.simulation_clock import (
    set_time_scale_preset,
    simulation_time,
    sync_to_realtime,
    time_scale_status_payload,
)
from simulate.astronomy.utils.ephemeris import observer_surface_vector_and_euler_angles_for_target
from simulate.astronomy.web_scene import scene_snapshot_payload

if TYPE_CHECKING:
    from gpio.button_menu import ButtonMenu
    from gpio.button_menu_host import HostButtonMenu

    ButtonTargetSource = ButtonMenu | HostButtonMenu
else:
    ButtonTargetSource = object

WEB_HOST = "0.0.0.0"
WEB_PORT_LINUX = 8765
WEB_PORT_DARWIN = 18765


def web_port_for_platform(platform: str) -> int:
    # Cursor often binds localhost:8765 on macOS; use a distinct port for local dev.
    if platform == "darwin":
        return WEB_PORT_DARWIN
    return WEB_PORT_LINUX


WEB_PORT = web_port_for_platform(sys.platform)
LOG_LINE_LIMIT = 200
_STATUS_POLL_MS = 2000
_VIEWER_DIST = Path(__file__).resolve().parent / "web_viewer" / "dist"
_SESSION_POLL_MS = 1000
_navigator_session_id: str | None = None

_BODY_TARGETS = frozenset(
    {
        PointingTarget.SUN,
        PointingTarget.MOON,
        PointingTarget.MILKY_WAY_CENTER,
        PointingTarget.ISS,
    }
)

_INDEX_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
  <title>Earth-sun viewer</title>
  <style>
    :root {
      color-scheme: dark light;
      --bg: #0f1419;
      --card: #1a2332;
      --text: #e8eef4;
      --muted: #8b9aab;
      --accent: #3d9cf0;
      --log-bg: #0a0e12;
      font-family: system-ui, -apple-system, "Segoe UI", sans-serif;
    }
    * { box-sizing: border-box; }
    html {
      height: 100%;
    }
    body {
      margin: 0;
      padding: 0;
      background: var(--bg);
      color: var(--text);
      height: 100dvh;
      max-height: 100dvh;
      overflow: hidden;
      box-sizing: border-box;
    }
    .app-shell {
      display: flex;
      flex-direction: row;
      align-items: stretch;
      gap: 1rem;
      height: 100%;
      min-height: 0;
      max-height: 100%;
      overflow: hidden;
    }
    .app-controls {
      flex: 0 0 auto;
      width: fit-content;
      max-width: 100%;
      min-height: 0;
      overflow-y: auto;
      padding: max(1rem, env(safe-area-inset-top)) max(1rem, env(safe-area-inset-right))
        max(1rem, env(safe-area-inset-bottom)) max(1rem, env(safe-area-inset-left));
      display: flex;
      flex-direction: column;
    }
    .app-viewer {
      flex: 1 1 0;
      min-width: 0;
      min-height: 0;
      display: flex;
      flex-direction: column;
      overflow: hidden;
    }
    #viewer-root {
      flex: 1 1 auto;
      min-height: 0;
      display: flex;
      flex-direction: column;
    }
    @media (max-width: 720px) {
      .app-shell {
        flex-direction: column;
      }
      .app-controls {
        width: 100%;
        flex: 1 1 auto;
      }
      .app-viewer {
        order: -1;
        flex: 0 1 42vh;
        max-height: 42vh;
      }
    }
    .target {
      font-size: clamp(1.75rem, 6vw, 2.25rem);
      font-weight: 700;
      line-height: 1.2;
      margin-bottom: 0.5rem;
    }
    .speed {
      font-size: 1.125rem;
      color: var(--muted);
      margin-bottom: 1.25rem;
      min-height: 1.5em;
    }
    .target-buttons-label {
      font-size: 0.875rem;
      color: var(--muted);
      margin-bottom: 0.5rem;
    }
    .target-buttons {
      display: grid;
      grid-template-columns: repeat(2, max-content);
      gap: 0.5rem;
      margin-bottom: 1.25rem;
      width: max-content;
      max-width: 100%;
    }
    button.target-btn {
      padding: 0.75rem 0.65rem;
      font-size: 0.9375rem;
      font-weight: 600;
      border: 1px solid rgba(61, 156, 240, 0.45);
      border-radius: 10px;
      background: var(--card);
      color: var(--text);
      cursor: pointer;
      touch-action: manipulation;
    }
    button.target-btn.active {
      border-color: var(--accent);
      background: rgba(61, 156, 240, 0.22);
    }
    button.target-btn:active { opacity: 0.85; }
    .time-scale {
      margin-bottom: 1.25rem;
    }
    .time-scale-label {
      font-size: 0.875rem;
      color: var(--muted);
      margin-bottom: 0.5rem;
    }
    .time-scale-buttons {
      display: flex;
      flex-wrap: wrap;
      gap: 0.5rem;
      width: max-content;
      max-width: 100%;
    }
    button.time-scale {
      flex: 0 0 auto;
      padding: 0.75rem 0.5rem;
      font-size: 0.8125rem;
      font-weight: 600;
      border: 1px solid rgba(61, 156, 240, 0.45);
      border-radius: 10px;
      background: var(--card);
      color: var(--text);
      cursor: pointer;
      touch-action: manipulation;
      white-space: nowrap;
    }
    button.time-scale.active {
      border-color: var(--accent);
      background: rgba(61, 156, 240, 0.22);
    }
    button.time-scale-now {
      border-color: rgba(120, 220, 160, 0.55);
      background: rgba(40, 80, 55, 0.45);
    }
    button.time-scale:active { opacity: 0.85; }
    details.logs-panel {
      width: max-content;
      max-width: 100%;
    }
    details.logs-panel summary.logs-label {
      font-size: 0.875rem;
      color: var(--muted);
      margin-bottom: 0.5rem;
      cursor: pointer;
      list-style: none;
      user-select: none;
    }
    details.logs-panel summary.logs-label::-webkit-details-marker {
      display: none;
    }
    details.logs-panel summary.logs-label::before {
      content: "▸ ";
      display: inline-block;
      width: 1em;
    }
    details.logs-panel[open] summary.logs-label::before {
      content: "▾ ";
    }
    details.logs-panel[open] summary.logs-label {
      margin-bottom: 0.5rem;
    }
    pre.logs {
      margin: 0;
      padding: 0.75rem;
      background: var(--log-bg);
      border-radius: 8px;
      font-size: 0.75rem;
      line-height: 1.45;
      overflow: auto;
      max-height: min(45vh, 18rem);
      min-height: 6rem;
      white-space: pre-wrap;
      word-break: break-word;
    }
  </style>
</head>
<body>
  <div class="app-shell">
  <div class="app-controls">
  <div class="time-scale">
    <div class="time-scale-label">Simulation time</div>
    <div class="time-scale-buttons">
      <button type="button" class="time-scale" data-preset="realtime">Realtime</button>
      <button type="button" class="time-scale" data-preset="minute">1 min / s</button>
      <button type="button" class="time-scale" data-preset="hour">1 hour / s</button>
      <button type="button" class="time-scale" data-preset="day">1 day / s</button>
      <button type="button" class="time-scale" data-preset="month">1 month / s</button>
      <button type="button" class="time-scale time-scale-now" data-preset="now">Now (realtime)</button>
    </div>
  </div>
  <div class="target" id="target">—</div>
  <div class="speed" id="speed"></div>
  <div class="target-buttons-label">Pointing target</div>
  <div class="target-buttons" id="target-buttons">
__TARGET_BUTTONS__
  </div>
  <details class="logs-panel">
    <summary class="logs-label">Log</summary>
    <pre class="logs" id="logs"></pre>
  </details>
  </div>
  <div class="app-viewer">
  <div id="viewer-root"></div>
  <script type="module" src="__VIEWER_SCRIPT__"></script>
  </div>
  </div>
  <script>
    const targetEl = document.getElementById("target");
    const speedEl = document.getElementById("speed");
    const logsEl = document.getElementById("logs");
    const timeScaleButtons = document.querySelectorAll("button.time-scale");
    const targetButtons = document.querySelectorAll("button.target-btn");

    function applyTargetUi(data) {
      targetEl.textContent = data.target_label;
      for (const btn of targetButtons) {
        btn.classList.toggle("active", btn.dataset.target === data.target);
      }
    }

    function applyTimeScaleUi(data) {
      for (const btn of timeScaleButtons) {
        const preset = btn.dataset.preset;
        const active =
          preset === "now"
            ? data.preset === "realtime"
            : preset === data.preset;
        btn.classList.toggle("active", active);
      }
    }

    async function refreshTimeScale() {
      const res = await fetch("/api/time-scale");
      const data = await res.json();
      applyTimeScaleUi(data);
    }

    async function setTimeScale(preset) {
      for (const btn of timeScaleButtons) {
        btn.disabled = true;
      }
      try {
        await fetch("/api/time-scale", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ preset }),
        });
        await refreshTimeScale();
      } finally {
        for (const btn of timeScaleButtons) {
          btn.disabled = false;
        }
      }
    }

    for (const btn of timeScaleButtons) {
      btn.addEventListener("click", () => setTimeScale(btn.dataset.preset));
    }

    async function refresh() {
      const res = await fetch("/api/status");
      const data = await res.json();
      applyTargetUi(data);
      if (data.speed_km_h != null) {
        speedEl.textContent = "Surface speed: " + data.speed_km_h.toFixed(2) + " km/h";
      } else {
        speedEl.textContent = "";
      }
      logsEl.textContent = (data.logs || []).join("\\n");
      logsEl.scrollTop = logsEl.scrollHeight;
    }

    async function setTarget(target) {
      for (const btn of targetButtons) {
        btn.disabled = true;
      }
      try {
        await fetch("/api/target", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ target }),
        });
        await refresh();
      } finally {
        for (const btn of targetButtons) {
          btn.disabled = false;
        }
      }
    }

    for (const btn of targetButtons) {
      btn.addEventListener("click", () => setTarget(btn.dataset.target));
    }

    refresh();
    refreshTimeScale();
    setInterval(refresh, __POLL_MS__);
    setInterval(refreshTimeScale, 500);

    let navigatorSession = null;
    async function pollNavigatorSession() {
      try {
        const res = await fetch("/api/navigator-session");
        if (!res.ok) {
          return;
        }
        const data = await res.json();
        if (navigatorSession === null) {
          navigatorSession = data.session;
          return;
        }
        if (data.session !== navigatorSession) {
          location.reload();
        }
      } catch {
      }
    }
    pollNavigatorSession();
    setInterval(pollNavigatorSession, __SESSION_POLL_MS__);
  </script>
</body>
</html>
"""


class LogBuffer:
    def __init__(self, *, maxlen: int = LOG_LINE_LIMIT) -> None:
        self._lines: deque[str] = deque(maxlen=maxlen)

    def append_line(self, line: str) -> None:
        if line:
            self._lines.append(line)

    def lines(self) -> list[str]:
        return list(self._lines)


@contextmanager
def capture_stdout(log_buffer: LogBuffer) -> Iterator[None]:
    original = sys.stdout
    pending = ""

    def on_line(line: str) -> None:
        log_buffer.append_line(line)

    class Tee:
        def write(self, data: str) -> None:
            nonlocal pending
            original.write(data)
            pending += data
            while "\n" in pending:
                line, pending = pending.split("\n", 1)
                on_line(line)

        def flush(self) -> None:
            nonlocal pending
            original.flush()
            if pending:
                on_line(pending)
                pending = ""

        def isatty(self) -> bool:
            return original.isatty()

    sys.stdout = Tee()
    try:
        yield
    finally:
        sys.stdout = original


def begin_navigator_session() -> str:
    global _navigator_session_id
    _navigator_session_id = uuid.uuid4().hex
    return _navigator_session_id


def navigator_session_id() -> str:
    if _navigator_session_id is None:
        begin_navigator_session()
    assert _navigator_session_id is not None
    return _navigator_session_id


def _target_buttons_html() -> str:
    lines: list[str] = []
    for target in PointingTarget:
        lines.append(
            f'    <button type="button" class="target-btn" data-target="{target.value}">{target.label}</button>'
        )
    return "\n".join(lines)


def _index_html() -> str:
    session = navigator_session_id()
    return (
        _INDEX_HTML.replace("__VIEWER_SCRIPT__", f"/viewer.js?v={session}")
        .replace("__POLL_MS__", str(_STATUS_POLL_MS))
        .replace("__SESSION_POLL_MS__", str(_SESSION_POLL_MS))
        .replace("__TARGET_BUTTONS__", _target_buttons_html())
    )


def _request_path(path: str) -> str:
    return path.split("?", 1)[0]


def _status_payload(buttons: ButtonTargetSource, log_buffer: LogBuffer) -> dict[str, object]:
    target = buttons.selected_button["target"]
    _surface, _euler, speed_au_s = observer_surface_vector_and_euler_angles_for_target(simulation_time(), target)
    speed_km_h: float | None = None
    if target not in _BODY_TARGETS:
        speed_km_s = speed_au_s * (1 * u.au).to_value(u.km)
        speed_km_h = float(speed_km_s * 3600.0)
    return {
        "target": target.value,
        "target_label": target.label,
        "speed_km_h": speed_km_h,
        "logs": log_buffer.lines(),
    }


async def _read_request(reader: asyncio.StreamReader) -> tuple[str, str, bytes]:
    header = await reader.readuntil(b"\r\n\r\n")
    first_line = header.split(b"\r\n", 1)[0].decode("utf-8", errors="replace")
    parts = first_line.split()
    if len(parts) < 2:
        raise ValueError("bad request line")
    method, path = parts[0], parts[1]
    body = b""
    content_length = 0
    for line in header.split(b"\r\n")[1:]:
        if line.lower().startswith(b"content-length:"):
            content_length = int(line.split(b":", 1)[1].strip())
            break
    if content_length > 0:
        body = await reader.readexactly(content_length)
    return method, path, body


def _apply_time_scale_preset(preset: str) -> None:
    if preset == "now":
        sync_to_realtime()
        return
    set_time_scale_preset(preset)


def _viewer_asset(path: str) -> tuple[bytes, str] | None:
    path = _request_path(path)
    if not path.startswith("/"):
        return None
    relative = path.lstrip("/")
    if not relative or any(part == ".." for part in relative.split("/")):
        return None
    file_path = (_VIEWER_DIST / relative).resolve()
    dist_root = _VIEWER_DIST.resolve()
    if not str(file_path).startswith(str(dist_root)) or not file_path.is_file():
        return None
    content_type = mimetypes.guess_type(file_path.name)[0] or "application/octet-stream"
    return file_path.read_bytes(), content_type


def _http_response(
    status: int,
    body: bytes,
    content_type: str,
    *,
    extra_headers: dict[str, str] | None = None,
) -> bytes:
    reason = {
        200: "OK",
        204: "No Content",
        400: "Bad Request",
        404: "Not Found",
        405: "Method Not Allowed",
    }[status]
    header = f"HTTP/1.1 {status} {reason}\r\nContent-Type: {content_type}\r\nContent-Length: {len(body)}\r\n"
    for key, value in (extra_headers or {}).items():
        header += f"{key}: {value}\r\n"
    header += "Connection: close\r\n\r\n"
    return header.encode("ascii") + body


async def _handle_client(
    reader: asyncio.StreamReader,
    writer: asyncio.StreamWriter,
    buttons: ButtonTargetSource,
    log_buffer: LogBuffer,
) -> None:
    try:
        method, path, body = await _read_request(reader)
        route = _request_path(path)
        if route == "/" and method == "GET":
            body = _index_html().encode("utf-8")
            writer.write(
                _http_response(
                    200,
                    body,
                    "text/html; charset=utf-8",
                    extra_headers={"Cache-Control": "no-cache"},
                )
            )
        elif method == "GET" and (asset := _viewer_asset(path)) is not None:
            body, content_type = asset
            writer.write(
                _http_response(
                    200,
                    body,
                    content_type,
                    extra_headers={"Cache-Control": "no-cache"},
                )
            )
        elif route == "/api/navigator-session" and method == "GET":
            payload = json.dumps({"session": navigator_session_id()}).encode("utf-8")
            writer.write(_http_response(200, payload, "application/json"))
        elif route == "/api/status" and method == "GET":
            payload = json.dumps(_status_payload(buttons, log_buffer)).encode("utf-8")
            writer.write(_http_response(200, payload, "application/json"))
        elif route == "/api/scene" and method == "GET":
            target = buttons.selected_button["target"]
            payload = json.dumps(scene_snapshot_payload(target)).encode("utf-8")
            writer.write(_http_response(200, payload, "application/json"))
        elif route == "/api/time-scale" and method == "GET":
            payload = json.dumps(time_scale_status_payload()).encode("utf-8")
            writer.write(_http_response(200, payload, "application/json"))
        elif route == "/api/time-scale" and method == "POST":
            try:
                data = json.loads(body.decode("utf-8"))
                preset = data["preset"]
                if not isinstance(preset, str):
                    raise ValueError("preset must be a string")
                _apply_time_scale_preset(preset)
            except (KeyError, ValueError, json.JSONDecodeError):
                writer.write(_http_response(400, b"bad request", "text/plain"))
            else:
                payload = json.dumps(time_scale_status_payload()).encode("utf-8")
                writer.write(_http_response(200, payload, "application/json"))
        elif route == "/api/target" and method == "POST":
            try:
                data = json.loads(body.decode("utf-8"))
                target_value = data["target"]
                if not isinstance(target_value, str):
                    raise ValueError("target must be a string")
                buttons.select_target(PointingTarget(target_value))
            except (KeyError, ValueError, json.JSONDecodeError):
                writer.write(_http_response(400, b"bad request", "text/plain"))
            else:
                writer.write(_http_response(204, b"", "text/plain"))
        elif method == "GET":
            writer.write(_http_response(404, b"Not found", "text/plain"))
        else:
            writer.write(_http_response(405, b"Method not allowed", "text/plain"))
    except (asyncio.IncompleteReadError, ConnectionResetError, ValueError):
        pass
    finally:
        writer.close()
        with suppress(Exception):
            await writer.wait_closed()


@asynccontextmanager
async def run_web_ui(buttons: ButtonTargetSource, log_buffer: LogBuffer) -> AsyncIterator[None]:
    async def client_handler(reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        await _handle_client(reader, writer, buttons, log_buffer)

    server = await asyncio.start_server(client_handler, WEB_HOST, WEB_PORT)
    addrs = ", ".join(str(sock.getsockname()) for sock in server.sockets or ())
    print(f"Navigator web UI at http://{addrs}")
    try:
        yield
    finally:
        server.close()
        await server.wait_closed()
