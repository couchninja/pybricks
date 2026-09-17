from __future__ import annotations

import asyncio
import json
import sys
from collections import deque
from collections.abc import AsyncIterator, Iterator
from contextlib import asynccontextmanager, contextmanager, suppress
from typing import TYPE_CHECKING

from astropy import units as u

from simulate.astronomy.constants import PointingTarget
from simulate.astronomy.utils.ephemeris import (
    current_time,
    observer_surface_vector_and_euler_angles_for_target,
)

if TYPE_CHECKING:
    from gpio.button_menu import ButtonMenu

WEB_HOST = "0.0.0.0"
WEB_PORT = 8765
LOG_LINE_LIMIT = 200
_STATUS_POLL_MS = 2000

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
  <title>Navigator</title>
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
    body {
      margin: 0;
      padding: max(1rem, env(safe-area-inset-top)) max(1rem, env(safe-area-inset-right))
        max(1rem, env(safe-area-inset-bottom)) max(1rem, env(safe-area-inset-left));
      background: var(--bg);
      color: var(--text);
      min-height: 100dvh;
    }
    h1 { font-size: 1.25rem; font-weight: 600; margin: 0 0 1rem; color: var(--muted); }
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
    button.cycle {
      width: 100%;
      padding: 1rem 1.25rem;
      font-size: 1.125rem;
      font-weight: 600;
      border: none;
      border-radius: 12px;
      background: var(--accent);
      color: #fff;
      cursor: pointer;
      touch-action: manipulation;
      margin-bottom: 1.25rem;
    }
    button.cycle:active { opacity: 0.85; }
    .logs-label { font-size: 0.875rem; color: var(--muted); margin-bottom: 0.5rem; }
    pre.logs {
      margin: 0;
      padding: 0.75rem;
      background: var(--log-bg);
      border-radius: 8px;
      font-size: 0.75rem;
      line-height: 1.45;
      overflow: auto;
      max-height: 45vh;
      white-space: pre-wrap;
      word-break: break-word;
    }
  </style>
</head>
<body>
  <h1>Navigator</h1>
  <div class="target" id="target">—</div>
  <div class="speed" id="speed"></div>
  <button type="button" class="cycle" id="cycle">Next target</button>
  <div class="logs-label">Log</div>
  <pre class="logs" id="logs"></pre>
  <script>
    const targetEl = document.getElementById("target");
    const speedEl = document.getElementById("speed");
    const logsEl = document.getElementById("logs");
    const cycleBtn = document.getElementById("cycle");

    async function refresh() {
      const res = await fetch("/api/status");
      const data = await res.json();
      targetEl.textContent = data.target_label;
      if (data.speed_km_h != null) {
        speedEl.textContent = "Surface speed: " + data.speed_km_h.toFixed(2) + " km/h";
      } else {
        speedEl.textContent = "";
      }
      logsEl.textContent = (data.logs || []).join("\\n");
      logsEl.scrollTop = logsEl.scrollHeight;
    }

    cycleBtn.addEventListener("click", async () => {
      cycleBtn.disabled = true;
      try {
        await fetch("/api/cycle", { method: "POST" });
        await refresh();
      } finally {
        cycleBtn.disabled = false;
      }
    });

    refresh();
    setInterval(refresh, __POLL_MS__);
  </script>
</body>
</html>
""".replace("__POLL_MS__", str(_STATUS_POLL_MS))


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


def _status_payload(buttons: ButtonMenu, log_buffer: LogBuffer) -> dict[str, object]:
    target = buttons.selected_button["target"]
    _surface, _euler, speed_au_s = observer_surface_vector_and_euler_angles_for_target(
        current_time(), target
    )
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


async def _read_request(reader: asyncio.StreamReader) -> tuple[str, str]:
    header = await reader.readuntil(b"\r\n\r\n")
    first_line = header.split(b"\r\n", 1)[0].decode("utf-8", errors="replace")
    parts = first_line.split()
    if len(parts) < 2:
        raise ValueError("bad request line")
    method, path = parts[0], parts[1]
    return method, path


def _http_response(status: int, body: bytes, content_type: str) -> bytes:
    reason = {200: "OK", 204: "No Content", 404: "Not Found", 405: "Method Not Allowed"}[status]
    header = (
        f"HTTP/1.1 {status} {reason}\r\n"
        f"Content-Type: {content_type}\r\n"
        f"Content-Length: {len(body)}\r\n"
        "Connection: close\r\n"
        "\r\n"
    )
    return header.encode("ascii") + body


async def _handle_client(
    reader: asyncio.StreamReader,
    writer: asyncio.StreamWriter,
    buttons: ButtonMenu,
    log_buffer: LogBuffer,
) -> None:
    try:
        method, path = await _read_request(reader)
        if path == "/" and method == "GET":
            body = _INDEX_HTML.encode("utf-8")
            writer.write(_http_response(200, body, "text/html; charset=utf-8"))
        elif path == "/api/status" and method == "GET":
            payload = json.dumps(_status_payload(buttons, log_buffer)).encode("utf-8")
            writer.write(_http_response(200, payload, "application/json"))
        elif path == "/api/cycle" and method == "POST":
            buttons.cycle_pointing_target()
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
async def run_web_ui(buttons: ButtonMenu, log_buffer: LogBuffer) -> AsyncIterator[None]:
    async def client_handler(
        reader: asyncio.StreamReader, writer: asyncio.StreamWriter
    ) -> None:
        await _handle_client(reader, writer, buttons, log_buffer)

    server = await asyncio.start_server(client_handler, WEB_HOST, WEB_PORT)
    addrs = ", ".join(str(sock.getsockname()) for sock in server.sockets or ())
    print(f"Navigator web UI at http://{addrs}")
    try:
        yield
    finally:
        server.close()
        await server.wait_closed()
