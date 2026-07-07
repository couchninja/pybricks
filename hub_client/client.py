"""Pybricks-like API that forwards commands to a hub over BLE."""

from __future__ import annotations

import asyncio
import time
from contextlib import asynccontextmanager
from typing import AsyncIterator

from pybricks.parameters import Color, Direction, Port
from pybricksdev.connections.pybricks import PybricksHubBLE

from hub_client.ble import RECOVERABLE_ERRORS, connect, disconnect_hub
from hub_client.broadcast import CommandBroadcaster, open_broadcaster


class HubClientError(RuntimeError):
    """Raised when the hub returns an error response."""


def _color_name(color: Color) -> str:
    for name, known in Color.__dict__.items():
        if isinstance(known, Color) and color == known:
            return name
    raise HubClientError(f"unsupported color: {color!r}")


class _CommandSession:
    def __init__(self, broadcaster: CommandBroadcaster, ble_hub: PybricksHubBLE):
        self._broadcaster = broadcaster
        self._ble_hub = ble_hub
        self._lock = asyncio.Lock()
        self._seq = 0

    async def call(self, command: str) -> str | int | Color | None:
        async with self._lock:
            self._seq += 1
            seq = str(self._seq)
            await self._broadcaster.broadcast(f"{seq} {command}")

            deadline = time.monotonic() + 10.0
            while time.monotonic() < deadline:
                try:
                    line = (await self._ble_hub.read_line()).strip()
                except Exception:
                    await asyncio.sleep(0.05)
                    continue

                if line.startswith("\x06"):
                    line = line[1:].strip()
                if line.startswith("Traceback"):
                    raise HubClientError("hub program error (see hub output)")

                parts = line.split(None, 1)
                if not parts or parts[0] != seq:
                    continue

                payload = parts[1] if len(parts) > 1 else ""
                return self._parse_response(payload)

            raise HubClientError("hub response timeout")

    @staticmethod
    def _parse_response(line: str) -> str | int | Color | None:
        line = line.strip()
        if line == "ok":
            return None
        if line.startswith("val "):
            return int(line[4:])
        if line.startswith("color "):
            return getattr(Color, line[6:])
        if line.startswith("err "):
            raise HubClientError(line[4:])
        raise HubClientError(f"unexpected hub response: {line!r}")


class HubLight:
    def __init__(self, session: _CommandSession):
        self._session = session

    async def on(self, color: Color) -> None:
        await self._session.call(f"light.on {_color_name(color)}")

    async def off(self) -> None:
        await self._session.call("light.off")


class Motor:
    def __init__(
        self,
        session: _CommandSession,
        port: Port,
        positive_direction: Direction = Direction.CLOCKWISE,
    ):
        self._session = session
        self.port = port
        self.positive_direction = positive_direction

    async def dc(self, duty: int | float) -> None:
        await self._session.call(f"motor.dc {self.port.name} {int(duty)}")

    async def run(self, speed: int | float) -> None:
        await self._session.call(f"motor.run {self.port.name} {int(speed)}")

    async def stop(self) -> None:
        await self._session.call(f"motor.stop {self.port.name}")

    async def brake(self) -> None:
        await self._session.call(f"motor.brake {self.port.name}")

    async def angle(self) -> int:
        result = await self._session.call(f"motor.angle {self.port.name}")
        assert isinstance(result, int)
        return result

    async def speed(self) -> int:
        result = await self._session.call(f"motor.speed {self.port.name}")
        assert isinstance(result, int)
        return result

    async def reset_angle(self, angle: int | None = None) -> None:
        if angle is None:
            await self._session.call(f"motor.reset_angle {self.port.name}")
        else:
            await self._session.call(f"motor.reset_angle {self.port.name} {int(angle)}")


class ColorDistanceSensor:
    def __init__(self, session: _CommandSession, port: Port):
        self._session = session
        self.port = port

    async def distance(self) -> int:
        result = await self._session.call(f"sensor.distance {self.port.name}")
        assert isinstance(result, int)
        return result

    async def color(self) -> Color:
        result = await self._session.call(f"sensor.color {self.port.name}")
        assert isinstance(result, Color)
        return result

    async def reflection(self) -> int:
        result = await self._session.call(f"sensor.reflection {self.port.name}")
        assert isinstance(result, int)
        return result

    async def ambient(self) -> int:
        result = await self._session.call(f"sensor.ambient {self.port.name}")
        assert isinstance(result, int)
        return result


class MoveHub:
    """Remote Move Hub with Pybricks-like device accessors."""

    _session: _CommandSession

    def __init__(self, session: _CommandSession):
        self._session = session
        self.light = HubLight(session)

    async def ping(self) -> None:
        await self._session.call("ping")

    def motor(
        self,
        port: Port,
        positive_direction: Direction = Direction.CLOCKWISE,
    ) -> Motor:
        return Motor(self._session, port, positive_direction)

    def color_distance_sensor(self, port: Port) -> ColorDistanceSensor:
        return ColorDistanceSensor(self._session, port)

    @classmethod
    @asynccontextmanager
    async def connect(
        cls,
        name: str | None = None,
        *,
        program: str | None = "pybricks/main.py",
        retries: int = 5,
    ) -> AsyncIterator[MoveHub]:
        """Deploy over GATT, stay connected; commands via broadcast, replies via stdout."""
        last_error: Exception | None = None
        for attempt in range(1, retries + 1):
            try:
                async with cls._open_session(
                    name, program=program, retries=retries
                ) as remote:
                    yield remote
                return
            except (*RECOVERABLE_ERRORS, HubClientError) as exc:
                last_error = exc
                if attempt < retries:
                    print(f"Session failed ({exc}); retrying ({attempt}/{retries})...")
                    await asyncio.sleep(2.0)

        if last_error is not None:
            raise last_error
        raise RuntimeError("failed to connect to hub")

    @classmethod
    @asynccontextmanager
    async def _open_session(
        cls,
        name: str | None,
        *,
        program: str | None,
        retries: int,
    ) -> AsyncIterator[MoveHub]:
        ble_hub = await connect(name, retries=retries)
        try:
            if program is not None:
                try:
                    await ble_hub.stop_user_program()
                    await asyncio.sleep(0.5)
                except Exception:
                    pass
                await ble_hub.download(program)

            await ble_hub.run(
                wait=False,
                print_output=False,
                line_handler=True,
            )
            await asyncio.sleep(0.5)

            ready = False
            for _ in range(30):
                try:
                    line = (await ble_hub.read_line()).strip()
                    if line == "ready" or line.endswith("ready"):
                        ready = True
                        break
                except Exception:
                    await asyncio.sleep(0.1)
            if not ready:
                raise HubClientError("hub did not report ready")

            async with open_broadcaster() as broadcaster:
                session = _CommandSession(broadcaster, ble_hub)
                remote = cls(session)
                await remote.ping()
                try:
                    yield remote
                finally:
                    try:
                        await ble_hub.stop_user_program()
                    except Exception:
                        pass
        finally:
            try:
                await disconnect_hub(ble_hub)
            except Exception:
                pass
