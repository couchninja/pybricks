import asyncio
from collections.abc import AsyncIterator
from contextlib import AsyncExitStack, asynccontextmanager

from pybricks.parameters import Port

from gpio.button_menu import ButtonData, ButtonMenu
from pybricks_client import Motor, MotorStalledError, MoveHub
from pybricks_client.ble import RECOVERABLE_ERRORS, format_error
from simulate.astronomy.constants import PointingTarget
from simulate.astronomy.utils.ephemeris import (
    current_time,
    observer_surface_vector_and_euler_angles_for_target,
)

INACTIVITY_REFRESH_S = 10.0
RECONNECT_DELAY_S = 2.0
PAN_DUTY_LIMIT = 150
TILT_DUTY_LIMIT = 200
MIN_TARGET_ANGLE_DELTA = 2
# Make sure this matches the ports in pybricks_hub/thin_ble_hub.py and the script is on the brick
EXTERNAL_MOTOR_PORT = Port.D
SENSOR_PORT = Port.C

_last_target_angles: dict[str, float] = {}

def clamp_yaw(yaw: float) -> float:
    return min(yaw % 360, 340)


def clamp_pitch(pitch: float) -> float:
    return max(min(pitch, 90), -90)


def get_motors(hub: MoveHub) -> tuple[Motor, Motor]:
    return hub.motor(Port.A), hub.motor(EXTERNAL_MOTOR_PORT)


def clear_last_target(motor: Motor) -> None:
    _last_target_angles.pop(motor.port.name, None)


async def calibrate_motors(hub: MoveHub) -> None:
    motor_pan, motor_tilt = get_motors(hub)
    _last_target_angles.clear()
    # duty_limit 50 for non-geared base
    pan_angle = await motor_pan.run_until_stalled(-150, duty_limit=PAN_DUTY_LIMIT)
    print(f"Pan motor stalled at angle {pan_angle:.1f} degrees.")
    await motor_pan.reset_angle(10)
    clear_last_target(motor_pan)
    await run_target_or_warn(motor_pan, PAN_DUTY_LIMIT, 180, "Pan")

    tilt_angle = await motor_tilt.run_until_stalled(-100, duty_limit=TILT_DUTY_LIMIT)
    print(f"Tilt motor stalled at angle {tilt_angle:.1f} degrees.")
    await motor_tilt.reset_angle(-85)
    clear_last_target(motor_tilt)


async def run_target_or_warn(
    motor: Motor, speed: float, target_angle: float, label: str
) -> None:
    port_name = motor.port.name
    last_target = _last_target_angles.get(port_name)
    if (
        last_target is not None
        and abs(target_angle - last_target) < MIN_TARGET_ANGLE_DELTA
    ):
        print(
            f"{label} motor: skipped (within {MIN_TARGET_ANGLE_DELTA:.1f}° of last target)"
        )
        return

    _last_target_angles[port_name] = target_angle
    try:
        await motor.run_target(speed, target_angle=target_angle)
    except MotorStalledError:
        actual = await motor.angle()
        print(
            f"Warning: {label} motor stalled at {actual:.1f}° "
            f"(target {target_angle:.1f}°)"
        )
        return

    print(f"{label} motor: moving to {target_angle:.1f}°")


async def point_at_target(hub: MoveHub, target: PointingTarget) -> None:
    print(f"Pointing at target: {target.label}")
    motor_pan, motor_tilt = get_motors(hub)

    _surface, (yaw, pitch, _roll), _speed = observer_surface_vector_and_euler_angles_for_target(
        current_time(), target
    )

    print(f"Raw yaw: {yaw:.1f} degrees. Pitch: {pitch:.1f} degrees.")
    yaw = clamp_yaw(yaw)
    pitch = clamp_pitch(pitch)
    print(f"Clamped yaw: {yaw:.1f} degrees. Pitch: {pitch:.1f} degrees.")

    await run_target_or_warn(motor_pan, PAN_DUTY_LIMIT, yaw, "Pan")
    print(f"Pan motor angle: {(await motor_pan.angle()):.1f} degrees.")

    await run_target_or_warn(motor_tilt, TILT_DUTY_LIMIT, pitch, "Tilt")
    print(f"Tilt motor angle: {(await motor_tilt.angle()):.1f} degrees.")


@asynccontextmanager
async def connected_hub(
    buttons: ButtonMenu, program: str | None
) -> AsyncIterator[MoveHub]:
    async with AsyncExitStack() as stack:
        async with buttons.blinking(0):
            while True:
                try:
                    hub = await stack.enter_async_context(
                        MoveHub.connect(program=program, retries=1)
                    )
                    break
                except RECOVERABLE_ERRORS as exc:
                    print(
                        f"Connection failed ({format_error(exc)}); searching again..."
                    )
                    await asyncio.sleep(RECONNECT_DELAY_S)
        yield hub


async def run_selection_loop(hub: MoveHub, buttons: ButtonMenu) -> None:
    sensor = hub.color_distance_sensor(SENSOR_PORT)
    activity = asyncio.Event()
    lock = asyncio.Lock()

    async def point_selected() -> None:
        async with lock:
            selected = buttons.selected_button
            print(f"Button: {selected}")
            await sensor.light.on(selected["color"])
            await point_at_target(hub, selected["target"])

    async def on_target_selected(button: ButtonData) -> None:
        activity.set()
        print(f"Target: {button['target'].label}")
        await point_selected()

    async def refresh_on_inactivity() -> None:
        while True:
            activity.clear()
            try:
                await asyncio.wait_for(activity.wait(), timeout=INACTIVITY_REFRESH_S)
            except TimeoutError:
                await point_selected()

    buttons.on_selection_changed(on_target_selected)
    try:
        buttons.apply_entry_mode()
        await point_selected()
        await asyncio.gather(buttons.run(), refresh_on_inactivity())
    finally:
        try:
            await sensor.light.off()
        except RECOVERABLE_ERRORS:
            pass


async def navigator_main(upload_program: bool = False) -> None:
    print("Navigator main")
    program = "pybricks_hub/thin_ble_hub.py" if upload_program else None

    with ButtonMenu() as buttons:
        while True:
            try:
                async with connected_hub(buttons, program) as hub:
                    print("Hub connected.")
                    async with buttons.blinking(1):
                        await calibrate_motors(hub)
                    await run_selection_loop(hub, buttons)
            except RECOVERABLE_ERRORS as exc:
                print(f"Hub lost ({format_error(exc)}); searching again...")
                await asyncio.sleep(RECONNECT_DELAY_S)


if __name__ == "__main__":
    # asyncio.run(navigator_main(upload_program=True))
    asyncio.run(navigator_main(upload_program=False))
