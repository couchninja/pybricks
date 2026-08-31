import asyncio
from collections.abc import AsyncIterator
from contextlib import AsyncExitStack, asynccontextmanager

from pybricks.parameters import Port

from gpio.button_menu import ButtonData, ButtonMenu
from pybricks_client import Motor, MoveHub
from simulate.astronomy.constants import ObserverFrame
from simulate.astronomy.utils.ephemeris import (
    current_time,
    observer_surface_vector_and_euler_angles_for_frame,
)

INACTIVITY_REFRESH_S = 10.0


def clamp_yaw(yaw: float) -> float:
    return min(yaw % 360, 340)


def clamp_pitch(pitch: float) -> float:
    return max(min(pitch, 90), -90)


def get_motors(hub: MoveHub) -> tuple[Motor, Motor]:
    return hub.motor(Port.B), hub.motor(Port.C)


async def calibrate_motors(hub: MoveHub) -> None:
    motor_pan, motor_tilt = get_motors(hub)
    # duty_limit 50 for non-geared base
    pan_angle = await motor_pan.run_until_stalled(-150, duty_limit=100)
    tilt_angle = await motor_tilt.run_until_stalled(-100, duty_limit=50)

    print(f"Pan motor stalled at angle {pan_angle} degrees.")
    print(f"Tilt motor stalled at angle {tilt_angle} degrees.")

    await motor_pan.reset_angle(-10)
    await motor_tilt.reset_angle(-90)


async def point_at_frame(hub: MoveHub, frame: ObserverFrame) -> None:
    print(f"Pointing at frame: {frame.label}")
    motor_pan, motor_tilt = get_motors(hub)

    _surface, (yaw, pitch, _roll), _speed = (
        observer_surface_vector_and_euler_angles_for_frame(current_time(), frame)
    )

    print(f"Raw yaw: {yaw} degrees. Pitch: {pitch} degrees.")
    yaw = clamp_yaw(yaw)
    pitch = clamp_pitch(pitch)
    print(f"Clamped yaw: {yaw} degrees. Pitch: {pitch} degrees.")

    await motor_pan.run_target(150, target_angle=yaw)
    print(f"Pan motor angle: {await motor_pan.angle()} degrees.")

    await motor_tilt.run_target(200, target_angle=pitch)
    print(f"Tilt motor angle: {await motor_tilt.angle()} degrees.")


@asynccontextmanager
async def connected_hub(
    buttons: ButtonMenu, program: str | None
) -> AsyncIterator[MoveHub]:
    async with AsyncExitStack() as stack:
        async with buttons.blinking(0):
            hub = await stack.enter_async_context(MoveHub.connect(program=program))
        yield hub


async def run_selection_loop(hub: MoveHub, buttons: ButtonMenu) -> None:
    sensor = hub.color_distance_sensor(Port.D)
    activity = asyncio.Event()
    lock = asyncio.Lock()

    async def point_selected() -> None:
        async with lock:
            button = buttons.selected_button
            print(f"Button: {button}")
            await sensor.light.on(button["color"])
            await point_at_frame(hub, button["frame"])

    async def on_frame_selected(button: ButtonData) -> None:
        print(f"Frame: {button['frame'].label}")
        await point_selected()
        activity.set()

    async def refresh_on_inactivity() -> None:
        while True:
            activity.clear()
            try:
                await asyncio.wait_for(activity.wait(), timeout=INACTIVITY_REFRESH_S)
            except TimeoutError:
                await point_selected()

    buttons.on_selection_changed(on_frame_selected)
    await point_selected()
    await asyncio.gather(buttons.run(), refresh_on_inactivity())


async def star_point_main(upload_program: bool = False) -> None:
    print("Star point main")
    program = "pybricks_hub/main.py" if upload_program else None

    with ButtonMenu() as buttons:
        async with connected_hub(buttons, program) as hub:
            print("Hub connected.")
            async with buttons.blinking(1):
                await calibrate_motors(hub)
            await run_selection_loop(hub, buttons)


if __name__ == "__main__":
    asyncio.run(star_point_main(upload_program=False))
