import asyncio

from pybricks.parameters import Port

from hub_client import MoveHub, Motor
from hub_client.starpoint.buttons import ButtonData, StarpointButtons
from simulate.astronomy.constants import ObserverFrame
from simulate.astronomy.utils.ephemeris import current_time, observer_surface_vector_and_euler_angles_for_frame

INACTIVITY_REFRESH_S = 10.0


def clamp_yaw(yaw: float) -> float:
    return min(yaw % 360, 340)

def clamp_pitch(pitch: float) -> float:
    return max(min(pitch, 90), -90)

def get_motors(hub: MoveHub) -> tuple[Motor, Motor]:
    return hub.motor(Port.B), hub.motor(Port.C)

async def calibrate_motors(hub: MoveHub) -> None:
    motorPan, motorTilt = get_motors(hub)
    # rotate motors until stuck
    # duty_limit 50 for non-geared base
    pan_angle = await motorPan.run_until_stalled(-150, duty_limit=100)
    tilt_angle = await motorTilt.run_until_stalled(-100, duty_limit=50)

    print(f"Pan motor stalled at angle {pan_angle} degrees.")
    print(f"Tilt motor stalled at angle {tilt_angle} degrees.")

    # Define the stall point as zero
    await motorPan.reset_angle(-10)
    await motorTilt.reset_angle(-90)

async def point_at_frame(hub: MoveHub, frame: ObserverFrame) -> None:
    print(f"Pointing at frame: {frame.label}")
    motorPan = hub.motor(Port.B)
    motorTilt = hub.motor(Port.C)

    _surface, (yaw, pitch, _roll), _speed = observer_surface_vector_and_euler_angles_for_frame(
        current_time(), frame
    )

    print(f"Raw yaw: {yaw} degrees. Pitch: {pitch} degrees.")

    yaw = clamp_yaw(yaw)
    pitch = clamp_pitch(pitch)

    print(f"Clamped yaw: {yaw} degrees. Pitch: {pitch} degrees.")

    await motorPan.run_target(150, target_angle=yaw)
    pan_angle = await motorPan.angle()
    print(f"Pan motor angle: {pan_angle} degrees.")

    await motorTilt.run_target(200, target_angle=pitch)
    tilt_angle = await motorTilt.angle()
    print(f"Tilt motor angle: {tilt_angle} degrees.")


async def star_point_main(upload_program: bool = False) -> None:
    print("Star point main")

    async with MoveHub.connect(program="pybricks/main.py" if upload_program else None) as hub:
        await hub.ping()
        print("Hub responded to ping.")

        await calibrate_motors(hub)

        buttons = StarpointButtons()
        activity = asyncio.Event()
        point_lock = asyncio.Lock()

        async def point_selected() -> None:
            async with point_lock:
                await point_at_frame(hub, buttons.selected_button["frame"])

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


if __name__ == "__main__":
    asyncio.run(star_point_main(upload_program=False))
