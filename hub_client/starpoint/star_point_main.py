import asyncio

from pybricks.parameters import Port

from hub_client import MoveHub, Motor
from simulate.astronomy.constants import ObserverMotionMode
from simulate.astronomy.utils.ephemeris import current_time, observer_surface_vector_and_euler_angles_for_mode




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

async def point_at_mode(hub: MoveHub, mode: ObserverMotionMode) -> None:
    motorPan = hub.motor(Port.B)
    motorTilt = hub.motor(Port.C)
    
    _surface, (yaw, pitch, _roll), _speed = observer_surface_vector_and_euler_angles_for_mode(
        current_time(), mode
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
        await point_at_mode(hub, ObserverMotionMode.EARTH_ROTATION)
        await asyncio.sleep(3)
        await point_at_mode(hub, ObserverMotionMode.SUN_ORBIT)
        await asyncio.sleep(3)
        await point_at_mode(hub, ObserverMotionMode.MILKY_WAY_ORBIT)
        await asyncio.sleep(3)
        await point_at_mode(hub, ObserverMotionMode.CMB_DIPOLE)
        await asyncio.sleep(3)


if __name__ == "__main__":
    asyncio.run(star_point_main(upload_program=False))
