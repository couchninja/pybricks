import asyncio

from pybricks.parameters import Port

from navigator.navigator_main import PAN_DUTY_LIMIT, TILT_DUTY_LIMIT, calibrate_motors, get_motors
from pybricks_client import MoveHub


async def motion_range(upload_program: bool = False) -> None:
    print("Motion range main")

    async with MoveHub.connect(program="pybricks_hub/thin_ble_hub.py" if upload_program else None) as hub:
        await hub.ping()
        print("Hub responded to ping.")

        await calibrate_motors(hub)

        motor_pan, motor_tilt = get_motors(hub)

        while True:
            await motor_pan.run_target(PAN_DUTY_LIMIT, target_angle=0)
            pan_angle = await motor_pan.angle()
            print(f"Pan motor angle: {pan_angle} degrees.")

            await motor_tilt.run_target(TILT_DUTY_LIMIT, target_angle=-90)
            tilt_angle = await motor_tilt.angle()
            print(f"Tilt motor angle: {tilt_angle} degrees.")
            await asyncio.sleep(3)

            await motor_tilt.run_target(TILT_DUTY_LIMIT, target_angle=90)
            tilt_angle = await motor_tilt.angle()
            print(f"Tilt motor angle: {tilt_angle} degrees.")
            await asyncio.sleep(3)


            await motor_pan.run_target(PAN_DUTY_LIMIT, target_angle=340)
            pan_angle = await motor_pan.angle()
            print(f"Pan motor angle: {pan_angle} degrees.")

            await motor_tilt.run_target(TILT_DUTY_LIMIT, target_angle=-90)
            tilt_angle = await motor_tilt.angle()
            print(f"Tilt motor angle: {tilt_angle} degrees.")
            await asyncio.sleep(3)

            await motor_tilt.run_target(TILT_DUTY_LIMIT, target_angle=90)
            tilt_angle = await motor_tilt.angle()
            print(f"Tilt motor angle: {tilt_angle} degrees.")
            await asyncio.sleep(3)


if __name__ == "__main__":
    asyncio.run(motion_range(upload_program=False))
