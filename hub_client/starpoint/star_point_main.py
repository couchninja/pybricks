import asyncio

from pybricks.parameters import Port

from hub_client import MoveHub


async def star_point_main(upload_program: bool = False):
    print("Star point main")

    async with MoveHub.connect(
        program="pybricks/main.py" if upload_program else None
    ) as hub:
        await hub.ping()
        print("Hub responded to ping.")

        # pan_gear_ratio = 1 / 5
        pan_gear_ratio = 1
        tilt_gear_ratio = 1

        # rotate motor B until it is stuck
        motorPan = hub.motor(Port.B)
        motorTilt = hub.motor(Port.C)
        # duty_limit 50 for non-geared base
        pan_angle = await motorPan.run_until_stalled(100, duty_limit=100)
        tilt_angle = await motorTilt.run_until_stalled(-100, duty_limit=50)

        print(f"Pan motor stalled at angle {pan_angle * pan_gear_ratio} degrees.")
        print(f"Tilt motor stalled at angle {tilt_angle * tilt_gear_ratio} degrees.")

        # Define the stall point as zero
        await motorPan.reset_angle(10)
        await motorTilt.reset_angle(0)

        while True:
            await motorPan.run_target(1200, target_angle=0 / pan_gear_ratio)
            pan_angle = await motorPan.angle()
            print(f"Pan motor angle: {pan_angle * pan_gear_ratio} degrees.")

            await motorTilt.run_target(1200, target_angle=0 / tilt_gear_ratio)
            tilt_angle = await motorTilt.angle()
            print(f"Tilt motor angle: {tilt_angle * tilt_gear_ratio} degrees.")
            await asyncio.sleep(3)

            await motorPan.run_target(1200, target_angle=-180 / pan_gear_ratio)
            pan_angle = await motorPan.angle()
            print(f"Pan motor angle: {pan_angle * pan_gear_ratio} degrees.")

            await motorTilt.run_target(1200, target_angle=180 / tilt_gear_ratio)
            tilt_angle = await motorTilt.angle()
            print(f"Tilt motor angle: {tilt_angle * tilt_gear_ratio} degrees.")
            await asyncio.sleep(3)


if __name__ == "__main__":
    asyncio.run(star_point_main(upload_program=False))
