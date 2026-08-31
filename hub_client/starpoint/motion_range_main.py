import asyncio

from pybricks.parameters import Port

from hub_client import MoveHub


async def motion_range(upload_program: bool = False) -> None:
    print("Star point main")

    async with MoveHub.connect(program="pybricks/main.py" if upload_program else None) as hub:
        await hub.ping()
        print("Hub responded to ping.")

        motorPan = hub.motor(Port.B)
        motorTilt = hub.motor(Port.C)
        # rotate motors until stuck
        # duty_limit 50 for non-geared base
        pan_angle = await motorPan.run_until_stalled(-100, duty_limit=100)
        tilt_angle = await motorTilt.run_until_stalled(-100, duty_limit=50)

        print(f"Pan motor stalled at angle {pan_angle} degrees.")
        print(f"Tilt motor stalled at angle {tilt_angle} degrees.")

        # Define the stall point as zero
        await motorPan.reset_angle(-10)
        await motorTilt.reset_angle(0)

        while True:
            await motorPan.run_target(1200, target_angle=0)
            pan_angle = await motorPan.angle()
            print(f"Pan motor angle: {pan_angle} degrees.")

            await motorTilt.run_target(1200, target_angle=0)
            tilt_angle = await motorTilt.angle()
            print(f"Tilt motor angle: {tilt_angle} degrees.")
            await asyncio.sleep(3)

            await motorPan.run_target(1200, target_angle=340)
            pan_angle = await motorPan.angle()
            print(f"Pan motor angle: {pan_angle} degrees.")

            await motorTilt.run_target(1200, target_angle=180)
            tilt_angle = await motorTilt.angle()
            print(f"Tilt motor angle: {tilt_angle} degrees.")
            await asyncio.sleep(3)


if __name__ == "__main__":
    asyncio.run(motion_range(upload_program=False))
