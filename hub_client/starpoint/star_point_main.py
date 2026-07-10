from pybricks.parameters import Port
import asyncio

from hub_client import MoveHub


async def star_point_main(upload_program: bool = False):
    print("Star point main")

    async with MoveHub.connect(
        program="pybricks/main.py" if upload_program else None
    ) as hub:
        await hub.ping()
        print("Hub responded to ping.")

        gear_ratio = 1 / 5

        # rotate motor B until it is stuck
        motor = hub.motor(Port.B)
        # duty_limit 50 for non-geared base
        angle = await motor.run_until_stalled(100, duty_limit=50)

        print(f"Motor stalled at angle {angle * gear_ratio} degrees.")

        # Define the stall point as zero
        await motor.reset_angle(20)

        while True:
            await motor.run_target(1200, target_angle=-90 / gear_ratio)
            angle = await motor.angle()
            print(f"Motor angle: {angle * gear_ratio} degrees.")

            await asyncio.sleep(3)
            await motor.run_target(1200, target_angle=-180 / gear_ratio)
            angle = await motor.angle()
            print(f"Motor angle: {angle * gear_ratio} degrees.")
            await asyncio.sleep(3)


if __name__ == "__main__":
    asyncio.run(star_point_main(upload_program=False))
