from pybricks.hubs import MoveHub
from pybricks.tools import wait
from pybricks.pupdevices import ColorDistanceSensor
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction, Stop, Color, Side


class Program:
    MAX_LOOKING = 60
    SPEED_REDUCE = 2

    hub = MoveHub()
    left_motor = Motor(Port.B, Direction.COUNTERCLOCKWISE)
    right_motor = Motor(Port.A, Direction.CLOCKWISE)
    cam_motor = Motor(Port.C, Direction.COUNTERCLOCKWISE)
    sensor = ColorDistanceSensor(Port.D)

    lookingDirection = -1
    stuckTime = 0
    toppledTime = 0
    toppled = False

    def __init__(self):
        print(self.hub.imu.acceleration())

        self.hub.light.on(Color.RED)

        self.cam_motor.control.limits(speed=None, acceleration=None, torque=30)

        self.cam_motor.run(100)
        while not self.cam_motor.control.stalled():
            wait(10)

        left_angle = self.cam_motor.angle()
        print("left angle:", left_angle)
        self.cam_motor.stop()

        self.cam_motor.run(-100)
        while not self.cam_motor.control.stalled():
            wait(10)

        right_angle = self.cam_motor.angle()
        print("right angle:", right_angle)
        self.cam_motor.stop()
        self.cam_motor.run_target(
            speed=100, target_angle=int((int(left_angle) + int(right_angle)) // 2)
        )
        self.cam_motor.stop()
        self.cam_motor.reset_angle(angle=0)

        self.resume_camera()

    def main_loop(self):
        while True:
            # print("dist:", sensor.distance())
            # print("camera angle:", self.cam_motor.angle())

            if self.hub.imu.up() == Side.TOP:
                self.toppledTime = 0
            else:
                self.toppledTime += 1

            if self.toppledTime < 5:
                # print("untoppled")
                if self.toppled:
                    self.toppled = False
                    self.resume_camera()

                self.run()
            else:
                # print("toppled")
                self.toppled = True
                self.hub.light.on(Color.ORANGE)
                self.left_motor.stop()
                self.right_motor.stop()
                self.cam_motor.run_target(speed=100, target_angle=0)

    def run(self):
        print("left motor load = ", self.cam_motor.control.load())

        if abs(self.MAX_LOOKING * self.lookingDirection - self.cam_motor.angle()) < 3:
            print("switch looking direction", self.lookingDirection)
            self.lookingDirection = -self.lookingDirection
            self.resume_camera()

        # If stuck for a while, drive backwards
        if self.stuckTime > 5:
            self.hub.light.on(Color.RED)
            print("Really stuck!!")

            self.left_motor.dc(-50)
            self.right_motor.dc(-50)
            self.cam_motor.run_target(speed=100, target_angle=0)
            wait(2000)

            self.resume_camera()
            self.stuckTime = 0
            return

        # Wall near or camera motor stalling: Turn
        if (
                self.sensor.distance() <= 80
                or self.cam_motor.control.stalled()
        ):
            self.hub.light.on(Color.BLUE)
            current_cam_pos = 1 if self.cam_motor.angle() > 0 else -1

            self.left_motor.dc(50 * current_cam_pos)
            self.right_motor.dc(-50 * current_cam_pos)
            self.cam_motor.run_target(speed=100, target_angle=0)

            wait(500)

            self.resume_camera()
            self.stuckTime += 1
        # Drive forward
        else:
            self.hub.light.on(Color.GREEN)
            # Probably we cannot detect stalling if we call this every frame
            self.left_motor.dc(100 // self.SPEED_REDUCE)
            self.right_motor.dc(100 // self.SPEED_REDUCE)
            self.stuckTime = 0

        wait(10)

    def resume_camera(self):
        self.cam_motor.run(100 * self.lookingDirection)


Program().main_loop()
