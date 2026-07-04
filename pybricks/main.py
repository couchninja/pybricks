"""Thin BLE command server for the Move Hub.

Commands arrive on COMMAND_CHANNEL via BLERadio. Responses go to stdout
(GATT) so the host can read them without BLE scanning.
"""

from pybricks.hubs import MoveHub
from pybricks.messaging import BLERadio
from pybricks.parameters import Color, Direction, Port
from pybricks.pupdevices import ColorDistanceSensor, Motor
from pybricks.tools import wait

COMMAND_CHANNEL = 7

MOTOR_PORTS = {
    "A": (Port.A, Direction.CLOCKWISE),
    "B": (Port.B, Direction.COUNTERCLOCKWISE),
    "C": (Port.C, Direction.COUNTERCLOCKWISE),
}

SENSOR_PORTS = {
    "D": Port.D,
}

COLORS = {
    "NONE": Color.NONE,
    "BLACK": Color.BLACK,
    "GRAY": Color.GRAY,
    "WHITE": Color.WHITE,
    "RED": Color.RED,
    "ORANGE": Color.ORANGE,
    "BROWN": Color.BROWN,
    "YELLOW": Color.YELLOW,
    "GREEN": Color.GREEN,
    "CYAN": Color.CYAN,
    "BLUE": Color.BLUE,
    "VIOLET": Color.VIOLET,
    "MAGENTA": Color.MAGENTA,
}


def parse_int(value):
    return int(value)


def color_name(color):
    name = str(color)
    if "." in name:
        return name.split(".")[-1]
    return name


def reply(seq, line):
    print(str(seq) + " " + line)


def command_line(cmd):
    if isinstance(cmd, tuple):
        return " ".join(str(part) for part in cmd)
    return str(cmd)


def handle_command(seq, cmd, arg1="", arg2=""):
    # Move Hub MicroPython can raise TypeError when list slices are passed as
    # function arguments after motors/sensors are initialized.
    seq = str(seq)
    cmd = str(cmd)

    try:
        if cmd == "ping":
            reply(seq, "ok")
            return

        if cmd == "light.on":
            hub.light.on(COLORS[arg1])
            reply(seq, "ok")
            return

        if cmd == "light.off":
            hub.light.off()
            reply(seq, "ok")
            return

        if cmd.startswith("motor."):
            port = arg1
            motor = motors[port]
            action = cmd.split(".", 1)[1]

            if action == "dc":
                motor.dc(parse_int(arg2))
            elif action == "run":
                motor.run(parse_int(arg2))
            elif action == "stop":
                motor.stop()
            elif action == "brake":
                motor.brake()
            elif action == "angle":
                reply(seq, "val " + str(motor.angle()))
                return
            elif action == "speed":
                reply(seq, "val " + str(motor.speed()))
                return
            elif action == "reset_angle":
                if arg2:
                    motor.reset_angle(parse_int(arg2))
                else:
                    motor.reset_angle()
            else:
                reply(seq, "err unknown motor command")
                return

            reply(seq, "ok")
            return

        if cmd.startswith("sensor."):
            port = arg1
            sensor = sensors[port]
            action = cmd.split(".", 1)[1]

            if action == "distance":
                reply(seq, "val " + str(sensor.distance()))
            elif action == "color":
                reply(seq, "color " + color_name(sensor.color()))
            elif action == "reflection":
                reply(seq, "val " + str(sensor.reflection()))
            elif action == "ambient":
                reply(seq, "val " + str(sensor.ambient()))
            else:
                reply(seq, "err unknown sensor command")
            return

        reply(seq, "err unknown command")
    except KeyError:
        reply(seq, "err unknown port or color")
    except (IndexError, ValueError):
        reply(seq, "err bad arguments")
    except Exception as exc:
        reply(seq, "err " + type(exc).__name__ + ":" + str(exc))


radio = BLERadio(observe_channels=[COMMAND_CHANNEL])
hub = MoveHub()
motors = {}
for port_name, (port, direction) in MOTOR_PORTS.items():
    motors[port_name] = Motor(port, direction)

sensors = {}
for port_name, port in SENSOR_PORTS.items():
    sensors[port_name] = ColorDistanceSensor(port)

hub.light.on(Color.GREEN)
print("ready")

while True:
    cmd = radio.observe(COMMAND_CHANNEL)
    if cmd is None:
        wait(10)
        continue

    parts = command_line(cmd).split()
    if len(parts) < 2:
        wait(10)
        continue

    if len(parts) > 3:
        handle_command(parts[0], parts[1], parts[2], parts[3])
    elif len(parts) > 2:
        handle_command(parts[0], parts[1], parts[2])
    else:
        handle_command(parts[0], parts[1])
    wait(10)
