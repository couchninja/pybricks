"""Thin BLE command server for the Move Hub.

Commands arrive on stdin over the existing GATT connection
(``PybricksHub.write_line`` on the host). Responses go to stdout so the
host can read them with ``read_line``.

Move Hub has no ``usys.stdin`` / blocking ``input()``; use
``read_input_byte`` instead.
"""

from pybricks.hubs import MoveHub
from pybricks.parameters import Color, Direction, Port, Stop
from pybricks.pupdevices import ColorDistanceSensor, Motor
from pybricks.tools import read_input_byte, wait

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


def parse_stop(value):
    if value in ("COAST", "BRAKE", "HOLD"):
        return Stop[value]
    return None


def handle_command(seq, cmd, arg1="", arg2="", arg3="", arg4=""):
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

        if cmd.startswith("mtr."):
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
            elif action in ("rset", "reset_angle"):
                motor.stop()
                if arg2:
                    motor.reset_angle(parse_int(arg2))
                else:
                    motor.reset_angle()
            elif action == "rang":
                speed = parse_int(arg2)
                rotation_angle = parse_int(arg3)
                then = parse_stop(arg4) or Stop.HOLD
                motor.run_angle(speed, rotation_angle, then)
            elif action == "rtgt":
                speed = parse_int(arg2)
                target_angle = parse_int(arg3)
                then = parse_stop(arg4) or Stop.HOLD
                motor.run_target(speed, target_angle, then)
            elif action == "stall":
                speed = parse_int(arg2)
                then = Stop.COAST
                duty_limit = None
                stop = parse_stop(arg3)
                if stop is not None:
                    then = stop
                elif arg3:
                    duty_limit = parse_int(arg3)
                stop = parse_stop(arg4)
                if stop is not None:
                    then = stop
                if duty_limit is not None:
                    angle = motor.run_until_stalled(speed, then, duty_limit)
                else:
                    angle = motor.run_until_stalled(speed, then)
                reply(seq, "val " + str(angle))
                return
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


def read_line():
    # Move Hub: read_input_byte(chr=True) does not work; use int + chr().
    buf = ""
    while True:
        b = read_input_byte()
        if b is None:
            wait(10)
            continue
        if b == 10 or b == 13:
            if buf:
                return buf
            continue
        if b >= 32 and b < 127:
            buf += chr(b)


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
    line = read_line()
    parts = line.split()
    if len(parts) < 2:
        continue

    if len(parts) > 5:
        handle_command(parts[0], parts[1], parts[2], parts[3], parts[4], parts[5])
    elif len(parts) > 4:
        handle_command(parts[0], parts[1], parts[2], parts[3], parts[4])
    elif len(parts) > 3:
        handle_command(parts[0], parts[1], parts[2], parts[3])
    elif len(parts) > 2:
        handle_command(parts[0], parts[1], parts[2])
    else:
        handle_command(parts[0], parts[1])
