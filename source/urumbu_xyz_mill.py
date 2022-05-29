
# Modified version of Quentin Bolsee 'urumbu_gcode.py'.
# Modified by Robert Hart 1/13/22 - 1/14/22.
# Used 1/14/22 to run Urumbu machine with x,y, and a cam-driven z-axis.
# Supports g-code in a limited way.
#
#
#
import re

import serial
import time
import multiprocessing
import logging
import argparse
import numpy as np
import math
import os

BAUDRATE_DEFAULT = 115200


class Module:
    def __init__(self, port, baudrate=BAUDRATE_DEFAULT):
        self.port = None
        self.baudrate = baudrate
        try:
            self.port = serial.Serial(port, baudrate)
        except serial.SerialException:
            logging.error(f"Cannot connect to {port}")

    @property
    def connected(self):
        return self.port is not None

    def write(self, txt):
        self.port.write(txt)

    def close(self):
        self.port.close()

    def pressed(self, nc=True):
        self.write(b"?")
        r = self.port.read(1)
        if nc:
            return r == b"1"
        else:
            return r == b"0"


class Stepper(Module):
    def __init__(self, steps_per_unit, port, baudrate=BAUDRATE_DEFAULT, reverse=False):
        super().__init__(port, baudrate)
        self.steps = 0
        self.reverse = reverse
        self.steps_per_unit = steps_per_unit

    def step(self, forward):
        self.steps += 1 if forward else -1
        if self.reverse:
            forward = not forward
        self.write(b"f" if forward else b"r")


class Servo(Module):
    def __init__(self, pulse_min, pulse_max, port, baudrate=BAUDRATE_DEFAULT):
        self.pulse_min = pulse_min
        self.pulse_max = pulse_max
        super().__init__(port, baudrate)
        self.delay_us = 0

    def pulse(self, delay_us):
        self.write(delay_us.to_bytes(2, byteorder='little'))

    def fraction(self, f):
        p = int(self.pulse_min + (self.pulse_max - self.pulse_min) * f)
        self.pulse(p)

    def pressed(self, nc=True):
        self.pulse(65535)
        r = self.port.read(1)
        if nc:
            return r == b"1"
        else:
            return r == b"0"


class RGB(Module):
    def __init__(self, port, baudrate=BAUDRATE_DEFAULT):
        super().__init__(port, baudrate)

    def write_rgb(self, red, green, blue):
        rgb = (blue << 16) | (green << 8) | red
        self.write(rgb.to_bytes(3, byteorder="little"))


class Spindle(Module):
    def __init__(self, port, baudrate=BAUDRATE_DEFAULT):
        super().__init__(port, baudrate)

    def set_rpm(self, rpm):
        pass

    def set_on(self, on):
        if on:
            self.write(b"1")
        else:
            self.write(b"0")


class Action:
    def __iter__(self):
        return [self].__iter__()


class HomingAction(Action):
    def __init__(self, axis, name, pos, feedrate, nc=True):
        self.axis = axis
        self.name = name
        self.pos = np.array(pos)
        self.feedrate = feedrate
        self.nc = nc


class PathAction(Action):
    def __call__(self, t):
        raise NotImplementedError()

    def init(self, pos_start):
        raise NotImplementedError()


class WaitAction(Action):
    def __init__(self, dt):
        self.dt = dt

    def __call__(self, dt):
        return dt <= self.dt


class SequenceAction(Action):
    def __init__(self, *sub_actions):
        self.sub_actions = sub_actions

    def __iter__(self):
        return self.sub_actions.__iter__()


class ServoAction(Action):
    def __init__(self, name, pulse, dt=0.01, wait=1.0):
        self.name = name
        self.pulse = pulse
        self.dt = dt
        self.wait = wait


class ColorAction(Action):
    def __init__(self, name, red, green, blue):
        self.name = name
        self.red = red
        self.green = green
        self.blue = blue


class SpindleAction(Action):
    def __init__(self, name, on, rpm=None):
        self.name = name
        self.rpm = rpm
        self.on = on


class Line(PathAction):
    def __init__(self, pos_end, feedrate):
        self.pos_start = np.zeros_like(pos_end)
        self.pos_end = np.array(pos_end)
        self.duration = -1
        self.feedrate = feedrate

    def init(self, pos_start):
        self.pos_start = np.array(pos_start)
        mask_nan = np.isnan(self.pos_end)
        self.pos_end[mask_nan] = self.pos_start[mask_nan]
        self.duration = np.linalg.norm(self.pos_end - self.pos_start) / self.feedrate

    def __call__(self, t):
        if t > self.duration:
            # end move
            return None
        u = t / self.duration
        return self.pos_start * (1 - u) + self.pos_end * u


def transform_corexy(pos, pos_transform):
    pos_transform[:] = pos[:]
    pos_transform[0] = (pos[0] + pos[1])/2
    pos_transform[1] = (pos[0] - pos[1])/2


def modules_manager(action_queue, modules_config, pos_transformer=None):
    logging.info("start loop")

    modules = {}

    modules_axis = {}

    for name, config in modules_config.items():
        if config["type"] == "stepper":
            obj = Stepper(config["steps_per_unit"],
                          config["port"],
                          config["baudrate"],
                          reverse=config.get("reverse", False))
            modules[name] = obj
            if "axis" in config:
                modules_axis[config["axis"]] = obj
        elif config["type"] == "servo":
            modules[name] = Servo(config["pulse_min"],
                                  config["pulse_max"],
                                  config["port"],
                                  config["baudrate"])
        elif config["type"] == "rgb":
            modules[name] = RGB(config["port"], config["baudrate"])
        elif config["type"] == "spindle":
            modules[name] = Spindle(config["port"], config["baudrate"])

    n_axis = len(modules_axis)
    pos = np.zeros((n_axis,))
    pos_motors = np.zeros((n_axis,))

    def tick_motor():
        if pos_transformer is None:
            pos_motors[:] = pos[:]
        else:
            pos_transformer(pos, pos_motors)
        for j in range(n_axis):
            m = modules_axis[j]
            s = int(pos_motors[j] * m.steps_per_unit)
            if m.steps < s:
                m.step(True)
            elif m.steps > s:
                m.step(False)

    while True:
        if not action_queue.empty():
            action = action_queue.get()
            t0 = time.perf_counter()

            for sub_action in action:
                if isinstance(sub_action, PathAction):
                    # time in s, ~us resolution
                    sub_action.init(pos)

                    while True:
                        t = time.perf_counter()

                        # path is a time function
                        pos_new = sub_action(t - t0)

                        if pos_new is None:
                            # done
                            break

                        pos[:] = pos_new[:]
                        tick_motor()
                elif isinstance(sub_action, WaitAction):
                    dt = 0
                    while sub_action(dt):
                        dt = time.perf_counter() - t0
                elif isinstance(sub_action, ServoAction):
                    dt1 = 0
                    t0_pwm = t0
                    action_wait = WaitAction(sub_action.wait)
                    while action_wait(dt1):
                        t1 = time.perf_counter()
                        dt1 = t1 - t0
                        dt2 = t1 - t0_pwm
                        if dt2 >= sub_action.dt:
                            t0_pwm = t1
                            modules[sub_action.name].pulse(sub_action.pulse)
                elif isinstance(sub_action, ColorAction):
                    modules[sub_action.name].write_rgb(sub_action.red,
                                                       sub_action.green,
                                                       sub_action.blue)
                elif isinstance(sub_action, HomingAction):
                    line = Line(sub_action.pos, sub_action.feedrate)
                    line.init(pos)

                    while True:
                        t = time.perf_counter()

                        # path is a time function
                        pos_new = line(t - t0)

                        if pos_new is None:
                            logging.error(f"Homing failed for axis {sub_action.axis}")
                            break

                        pos[:] = pos_new[:]

                        tick_motor()

                        if modules[sub_action.name].pressed(sub_action.nc):
                            logging.info(f"Homing axis {sub_action.axis}")
                            # homing success
                            for i in range(n_axis):
                                motor = modules_axis[i]
                                motor.steps = 0
                            pos[sub_action.axis] = 0
                            break
                elif isinstance(sub_action, SpindleAction):
                    spindle = modules[sub_action.name]
                    spindle.set_on(sub_action.on)
                    spindle.set_rpm(sub_action.rpm)


def parse_arguments():
    usage_text = (
        "Usage:  python urumbu_corexy.py [options]"
    )
    parser = argparse.ArgumentParser(description=usage_text)
    parser.add_argument("-f", "--filename", type=str, required=True,
                        help="filename for .xy file")
    parser.add_argument("--feedrate", type=float, default=5,
                        help="feedrate for XY motion")
    parser.add_argument("-a", type=str, default="/dev/tty.usbmodem11401",
                        help="COM port for A")
    parser.add_argument("-b", type=str, default="/dev/tty.usbmodem11301",
                        help="COM port for B")
    parser.add_argument("-z", type=str, default="/dev/tty.usbmodem11201",
                        help="COM port for Z")
    # parser.add_argument("--rgb", type=str, default="COM14",
    #                     help="COM port for Z")
    parser.add_argument("-s", "--spindle", default="/dev/tty.usbmodem111401",
                        help="COM port for the spindle")
    parser.add_argument("-u", "--unit_scale", type=float, default=1.0,
                        help="Scaling factor")
    return parser.parse_known_args()


def parse_xy(filename, action_queue,
             feedrate,
             servo_up_action=None,
             servo_down_action=None):

   # action_queue.put(homing_action)

    with open(filename, "r") as f:
        for line in f.readlines():
            if line.upper().startswith("UP"):
                if servo_up_action is not None:
                    action_queue.put(servo_up_action)
            elif line.upper().startswith("DOWN"):
                if servo_down_action is not None:
                    action_queue.put(servo_down_action)
            else:
                action_queue.put(Line([float(x) for x in line.strip().split(",")], feedrate))


def parse_gcode(filename, action_queue, default_feedrate):
    feedrate = default_feedrate

    with open(filename, "r") as f:
        for line in f.readlines():
            if line.startswith("G1") or line.startswith("G0"):

                params_parsed = {
                    "X": np.nan,
                    "Y": np.nan,
                    "Z": np.nan,
                    "F": np.nan
                }

                fx = line.find('X')
                fy = line.find('Y')
                fz = line.find('Z')
                ff = line.find('F')

                if fx > 0:
                    end = fx + 1
                    while line[end].isdigit() or line[end] == '-' or line[end] == '.':
                        end = end + 1

                    params_parsed["X"] = float(line[fx + 1:end])

                if fy > 0:
                    end = fy + 1
                    while line[end].isdigit() or line[end] == '-' or line[end] == '.':
                        end = end + 1

                    params_parsed["Y"] = float(line[fy + 1:end])

                if fz > 0:
                    end = fz + 1
                    while line[end].isdigit() or line[end] == '-' or line[end] == '.':
                        end = end + 1

                    params_parsed["Z"] = float(line[fz + 1:end])

                if ff > 0:
                    end = ff + 1
                    while line[end].isdigit() or line[end] == '-' or line[end] == '.':
                        end = end + 1

                    params_parsed["F"] = float(line[ff + 1:end])

                if not math.isnan(params_parsed["F"]):
                    feedrate = params_parsed["F"]

                action_queue.put(Line([params_parsed["X"],
                                       params_parsed["Y"],
                                       params_parsed["Z"]], feedrate))

            if line.startswith("G4"):
                fp = line.find('P')
                params_parsed = {
                    "P": math.nan,
                }
                if fp > 0:
                    end = fp + 1
                    while line[end].isdigit() or line[end] == '-' or line[end] == '.':
                        end = end + 1
                    params_parsed["P"] = float(line[fp + 1:end])

                if not math.isnan(params_parsed["P"]):
                    wait = params_parsed["P"] / 1000.0
                    action_queue.put(WaitAction(wait))

            elif line.startswith("M150"):
                m = re.match(r"M150\s+R(\d+)\sU(\d+)\sB(\d+)", line)

                if m is None:
                    continue

                action_queue.put(ColorAction("esp32",
                                             int(m.group(1)),
                                             int(m.group(2)),
                                             int(m.group(3))))

            elif line.startswith("M03") or line.startswith("M04") or line.startswith("M3") or line.startswith("M4"):
                action_queue.put(SpindleAction("spindle", None, True))
            elif line.startswith("M05") or line.startswith("M5"):
                action_queue.put(SpindleAction("spindle", None, False))

        #     action_queue.put(homing_action)


def init_machine(ser_a, ser_b, ser_z, ser_spindle):
    multiprocessing.set_start_method('spawn')
    action_queue = multiprocessing.Queue()

    modules_config = {
        "a": {
            "type": "stepper",
            "port": ser_a,
            "baudrate": 115200,
            "axis": 0,
            # "steps_per_unit": 102,  # 4.85mm radius
            "steps_per_unit": 3200 / 32,  # 4.85mm radius
            "reverse": False
        },
        "b": {
            "type": "stepper",
            "port": ser_b,
            "baudrate": 115200,
            "axis": 1,
            # "steps_per_unit": 102,  # 4.85mm radius
            "steps_per_unit": 3200 / 32,  # 4.85mm radius
            "reverse": False
        },
        "z": {
            "type": "stepper",
            "port": ser_z,
            "baudrate": 115200,
            "axis": 2,
            "steps_per_unit": 385,  # (steps/rev) / (mm/rev)
            "reverse": True
        },
        # "spindle": {
        #     "type": "spindle",
        #     "port": ser_spindle,
        #     "baudrate": 115200
        # }
    }

    p1 = multiprocessing.Process(target=modules_manager, args=(action_queue, modules_config, transform_corexy))
    p1.start()

    return action_queue


def main():
    args, _ = parse_arguments()

    action_queue = init_machine(args.a, args.b, args.z, args.spindle)


    action_queue.put(SpindleAction('spindle', True))
    action_queue.put(WaitAction(5))
    action_queue.put(Line([15, 0, 0], 8))
    action_queue.put(Line([15, 15, 5], 8))
    action_queue.put(Line([0, 0, 0], 8))
    action_queue.put(SpindleAction('spindle', False))


    time.sleep(10)

    return

    feedrate = args.feedrate

    filename = args.filename
    ext = os.path.splitext(filename)[-1]
    if ext == ".xy":
        parse_xy(filename, action_queue, feedrate)
    elif ext in (".nc", ".gcode"):
        parse_gcode(filename, action_queue, args.feedrate)
    else:
        print(f"Unrecognized file type: '{ext}'")


if __name__ == "__main__":
    main()
