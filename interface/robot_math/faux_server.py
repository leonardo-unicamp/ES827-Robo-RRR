import random
from time import time

import matplotlib.pyplot as plt
import numpy as np

from Com import Com
from DH import DH

HOST = "localhost"
PORT = 12345


class ParseError(Exception):
    pass


def parse(msg):
    try:
        msg = msg.decode("ASCII").split("#")
        decoded = [float(x) for x in msg[1].split(";")]
        if len(decoded) != 4:
            raise ValueError
    except Exception as error:
        raise ParseError(repr(error))

    return decoded


def split_3d_point_array(array):
    return np.array([array[:, 0], array[:, 1], array[:, 2]])


def main() -> None:
    # robot = Robot()

    r = DH()

    com = Com(HOST, PORT)
    rec = com.receive(1024)
    print("*" * 20, "Ready", "*" * 20, sep="\n")
    last_time = time()

    ax = plt.figure().add_subplot(projection="3d")
    (line,) = ax.plot(
        *np.hsplit(r.fw_kinematics([0, 0, 0]), 3)
    )  # Update the line object with new data

    while True:
        msg = next(rec)
        try:
            joints = parse(msg)
            print(joints)
            # robot.move(*joints)
            # t = time()
            # print(t - last_time)
            # last_time = t
        except ParseError as error:
            print(repr(error))  # Might be slow
            pass  # Ignore errors in communincation
        except KeyboardInterrupt:
            break
        (x, y, z) = split_3d_point_array(r.fw_kinematics(joints[:3]))
        line.set_data(x, y)
        line.set_3d_properties(z)

        ax.relim()  # Recalculate the data limits
        ax.autoscale_view()  # Autoscale the axes
        plt.draw()  # Redraw the plot
        plt.pause(0.1)  # Pause to allow the plot to be displayed


if __name__ == "__main__":
    main()
