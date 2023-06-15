#!/usr/bin/env python3

from time import time

from Com import Com
from Robot import Robot

from env import HOST

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


def main() -> None:
    robot = Robot()
    

    com = Com(HOST, PORT)
    rec = com.receive(1024)
    print("*" * 20, "Ready", "*" * 20,sep = "\n")
    last_time = time()
    while True:
        msg = next(rec)
        try:
            robot.move(*parse(msg))
            t = time()
            print(t - last_time)
            last_time = t
        except ParseError as error:
            print(repr(error))  # Might be slow
            pass  # Ignore errors in communincation
        except KeyboardInterrupt:
            break


if __name__ == "__main__":
    main()
