#!/usr/bin/env python3


from ev3dev2.motor import (OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D, LargeMotor,
                           Motor, SpeedDPS, SpeedPercent)


class ReducedMotor:
    def __init__(self, motor, reduction):
        self.motor = motor
        self.reduction = reduction

    def on_to_position(self, speed, position, brake=True, block=True):
        self.motor.on_to_position(
            speed, position * self.reduction, brake=brake, block=block
        )

    def run_until_stalled(self, duty, stop_action="brake"):
        self.motor.run_direct(duty_cycle_sp=duty)
        self.motor.wait_until("stalled")
        self.motor.stop(stop_action=stop_action)

    def set_position(self, pos):
        self.motor.position = pos


class Robot:
    def __init__(self):
        shoulder = ReducedMotor(LargeMotor(OUTPUT_A), 2)
        shoulder.run_until_stalled(5)
        shoulder.set_position(0)
        self.shoulder = shoulder

        # "shoulder":
        # "elbow":
        # "wrist":
        # "claw":
        pass

    def move(self, q1, q2, q3, q4):
        self.shoulder.on_to_position(SpeedPercent(10), q1, block=False)
        pass
        # print(q1, q2, q3, q4)


def main():
    robot = Robot()
    robot.move(1, 2, 3, 4)
    pass


if __name__ == "__main__":
    main()
