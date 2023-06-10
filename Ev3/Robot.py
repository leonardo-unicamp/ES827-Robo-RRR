#!/usr/bin/env python3


from ev3dev2.motor import (OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D, LargeMotor,MediumMotor,
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
        self.motor.position = pos* self.reduction


class Robot:
    def __init__(self):
        base = ReducedMotor(LargeMotor(OUTPUT_A), 24 * 2.5)
        # base.run_until_stalled(5)
        base.set_position(0)
        self.base = base

        shoulder = ReducedMotor(LargeMotor(OUTPUT_B), (40/24) * (40/8))
        # shoulder.run_until_stalled(-20)
        shoulder.set_position(0)
        self.shoulder = shoulder

        elbow = ReducedMotor(MediumMotor(OUTPUT_C), 40)
        # elbow.run_until_stalled(5)
        elbow.set_position(0)
        self.elbow = elbow

        claw = ReducedMotor(MediumMotor(OUTPUT_D), 24)
        # claw.run_until_stalled(5)
        claw.set_position(0)
        self.claw = claw
    

    def move(self, q1, q2, q3, q4):
        self.base.on_to_position(SpeedPercent(100), q1, block=False)
        self.shoulder.on_to_position(SpeedPercent(10), q2, block=False)
        self.elbow.on_to_position(SpeedPercent(40), q3, block=False)
        self.claw.on_to_position(SpeedPercent(20), q4, block=False)
        pass
        # print(q1, q2, q3, q4)


def main():
    robot = Robot()
    robot.move(1, 2, 3, 4)
    pass


if __name__ == "__main__":
    main()
