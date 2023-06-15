#!/usr/bin/env python3


from ev3dev2.motor import (OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D, LargeMotor,MediumMotor,
                           Motor, SpeedDPS, SpeedPercent)

delta_angle = .5

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

    def get_position(self):
        return self.motor.position / self.reduction


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
        q2 = -q2
        q3 = -q3
        if abs(self.base.get_position() - q1)>delta_angle:
            self.base.on_to_position(SpeedPercent(100), q1, block=False)
        if abs(self.shoulder.get_position() - q2)>delta_angle:
            self.shoulder.on_to_position(SpeedPercent(10), q2, block=False)
        if abs(self.elbow.get_position() - q3)>delta_angle:
            self.elbow.on_to_position(SpeedPercent(40), q3, block=False)
        if abs(self.claw.get_position() - q4)>delta_angle:
            self.claw.on_to_position(SpeedPercent(20), q4, block=False)


def main():
    robot = Robot()
    robot.move(1, 2, 3, 4)
    pass


if __name__ == "__main__":
    main()
