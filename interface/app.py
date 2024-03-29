from PyQt5.QtWidgets import *
from PyQt5.uic import loadUi
from PyQt5.QtGui import *
from PyQt5.QtCore import *

# Robot libraries
from robot_control.control import RobotControl

# Other libraries
import threading
from math import degrees, radians, pi
from time import sleep


class GuiRobo(QMainWindow):

    
    def __init__(self):
        QMainWindow.__init__(self)
        loadUi("./gui/gui.ui", self)
        self.setWindowTitle("Controlador do Robô")
        
        # Initialize the robot
        self.robot = RobotControl()

        # Initialize simulation plot
        self.mpl_widget.real_time(self.robot.get_all_joints_position)

        # Initialize label updater
        self.label_thread_start()

        # Joystick buttons
        self.btn_up.pressed.connect(lambda: self.joystick("up"))
        self.btn_down.pressed.connect(lambda: self.joystick("down"))
        self.btn_left.pressed.connect(lambda: self.joystick("left"))
        self.btn_right.pressed.connect(lambda: self.joystick("right"))
        self.btn_up_z.pressed.connect(lambda: self.joystick("up_z"))
        self.btn_down_z.pressed.connect(lambda: self.joystick("down_z"))
        self.btn_ini.pressed.connect(lambda: self.joystick("initial"))

        # Joints buttons
        self.btn_up_j1.pressed.connect(lambda: self.joints("up_j1"))
        self.btn_down_j1.pressed.connect(lambda: self.joints("down_j1"))
        self.btn_up_j2.pressed.connect(lambda: self.joints("up_j2"))
        self.btn_down_j2.pressed.connect(lambda: self.joints("down_j2"))
        self.btn_up_j3.pressed.connect(lambda: self.joints("up_j3"))
        self.btn_down_j3.pressed.connect(lambda: self.joints("down_j3"))

        # Trajectory
        self.btn_trajectory_add.pressed.connect(self.add_to_trajectory)
        self.btn_trajectory_run.pressed.connect(self.run_trajectory)
        self.btn_trajectory_remove.pressed.connect(self.remove_trajectory)
        self.btn_trajectory_save.pressed.connect(self.save_trajectory)
        self.btn_trajectory_load.pressed.connect(self.load_trajectory)

        # Claw buttons
        self.btn_set_claw.pressed.connect(self.set_claw)

        # Inverse kinematics button
        #self.btn_inverse.pressed.connect(self.inverse_movement)

    def set_claw(self):
        j1, j2, j3, _ = self.robot.get_joint_angles()
        j4 = self.dsb_claw.value()
        self.robot.set_joint_angles(j1, j2, j3, j4)


    def update_simulation(self):
        th = threading.Thread(target=lambda: self.mpl_widget.plot(*self.robot.get_all_joints_position()))
        th.start()


    def save_trajectory(self):
        name = self.le_trajectory_name.text()
        if name != "":
            if self.robot.save_trajectory(name):
                self.browser.append("Trajectory %s saved!" % name)


    def load_trajectory(self):
        name = self.le_trajectory_name.text()
        if name != "":
            if self.robot.load_trajectory(name):
                self.browser.append("Trajectory %s loaded!" % name)


    def remove_trajectory(self):
        self.robot.remove_trajectory()
        self.browser.append("Trajectory reseted!")
        self.dsb_time.setValue(0)


    def add_to_trajectory(self):

        # Get values required by trajectory calculator
        speed_i = self.dsb_si.value()
        speed_f = self.dsb_sf.value()
        time = self.dsb_time.value()

        # Add the current point to trajectory
        self.robot.add_to_trajectory(speed_i, speed_f, time)
        
        # Increment the time field
        self.dsb_time.setValue(time + 2)

        # Log info into browser
        x, y, z = self.robot.get_manipulator_position()
        self.browser.append("Point (%.2f, %.2f, %.2f) added." % (x, y, z))


    def run_trajectory(self):
        self.robot.run_trajectory()


    def joystick(self, button: str):

        # Step in milimetters
        step = 5

        # Get the current position
        x, y, z = self.robot.get_manipulator_position()

        if button == "up":
            self.robot.set_manipulator_position(x, y + step, z)
        elif button == "down":
            self.robot.set_manipulator_position(x, y - step, z)
        elif button == "right":
            self.robot.set_manipulator_position(x + step, y, z)
        elif button == "left":
            self.robot.set_manipulator_position(x - step, y, z)
        elif button == "up_z":
            self.robot.set_manipulator_position(x, y, z + step)
        elif button == "down_z":
            self.robot.set_manipulator_position(x, y, z - step)
        elif button == "initial":
            self.robot.go_to(0, 0, 0, 0, 5)


    def joints(self, button: str):

        step = radians(2)   # Step in radians

        # Get the current joints
        j1, j2, j3, j4 = self.robot.get_joint_angles()

        if button == "up_j1" and -2*pi < j1 < 2*pi:
            self.robot.set_joint_angles(j1 + step, j2, j3, j4)
        elif button == "down_j1" and -2*pi < j1 < 2*pi:
            self.robot.set_joint_angles(j1 - step, j2, j3, j4)
        if button == "up_j2" and -2*pi < j2 < 2*pi:
            self.robot.set_joint_angles(j1, j2 + step, j3, j4)
        elif button == "down_j2" and -2*pi < j2 < 2*pi:
            self.robot.set_joint_angles(j1, j2 - step, j3, j4)
        if button == "up_j3" and -2*pi < j3 < 2*pi:
            self.robot.set_joint_angles(j1, j2, j3 + step, j4)
        elif button == "down_j3" and -2*pi < j3 < 2*pi:
            self.robot.set_joint_angles(j1, j2, j3 - step, j4)


    def label_thread_start(self):
        self.label_updater_thread = QThread()
        self.label_worker = LabelUpdater(self)
        self.label_worker.moveToThread(self.label_updater_thread)
        self.label_updater_thread.started.connect(self.label_worker.run)
        self.label_updater_thread.start()


class LabelUpdater(QObject):

    def __init__(self, gui: GuiRobo):
        super().__init__()
        self.gui = gui

    def run(self):
        while True:

            # Update (x,y,z) manipulator position
            x, y, z = self.gui.robot.get_manipulator_position()
            self.gui.lb_x.setText("%.2f" % x)
            self.gui.lb_y.setText("%.2f" % y)
            self.gui.lb_z.setText("%.2f" % z)

            # Update joints angles
            j1, j2, j3, j4 = self.gui.robot.get_joint_angles()
            self.gui.lb_j1.setText("%.1f°" % degrees(j1))
            self.gui.lb_j2.setText("%.1f°" % degrees(j2))
            self.gui.lb_j3.setText("%.1f°" % degrees(j3))
            self.gui.lb_claw.setText("%.1f°" % j4)

            sleep(0.5)


app = QApplication([])
window = GuiRobo()
window.show()
app.exec_()