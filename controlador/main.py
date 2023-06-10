from PyQt5.QtWidgets import *
from PyQt5.uic import loadUi
from PyQt5.QtGui import *
from PyQt5.QtCore import *

import numpy as np
from robot import Robot

class GuiRobo(QMainWindow):
    
    def __init__(self):
        QMainWindow.__init__(self)
        loadUi("./controlador/gui.ui", self)
        self.setWindowTitle("Controlador do Robô")

        self.robot = Robot(2, 2, 2)

        self.coord_x = 2
        self.coord_y = 0
        self.coord_z = 0
        self.robot_inverse(self.coord_x, self.coord_y, self.coord_z)

        self.ang_t1 = 0
        self.ang_t2 = 0
        self.ang_t3 = 0

        self.trajectory = {
            "x":  [], "y":  [], "z":  [],
            "j1": [], "j2": [], "j3": [], 
            "vi": [], "vf": [], "claw": [], 
            "t":  []
        }

        self.btn_inverse_move.clicked.connect(self.move_robot_inverse)
        self.btn_trajectory_add.clicked.connect(self.trajectory_add)
        self.btn_trajectory_run.clicked.connect(self.trajectory_run)
        self.btn_joy_up.clicked.connect(lambda: self.robot_inverse(float(self.coord_x), float(self.coord_y + 0.1), float(self.coord_z)))
        self.btn_joy_down.clicked.connect(lambda: self.robot_inverse(float(self.coord_x), float(self.coord_y - 0.1), float(self.coord_z)))
        self.btn_joy_left.clicked.connect(lambda: self.robot_inverse(float(self.coord_x + 0.1), float(self.coord_y), float(self.coord_z)))
        self.btn_joy_right.clicked.connect(lambda: self.robot_inverse(float(self.coord_x - 0.1), float(self.coord_y), float(self.coord_z)))
        self.sld_joy_z.valueChanged.connect(lambda: self.robot_inverse(float(self.coord_x), float(self.coord_y), float(self.sld_joy_z.value())))

    def trajectory_add(self):
        t = float(self.le_ti.text()) 
        if t not in self.trajectory["t"] and (self.trajectory["t"] == [] or t > self.trajectory["t"][-1]):
            self.trajectory["x"].append(self.coord_x)
            self.trajectory["y"].append(self.coord_y)
            self.trajectory["z"].append(self.coord_z)
            self.trajectory["j1"].append(self.ang_t1)
            self.trajectory["j2"].append(self.ang_t2)
            self.trajectory["j3"].append(self.ang_t3)
            self.trajectory["vi"].append(float(self.le_vi.text()))
            self.trajectory["vf"].append(float(self.le_vf.text()))
            self.trajectory["t"].append(t)
            self.trajectory["claw"].append(float(self.le_claw.text()))
            self.update_trajectory_browser()
        else:
            QMessageBox.warning(self, "Trajetória", "O instante de tempo é invalido.")

    def trajectory_run(self):
        _, joint1, joint2, joint3 = self.robot.create_trajectory(self.trajectory, 100)
        x, y, z = [], [], []
        for i in range(len(joint1)):
            xj, yj, zj = self.robot.forward_kinematics(
                joint1[i], joint2[i], joint3[i])
            x.append(xj)
            y.append(yj)
            z.append(zj)
        self.mpl_widget.animation(x, y, z, 100)

    def update_trajectory_browser(self):
        self.browser_trajectory.clear()
        for i in range(len(self.trajectory["j1"])):
            t = self.trajectory["t"][i]
            x = self.trajectory["x"][i]
            y = self.trajectory["y"][i]
            z = self.trajectory["z"][i]
            self.browser_trajectory.append("" +
                "Tempo %.2fs:\n" % (t) +
                "x: %.2f, y: %.2f, z: %.2f\n" % (x, y, z))

    def move_robot_inverse(self):
        x = float(self.le_set_x.text())
        y = float(self.le_set_y.text())
        z = float(self.le_set_z.text())
        self.robot_inverse(x, y, z)

    def robot_inverse(self, x, y, z):
        try:
            self.ang_t1, self.ang_t2, self.ang_t3 = self.robot.inverse_kinematics(x, y, z)
            self.move_robot_forward()
        except:
            QMessageBox.warning(self, "Ponto Alvo", "Preencha todos os campos!")

    def move_robot_forward(self):        
        try:
            xj, yj, zj = self.robot.forward_kinematics(
                self.ang_t1, self.ang_t2, self.ang_t3)
            self.mpl_widget.plot(xj, yj, zj)
            self.coord_x, self.coord_y, self.coord_z = xj[-1], yj[-1], zj[-1]
            self.update_coordinates_label()
        except:
            QMessageBox.warning(self, "Ponto Alvo", "O ponto inserido está fora da área de trabalho do robô.")

    def update_coordinates_label(self):
        self.lb_x.setText("%.2f" % self.coord_x)
        self.lb_y.setText("%.2f" % self.coord_y)
        self.lb_z.setText("%.2f" % self.coord_z)


app = QApplication([])
window = GuiRobo()
window.show()
app.exec_()