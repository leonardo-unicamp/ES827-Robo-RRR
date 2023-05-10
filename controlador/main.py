from PyQt5.QtWidgets import *
from PyQt5.uic import loadUi
from PyQt5.QtGui import *
from PyQt5.QtCore import *

import numpy as np
import pandas as pd
import sympy as sym
from robot import Robot

class GuiRobo(QMainWindow):
    
    def __init__(self):
        QMainWindow.__init__(self)
        loadUi("./controlador/gui.ui", self)
        self.setWindowTitle("Controlador do Robô")

        self.robot = Robot(2, 2, 2)

        self.coord_x = 2
        self.coord_y = 2
        self.coord_z = 2

        self.ang_theta1 = 0
        self.ang_theta2 = 0
        self.ang_theta3 = 0

        self.slider_theta1.valueChanged.connect(lambda: self.theta_changed(1))
        self.slider_theta2.valueChanged.connect(lambda: self.theta_changed(2))
        self.slider_theta3.valueChanged.connect(lambda: self.theta_changed(3))

        self.move_robot()

    def move_robot(self):
        #t1, t2, t3 = self.robot.inverse_kinematics(4, 0, 2)
        xj, yj, zj = self.robot.forward_kinematics(self.ang_theta1, 
                                                   self.ang_theta2, 
                                                   self.ang_theta3)
        self.mpl_widget.plot(xj, yj, zj)

    def theta_changed(self, theta_type):
        if theta_type == 1:
            self.label_theta1.setText(str(self.slider_theta1.value()))
            self.ang_theta1 = self.slider_theta1.value()*np.pi/180
        if theta_type == 2:
            self.label_theta2.setText(str(self.slider_theta2.value()))
            self.ang_theta2 = self.slider_theta2.value()*np.pi/180
        if theta_type == 3:
            self.label_theta3.setText(str(self.slider_theta3.value()))
            self.ang_theta3 = self.slider_theta3.value()*np.pi/180
        self.move_robot()


app = QApplication([])
window = GuiRobo()
window.show()
app.exec_()