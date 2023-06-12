from PyQt5.QtWidgets import *
from PyQt5.QtCore import *

import time
import matplotlib
from matplotlib.figure import Figure
import matplotlib.dates as mdates

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FC
from matplotlib.backends.backend_qt5agg import (NavigationToolbar2QT as NavigationToolbar)

import numpy as np
from datetime import datetime, timedelta

matplotlib.use('Qt5Agg')

class matplotlibwidget(QWidget):
    
    def __init__(self, parent=None):
        QWidget.__init__(self, parent)
        
        self.plots = {}

        # Figure Properties
        figure = Figure(dpi=90)
        figure.set_constrained_layout(True)

        FC.setSizePolicy(self, QSizePolicy.Expanding, QSizePolicy.Expanding)
        FC.updateGeometry(self)
        
        # Basic plot configurations
        self.canvas = FC(figure)
        self.canvas.axes = self.canvas.figure.add_subplot(111, projection="3d")

        vertical_layout = QVBoxLayout()
        vertical_layout.addWidget(NavigationToolbar(self.canvas, self))
        vertical_layout.addWidget(self.canvas)
        self.setLayout(vertical_layout)

        # Data plotted
        self.lines = []
        self.graphs = {}

    #
    def plot(self, x: list, y: list, z: list) -> None:
        self.x_axis = x
        self.y_axis = y

        self.canvas.axes.cla()
        self.canvas.axes.plot(x, y, z, c="C0")
        self.canvas.axes.scatter(x, y, z, color="C1", alpha=1)
        self.canvas.axes.set_xlim(-300, 300)
        self.canvas.axes.set_ylim(-300, 300)
        self.canvas.axes.set_zlim(0, 300)
        self.canvas.axes.tick_params(left = False, right = False, 
            labelleft = False, labelbottom = False, bottom = False)

        self.updateDraw()

    #
    def update(self, x, y, labels):
        self.x_axis = x
        self.y_axis = y
        self.label = labels
        self.clearAxisY()
        line = self.canvas.axes.plot(x, y, color="C0")
        self.lines.append(line)
        self.updateDraw()

    #
    def updateDraw(self):
        self.canvas.axes.relim()            # recompute the ax.dataLim
        self.canvas.axes.autoscale_view()   # update ax.viewLim using the new dataLim
        self.canvas.draw()

    #
    def clearAxisY(self):
        for i in range(len(self.lines)):
            self.lines[i].remove()
        self.lines = []

