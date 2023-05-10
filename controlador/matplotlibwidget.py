from PyQt5.QtWidgets import *

import matplotlib
from matplotlib.figure import Figure
import matplotlib.dates as mdates

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FC
from matplotlib.backends.backend_qt5agg import (NavigationToolbar2QT as NavigationToolbar)

import numpy as np
from datetime import datetime, timedelta

matplotlib.use('Qt5Agg')

class matplotlibwidget(QWidget):

    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', 
              '#17becf', '#E52B50', '#FFBF00', '#9966CC', '#0000FF', '#7FFF00', '#FF7F50', '#FFD700', '#BEBEBE', 
              '#3FFF00', '#4B0082', '#FF00FF', '#FF4500', '#0F52BA', '#FF2400', '#00FF7F', '#D2B48C', '#92000A', 
              '#808000', '#C8A2C8', '#29AB87', '#0047AB', '#8A2BE2', '#6F4E37', '#841B2D', '#8DB600', '#000000',
              '#1B4D3E', '#CC5500', '#91A3B0', '#B2FFFF']
    
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
        self.canvas.axes = self.canvas.figure.add_subplot(111)
        self.canvas.axes.ticklabel_format(axis="y", style="sci")

        vertical_layout = QVBoxLayout()
        vertical_layout.addWidget(NavigationToolbar(self.canvas, self))
        vertical_layout.addWidget(self.canvas)
        self.setLayout(vertical_layout)

        # Data plotted
        self.lines = []
        self.x_axis = None
        self.y_axis = None
        self.label = None
        self.graphs = {}

    #
    def plot(self, x: list, y: list, label: list) -> None:
        self.x_axis = x
        self.y_axis = y
        self.label = label

        self.canvas.axes.cla()
        for i in range(len(x)):
            line = self.canvas.axes.plot(x[i], y[i], label=label[i], color=self.colors[i])
            self.lines.append(line[0])

        try:
            if type(x[0][0]) == datetime:
                self.setDateToX(x)
            
            if x[0][0] < x[0][-1]:
                self.canvas.axes.set_xlim([x[0][0], x[0][-1]])
            else:
                self.canvas.axes.set_xlim([x[0][-1], x[0][0]])
        except: pass

        self.canvas.axes.grid(True, axis="x", linestyle="--")
        self.updateDraw()

    #
    def update(self, x, y, labels):
        self.x_axis = x
        self.y_axis = y
        self.label = labels
        self.clearAxisY()
        for i in range(len(y)):
            line = self.canvas.axes.plot(x[i], y[i], label=labels[i], color=self.colors[i])
            self.lines.append(line[0])
        self.updateDraw()

    #
    def updateDraw(self):
        legend = self.canvas.axes.legend(self.lines, self.label, bbox_to_anchor=(0, 1.02, 1, 0.2), loc="lower left",
                                mode="expand", borderaxespad=0, ncol=3)
        self.canvas.figure.canvas.mpl_connect("pick_event", self.on_pick)

        lines_legend = legend.get_lines()
        for i in range(len(lines_legend)):
            lines_legend[i].set_picker(True)
            lines_legend[i].set_pickradius(5)
            self.graphs[lines_legend[i]] = self.lines[i]

        self.canvas.axes.relim()            # recompute the ax.dataLim
        self.canvas.axes.autoscale_view()   # update ax.viewLim using the new dataLim
        self.canvas.draw()

    #
    def on_pick(self, event):
        legend = event.artist
        isVisible = legend.get_visible()
        self.graphs[legend].set_visible(not isVisible)
        legend.set_visible(not isVisible)
        self.updateDraw()

    #
    def clearAxisY(self):
        for i in range(len(self.lines)):
            self.lines[i].remove()
        self.lines = []

    #
    def setDateToX(self, x):
        #step = (x[0][-1] - x[0][0])/8
        self.canvas.axes.xaxis.set_major_formatter(mdates.DateFormatter('%b-%d, %Y'))
        #deltat = timedelta(days=step.days, seconds=step.seconds, microseconds=step.microseconds)
        #self.canvas.axes.xaxis.set_ticks(np.arange(x[0][0], x[0][-1] + deltat, deltat))

