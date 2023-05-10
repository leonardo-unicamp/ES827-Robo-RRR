from PyQt5.QtWidgets import *
from PyQt5.uic import loadUi
from PyQt5.QtGui import *
from PyQt5.QtCore import *

class DataAnalyzer(QMainWindow):
    
    def __init__(self):
        QMainWindow.__init__(self)
        loadUi("./controlador/gui.ui", self)
        self.setWindowTitle("Controlador do Robô")

app = QApplication([])
window = DataAnalyzer()
window.show()
app.exec_()