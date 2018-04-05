from PyQt5 import QtGui
from mainwindow import Ui_MainWindow #same name as appears in mainWindowUi.py

class MainWindow (QtGui.QMainWindow): #Or wherever you are inheriting from
    def __init__ (self, parent = None):
        super (MainWindow, self).__init__ ()
        self.ui = Ui_MainWindow () #same name as appears in mainWindowUi.py
        self.ui.setupUi (self)

    #implement slots and signals and other funny things