# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'mainwindow.ui'
#
# Created by: PyQt5 UI code generator 5.10.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(402, 224)
        self.centralWidget = QtWidgets.QWidget(MainWindow)
        self.centralWidget.setObjectName("centralWidget")
        self.gridLayoutWidget = QtWidgets.QWidget(self.centralWidget)
        self.gridLayoutWidget.setGeometry(QtCore.QRect(110, 20, 171, 71))
        self.gridLayoutWidget.setObjectName("gridLayoutWidget")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.gridLayoutWidget)
        self.gridLayout_2.setContentsMargins(11, 11, 11, 11)
        self.gridLayout_2.setSpacing(6)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.port_combobox = QtWidgets.QComboBox(self.gridLayoutWidget)
        self.port_combobox.setObjectName("port_combobox")
        self.gridLayout_2.addWidget(self.port_combobox, 1, 0, 1, 1)
        self.text_label = QtWidgets.QLabel(self.gridLayoutWidget)
        self.text_label.setObjectName("text_label")
        self.gridLayout_2.addWidget(self.text_label, 0, 0, 1, 1)
        self.gridLayoutWidget_2 = QtWidgets.QWidget(self.centralWidget)
        self.gridLayoutWidget_2.setGeometry(QtCore.QRect(110, 100, 168, 27))
        self.gridLayoutWidget_2.setObjectName("gridLayoutWidget_2")
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout(self.gridLayoutWidget_2)
        self.horizontalLayout_4.setContentsMargins(11, 11, 11, 11)
        self.horizontalLayout_4.setSpacing(6)
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.connect_button_2 = QtWidgets.QPushButton(self.gridLayoutWidget_2)
        self.connect_button_2.setObjectName("connect_button_2")
        self.horizontalLayout_4.addWidget(self.connect_button_2)
        self.refresh_button_2 = QtWidgets.QPushButton(self.gridLayoutWidget_2)
        self.refresh_button_2.setObjectName("refresh_button_2")
        self.horizontalLayout_4.addWidget(self.refresh_button_2)
        MainWindow.setCentralWidget(self.centralWidget)
        self.menuBar = QtWidgets.QMenuBar(MainWindow)
        self.menuBar.setGeometry(QtCore.QRect(0, 0, 402, 22))
        self.menuBar.setObjectName("menuBar")
        MainWindow.setMenuBar(self.menuBar)
        self.mainToolBar = QtWidgets.QToolBar(MainWindow)
        self.mainToolBar.setObjectName("mainToolBar")
        MainWindow.addToolBar(QtCore.Qt.TopToolBarArea, self.mainToolBar)
        self.statusBar = QtWidgets.QStatusBar(MainWindow)
        self.statusBar.setObjectName("statusBar")
        MainWindow.setStatusBar(self.statusBar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "EEG Scope Window"))
        self.text_label.setText(_translate("MainWindow", "TextLabel"))
        self.connect_button_2.setText(_translate("MainWindow", "PushButton"))
        self.refresh_button_2.setText(_translate("MainWindow", "PushButton"))
        self.mainToolBar.setWindowTitle(_translate("MainWindow", "EEG Scope"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())

