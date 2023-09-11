# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'test.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(800, 600)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.widget = QtWidgets.QWidget(self.centralwidget)
        self.widget.setGeometry(QtCore.QRect(20, 30, 181, 411))
        self.widget.setObjectName("widget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.widget)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.label = QtWidgets.QLabel(self.widget)
        self.label.setObjectName("label")
        self.verticalLayout.addWidget(self.label)
        self.doubleSpinBox = QtWidgets.QDoubleSpinBox(self.widget)
        self.doubleSpinBox.setObjectName("doubleSpinBox")
        self.verticalLayout.addWidget(self.doubleSpinBox)
        self.label_2 = QtWidgets.QLabel(self.widget)
        self.label_2.setObjectName("label_2")
        self.verticalLayout.addWidget(self.label_2)
        self.doubleSpinBox_2 = QtWidgets.QDoubleSpinBox(self.widget)
        self.doubleSpinBox_2.setObjectName("doubleSpinBox_2")
        self.verticalLayout.addWidget(self.doubleSpinBox_2)
        self.label_3 = QtWidgets.QLabel(self.widget)
        self.label_3.setObjectName("label_3")
        self.verticalLayout.addWidget(self.label_3)
        self.doubleSpinBox_3 = QtWidgets.QDoubleSpinBox(self.widget)
        self.doubleSpinBox_3.setObjectName("doubleSpinBox_3")
        self.verticalLayout.addWidget(self.doubleSpinBox_3)
        self.label_4 = QtWidgets.QLabel(self.widget)
        self.label_4.setObjectName("label_4")
        self.verticalLayout.addWidget(self.label_4)
        self.doubleSpinBox_4 = QtWidgets.QDoubleSpinBox(self.widget)
        self.doubleSpinBox_4.setObjectName("doubleSpinBox_4")
        self.verticalLayout.addWidget(self.doubleSpinBox_4)
        self.label_5 = QtWidgets.QLabel(self.widget)
        self.label_5.setObjectName("label_5")
        self.verticalLayout.addWidget(self.label_5)
        self.doubleSpinBox_5 = QtWidgets.QDoubleSpinBox(self.widget)
        self.doubleSpinBox_5.setObjectName("doubleSpinBox_5")
        self.verticalLayout.addWidget(self.doubleSpinBox_5)
        self.label_6 = QtWidgets.QLabel(self.widget)
        self.label_6.setObjectName("label_6")
        self.verticalLayout.addWidget(self.label_6)
        self.doubleSpinBox_6 = QtWidgets.QDoubleSpinBox(self.widget)
        self.doubleSpinBox_6.setObjectName("doubleSpinBox_6")
        self.verticalLayout.addWidget(self.doubleSpinBox_6)
        self.pushButton = QtWidgets.QPushButton(self.widget)
        self.pushButton.setObjectName("pushButton")
        self.verticalLayout.addWidget(self.pushButton)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 800, 22))
        self.menubar.setObjectName("menubar")
        self.menuHome = QtWidgets.QMenu(self.menubar)
        self.menuHome.setObjectName("menuHome")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)
        self.menubar.addAction(self.menuHome.menuAction())

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.label.setText(_translate("MainWindow", "Num Positions"))
        self.label_2.setText(_translate("MainWindow", "Positions"))
        self.label_3.setText(_translate("MainWindow", "Position Width Radians"))
        self.label_4.setText(_translate("MainWindow", "Detent Strength Unit"))
        self.label_5.setText(_translate("MainWindow", "Endstop Strength Unit"))
        self.label_6.setText(_translate("MainWindow", "Text"))
        self.pushButton.setText(_translate("MainWindow", "Publish"))
        self.menuHome.setTitle(_translate("MainWindow", "Home"))


if __name__ == '__main__':
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())