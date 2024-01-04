#!/usr/bin/env python3
# Author: Xiangyu Fu
# Date: 2023-09-01
# Description: This script is used to control the robot arm through the robot movement interface.

import sys
import rospy
import math
from robot_movement_interface.msg import Command, CommandList
from robot_movement_interface.msg import Result
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Int32, Float32
from knob_robot_control.msg import KnobState, KnobCommand
from device_msgs.msg import Status
from threading import Lock
from robot_controller import RobotController
from geometry_msgs.msg import WrenchStamped
from qt5_gui import Ui_MainWindow
from PyQt5 import QtCore, QtGui, QtWidgets


class KnobGui(Ui_MainWindow):
    def __init__(self) -> None:
        super().__init__()

        self.robot_controller = RobotController()

        # define the subscibers
        self.knob_current_pos = None
        self.knob_current_force = None
        self.tcp_wrench = None

    def knob_state_callback(self, data) -> None:
        # if knob state changed, then send the command to the robot
        rospy.loginfo("GET!")
        if self.knob_current_pos != data.position.data:
            CONTROL_MODE, TCP_AXIS, CONTROL_JOINT = self.check_current_selections()
            if CONTROL_MODE == "JOINT":
                rospy.logwarn("Future work: joint mode is not supported yet.")
                return
            elif CONTROL_MODE == "TCP":
                self.knob_current_pos = data.position.data
                self.knob_current_force = data.force.data
                iiwa_tcp_position = [-0.0827, 0.7009, 0.2761]
                iiwa_tcp_angle = [0.0, -0.0, 3.14]
                iiwa_tcp_force = [5, 5, 5]
                iiwa_tcp_position[TCP_AXIS] = iiwa_tcp_position[TCP_AXIS] + 0.001 * self.knob_current_pos
                if not self.robot_controller.sendMoveCartesianLinImpedence(iiwa_tcp_position, iiwa_tcp_angle, iiwa_tcp_force):
                    rospy.logwarn("Failed to send move position to robot: ...")
            else:
                rospy.logerr("Unknown mode: {}".format(CONTROL_MODE))
                return

    def tcp_wrench_callback(self, data) -> None:    
        """
        tcp_wrench_callback
          force: 
            x: -1.7054017799938483
            y: -3.5376136956553155
            z: -1.5299034579481523

        """
        self.tcp_wrench = data.wrench.force
        # add a threshold, if the force is larger than the threshold, then print the force
        if self.tcp_wrench.x > -0:
            rospy.loginfo("tcp wrench: {}".format(self.tcp_wrench))
            rospy.loginfo("knob current force: {}".format(self.knob_current_force))
            rospy.loginfo("knob current pos: {}".format(self.knob_current_pos))
    
    
if __name__ == "__main__":
    try:
        KnobGUI = KnobGui()
        app = QtWidgets.QApplication(sys.argv)
        MainWindow = QtWidgets.QMainWindow()
        # ui = Ui_MainWindow()
        # ui.setupUi(MainWindow)
        KnobGUI.setupUi(MainWindow)
        MainWindow.show()
        KnobGUI.setup_ROS()
    except rospy.ROSInterruptException:
        pass

    sys.exit(app.exec_())

