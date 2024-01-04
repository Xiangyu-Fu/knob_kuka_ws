#!/usr/bin/env python3
# Author: Xiangyu Fu
# Date: 2023-09-01
# Description: This script is used to control the robot arm through MoveIt! in the simulation environment.

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf
import copy
from moveit_msgs.msg import PlanningScene
from visualization_msgs.msg import Marker
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Int32, Float32
from knob_robot_control.msg import KnobState


class KnobCommander:
    def __init__(self):
        super(KnobCommander, self).__init__()

        rospy.init_node("knob_commander", anonymous=True)

        self.knob_command_pub = rospy.Publisher("/knob_command", KnobState, queue_size=10)

        while not rospy.is_shutdown():
            knob_command = KnobState()
            knob_command.header.stamp = rospy.Time.now()
            knob_command.mode.data = "fine value"
            self.knob_command_pub.publish(knob_command)

            rospy.loginfo(knob_command)
            rospy.sleep(1.0)






if __name__ == "__main__":
    try:
        KnobCommander()
    except rospy.ROSInterruptException:
        pass