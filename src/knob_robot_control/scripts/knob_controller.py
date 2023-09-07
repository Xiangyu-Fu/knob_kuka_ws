#!/usr/bin/env python3
# Author: Xiangyu Fu
# Date: 2023-09-01
# Description: This script is used to control the robot arm through the robot movement interface.

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

class KnobController(RobotController):
    def __init__(self):
        super().__init__()

        # define the subscibers
        self.knob_state_sub = rospy.Subscriber("/knob_state", KnobState, self.knob_state_callback)
        self.tcp_wrench_sub = rospy.Subscriber("/tcp_wrench", WrenchStamped, self.tcp_wrench_callback)
        self.knob_command_pub = rospy.Publisher("/knob_command", KnobCommand, queue_size=10)
        self.knob_current_pos = None
        self.knob_current_force = None
        self.tcp_wrench = None

        while not rospy.is_shutdown():
            rospy.spin()
            
    def change_knob_state(self) -> None:
        # publish example
        knob_command = KnobCommand()
        knob_command.header.stamp = rospy.Time.now()
        knob_command.num_positions.data = 11
        knob_command.position.data = 0
        knob_command.position_width_radians.data = 10 * math.pi / 180
        knob_command.detent_strength_unit.data = 0.0
        knob_command.endstop_strength_unit.data = 1.0
        knob_command.snap_point.data = 1.1
        knob_command.text.data = "Bounded 0-10\nNo detents"
        self.knob_command_pub.publish(knob_command)

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

    def knob_state_callback(self, data) -> None:
        if self.knob_current_pos != data.position.data:
            self.knob_current_pos = data.position.data
            self.knob_current_force = data.force.data
            position = [-0.0827, 0.7009, 0.2761]
            position[0] = position[0] + 0.001 * self.knob_current_pos
            if not self.sendMoveCartesianLin(position, [0.0, -0.0, 3.14]):
                rospy.loginfo("Failed to send move position to robot: ...")
    
if __name__ == "__main__":
    try:
        KnobController()
    except rospy.ROSInterruptException:
        pass

