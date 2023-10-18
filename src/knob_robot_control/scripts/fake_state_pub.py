#!/usr/bin/env python3
# Author: Xiangyu Fu
# Date: 2023-10-18
# Description: This script is used to control the robot arm through the robot movement interface.
import rospy
from knob_robot_control.msg import KnobState

rospy.init_node('fake_state_pub')
pub = rospy.Publisher('/knob_state', KnobState, queue_size=10)

rate = rospy.Rate(10) # 10hz
import math

while not rospy.is_shutdown():
    fake_state = KnobState()
    fake_state.header.stamp = rospy.Time.now()
    fake_state.mode.data = "TCP"
    fake_state.position.data = int(10 * math.sin(rospy.Time.now().to_sec()))
    fake_state.force.data = 10 * math.cos(rospy.Time.now().to_sec())
    
    pub.publish(fake_state)
    rate.sleep()
