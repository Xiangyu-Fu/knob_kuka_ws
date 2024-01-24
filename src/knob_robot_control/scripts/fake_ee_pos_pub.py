#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import WrenchStamped, PoseStamped
from knob_robot_control.msg import KnobState, KnobCommand
import random
import math

def sigmoid(x):
    return 1 / (1 + math.exp(-x))

class FakePublisher:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('fake_publisher', anonymous=True)

        self.knob_current_pos = None
        self.knob_current_force = None

        # Publishers
        self.knob_state_sub = rospy.Subscriber("/knob_state", KnobState, self.knob_state_callback)
        self.tcp_wrench_pub = rospy.Publisher('/tcp_wrench', WrenchStamped, queue_size=10)
        self.tool_frame_pub = rospy.Publisher('/tool_frame', PoseStamped, queue_size=10)

        # Publishing rate
        self.rate = rospy.Rate(10) # 10hz

        self.start_time = rospy.get_time()

        self.position_home = [-0.08374, 0.4547, 0.25418]

        

    def publish_tcp_wrench(self):
        current_time = rospy.get_time() - self.start_time
        wrench_msg = WrenchStamped()
        wrench_msg.header.stamp = rospy.Time.now()

        wrench_msg.wrench.force.x = random.uniform(-10, 10)
        wrench_msg.wrench.force.y = random.uniform(-10, 10)

        # if current_time < 5:
        #     wrench_msg.wrench.force.z = 1 + random.uniform(-0.1, 0.1)
        # elif 5 <= current_time < 15:
        #     wrench_msg.wrench.force.z = 4 * sigmoid(current_time - 10) + 1 + random.uniform(-0.1, 0.1)
        # elif 15 <= current_time < 20:
        #     wrench_msg.wrench.force.z = 5 + random.uniform(-0.1, 0.1)
        # else:
        #     self.start_time = rospy.get_time()

        if self.knob_current_pos is None:
            wrench_msg.wrench.force.z = 1 + random.uniform(-0.1, 0.1)
        else:
            if self.knob_current_pos < 20:
                wrench_msg.wrench.force.z = 1 + random.uniform(-0.1, 0.1)
            elif 20 <= self.knob_current_pos < 40:
                wrench_msg.wrench.force.z = self.knob_current_pos -15 + random.uniform(-0.1, 0.1)
            elif 40 <= self.knob_current_pos < 60:
                wrench_msg.wrench.force.z = 40 + random.uniform(-0.1, 0.1)

        wrench_msg.wrench.torque.x = random.uniform(-5, 5)
        wrench_msg.wrench.torque.y = random.uniform(-5, 5)
        wrench_msg.wrench.torque.z = random.uniform(-5, 5)

        self.tcp_wrench_pub.publish(wrench_msg)

    def publish_tool_frame(self):
        current_time = rospy.get_time() - self.start_time
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.position.x = self.position_home[0] + random.uniform(-0.01, 0.01)
        pose_msg.pose.position.y = self.position_home[1] +random.uniform(-0.01, 0.01)
        # pose_msg.pose.position.z = random.uniform(-0.1, 0.1)
        if current_time < 5:
            pose_msg.pose.position.z = self.position_home[2] + random.uniform(-0.01, 0.01)
        elif 5 <= current_time < 15:
            pose_msg.pose.position.z = - 0.15 * sigmoid(current_time - 10) + self.position_home[2] + random.uniform(-0.01, 0.01)
        elif 15 <= current_time < 20:
            pose_msg.pose.position.z = - 0.15  + self.position_home[2] + random.uniform(-0.01, 0.01)
        else:
            self.start_time = rospy.get_time()


        angle = random.uniform(0, 2*math.pi)
        pose_msg.pose.orientation.x = math.sin(angle / 2)
        pose_msg.pose.orientation.y = 0
        pose_msg.pose.orientation.z = 0
        pose_msg.pose.orientation.w = math.cos(angle / 2)



        self.tool_frame_pub.publish(pose_msg)

    def knob_state_callback(self, data):
        self.knob_current_pos = data.position.data
        self.knob_current_force = data.force.data
        # print("knob_current_pos: ", self.knob_current_pos)

    def run(self):
        while not rospy.is_shutdown():
            try:
                self.publish_tcp_wrench()
                self.publish_tool_frame()
            except rospy.ROSInterruptException:
                pass
            self.rate.sleep()

if __name__ == '__main__':
    try:
        fake_publisher = FakePublisher()
        fake_publisher.run()
    except rospy.ROSInterruptException:
        pass
