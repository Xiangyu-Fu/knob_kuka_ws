#!/usr/bin/env python3
# Author: Xiangyu Fu
# Date: 2023-09-01
# Description: This script is used to control the robot arm through the robot movement interface.

import rospy
from robot_movement_interface.msg import Command, CommandList
from robot_movement_interface.msg import Result
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Int32, Float32
from knob_robot_control.msg import KnobState
from device_msgs.msg import Status
from threading import Lock


class RobotController:
    SPEED_FACTOR = 0.02  # Replace with appropriate value if needed
    HARDCODE_SPEED_CART = 1.0  # Replace with appropriate value if needed

    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)
        
        # # Publishers and Subscribers
        # self.pub = rospy.Publisher('/command_list', CommandList, queue_size=10)
        # self.iiwaDriverCommandList = rospy.Publisher('YOUR_TOPIC_NAME_HERE', CommandList, queue_size=10) # Replace with the correct topic name
        # rospy.Subscriber("/command_result", Result, self.command_result_callback)

        self.iiwa_driver_command_list = rospy.Publisher("/command_list", CommandList, queue_size=2)
        self.iiwa_driver_command_result = rospy.Subscriber("/command_result", Result, self.command_result_callback)
        self.iiwa_driver_joint_states = rospy.Subscriber("/joint_states", JointState, self.joint_states_callback)

        # define the subscibers
        self.knob_state_sub = rospy.Subscriber("/knob_state", KnobState, self.knob_state_callback)
        self.knob_current_pos = None
        self.knob_current_force = None

        self.currentCommandId = 0

        self.mStatus = Status()
        self.mStatusMutex = Lock()

    def joint_states_callback(self, data) -> None:
        # rospy.loginfo("Joint states received: ...")  
        pass

    def knob_state_callback(self, data) -> None:
        if self.knob_current_pos != data.position.data:
            self.knob_current_pos = data.position.data
            self.knob_current_force = data.force.data
            position = [0.0, 0.5, 0.6]
            position[1] = position[1] + 0.002 * self.knob_current_pos
            if not self.sendMoveCartesianLin(position, [0.0, 0.0, 0.0]):
                rospy.loginfo("Failed to send move position to robot: ...")
        
    def send_command_to_robot(self, commands, replace_previous=True) -> None:
        """
        Send a list of commands to the robot.
        
        :param commands: List of Command objects to be sent to the robot.
        :param replace_previous: Whether to replace previous commands or not.
        """
        cmd_list = CommandList()
        cmd_list.commands = commands
        cmd_list.replace_previous_commands = replace_previous
        self.pub.publish(cmd_list)

    def command_result_callback(self, data) -> None:
        """
        Callback function to handle feedback from the robot.
        
        :param data: Result message from the robot.
        """
        # Handle the feedback. For now, just print it.
        print(f"Command ID: {data.command_id}, Result Code: {data.result_code}, Additional Info: {data.additional_information}")

    def getSpeed(self, speed_data):
        # Implement the method to retrieve speed data
        # For now, I am assuming it always returns True with a dummy speed
        speed_data = 0.1
        return True
    
    def getStatus(self, data):
            with self.mStatusMutex:
                data.deviceName = self.mStatus.deviceName
                data.mode = self.mStatus.mode
                data.action = self.mStatus.action
                data.actionIdentifier = self.mStatus.actionIdentifier
                data.result = self.mStatus.result
                data.timeStamp = self.mStatus.timeStamp
            
            # print the status
            print("Device Name: ", data.deviceName)
            print("Mode: ", data.mode)
            print("Action: ", data.action)
            print("Action Identifier: ", data.actionIdentifier)
            print("Result: ", data.result)
            print("Time Stamp: ", data.timeStamp)

    def sendMoveCartesianLin(self, position, angles) -> bool:
        """
        Send a move command to the robot.
        :param position: List of 3 floats representing the position of the end effector.
        :param angles: List of 3 floats representing the orientation of the end effector.
        :return: True if the command was sent successfully, False otherwise.
        """
        speed_data = None
        if not self.getSpeed(speed_data):
            return False

        speed_data = self.HARDCODE_SPEED_CART * self.SPEED_FACTOR

        command = Command()
        command.command_id = self.currentCommandId + 1
        self.currentCommandId = command.command_id
        command.command_type = "LIN"
        command.pose_type = "EULER_INTRINSIC_ZYX"

        command.pose.extend([float(val) for val in position])
        command.pose.extend([float(val) for val in angles])
        command.velocity.append(float(0.1))
        command.blending = [0, 0.0001]

        commandList = CommandList()
        commandList.commands.append(command)
        commandList.replace_previous_commands = True

        rospy.loginfo("Sending move position to robot: ...")  # Complete the log message
        print(commandList)

        self.iiwa_driver_command_list.publish(commandList)

        return True

    def sendMoveCartesianLinForce(self, position, angles, force) -> bool:
        """
        TODO: Future work. Send a move command with force to the robot.
        :param position: List of 3 floats representing the position of the end effector.
        :param angles: List of 3 floats representing the orientation of the end effector.
        :param force: List of 3 floats representing the force threshold of the end effector.
        :return: True if the command was sent successfully, False otherwise.
        """
        speed_data = None
        if not self.getSpeed(speed_data):
            return False

        speed_data = self.HARDCODE_SPEED_CART * self.SPEED_FACTOR

        command = Command()
        command.command_id = self.currentCommandId + 1
        command.command_type = "LINFORCE"
        command.pose_type = "EULER_INTRINSIC_ZYX"

        command.pose.append(float(position[0]))
        command.pose.append(float(position[1]))
        command.pose.append(float(position[2]))
        command.pose.append(float(angles[0]))
        command.pose.append(float(angles[1]))
        command.pose.append(float(angles[2]))
        command.velocity.append(float(speed_data))
        command.blending = [0, 0.0001]
        command.force_threshold = [float(val) for val in force]

        commandList = CommandList()
        commandList.commands.append(command)
        commandList.replace_previous_commands = True

        rospy.loginfo("Sending move position (force) to robot: {}, {}, {}, with speed {}".format(position[0], position[1], position[2], speed_data))

        self.iiwa_driver_command_list.publish(commandList)

        return True

    def sendMoveJointPtp(self, q) -> bool:
        """
        :param q: List of 7 floats representing the joint angles.
        :return: True if the command was sent successfully, False otherwise.
        """
        if len(q) != 7:
            rospy.logerr("Got move vector with invalid size: %d != 7", len(q))
            return False
        
        speed_data = None
        if not self.getSpeed(speed_data):
            return False
        
        speed_data = self.HARDCODE_SPEED_CART * self.SPEED_FACTOR

        command = Command()
        command.command_id = self.currentCommandId + 1
        self.currentCommandId = command.command_id
        command.command_type = "PTP"
        command.pose_type = "JOINTS"

        for i in range(7):
            command.pose.append(float(q[i]))
            command.velocity.append(float(speed_data))

        command.blending = [0, 0.0001]

        commandList = CommandList()
        commandList.commands.append(command)
        commandList.replace_previous_commands = True

        rospy.loginfo("Sending move position to robot: {}, {}, {}, {}, {}, {}, {}, with speed: {}".format(q[0], q[1], q[2], q[3], q[4], q[5], q[6], speed_data))  

        self.iiwa_driver_command_list.publish(commandList)
        return True

    def run(self) -> None:
        feq = 1
        rate = rospy.Rate(feq) # 10hz
        
        # Sample data for testing
        position = [-0.0827, 0.7209, 0.2761]
        angles = [0.0, -0.0, 3.14]
        joints = [1.5710205516437281, 0.26206090357094514, -2.6964464278686393e-05, -1.2529596421209066, 7.200128936299848e-05, 1.6281237054938813, -1.570994186372798]
        force = [0.01, 0.01, 0.01]

        FLAG = True
        while not rospy.is_shutdown():
            # Test sendMoveCartesianLin
            # self.sendMoveCartesianLin(position, angles)
            self.sendMoveJointPtp(joints)
            rospy.sleep(10)
            self.sendMoveCartesianLin(position, angles)
            rospy.sleep(10)
        
        # # Test sendMoveCartesianLinForce
        # self.sendMoveCartesianLinForce(position, angles, force)


if __name__ == '__main__':
    controller = RobotController()
    controller.run()

