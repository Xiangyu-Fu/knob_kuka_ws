#!/usr/bin/env python3
# Author: Xiangyu Fu
# Date: 2023-09-01
# Description: This script is used to control the robot arm through the robot movement interface.

import rospy
from robot_movement_interface.msg import Command, CommandList, EulerFrame
from robot_movement_interface.msg import Result
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Int32, Float32
from knob_robot_control.msg import KnobState
from device_msgs.msg import Status
from geometry_msgs.msg import WrenchStamped
from threading import Lock


class RobotController:
    SPEED_FACTOR = 0.1  # Replace with appropriate value if needed
    HARDCODE_SPEED_CART = 1.0  # Replace with appropriate value if needed

    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)
        
        # # Publishers and Subscribers
        # self.pub = rospy.Publisher('/command_list', CommandList, queue_size=10)
        # self.iiwaDriverCommandList = rospy.Publisher('YOUR_TOPIC_NAME_HERE', CommandList, queue_size=10) # Replace with the correct topic name
        # rospy.Subscriber("/command_result", Result, self.command_result_callback)

        self.iiwa_driver_command_list = rospy.Publisher("/command_list", CommandList, queue_size=2)
        #self.iiwa_driver_command_result = rospy.Subscriber("/command_result", Result, self.command_result_callback)
        #self.iiwa_driver_joint_states = rospy.Subscriber("/joint_states", JointState, self.joint_states_callback)

        self.robot_tcp_state_subscriber = rospy.Subscriber("/tool_frame", EulerFrame, self.tool_frame_subscribe_callback)
        self.robot_tcp_state_subscriber = rospy.Subscriber("/tcp_wrench", WrenchStamped, self.tcp_wrench_subscribe_callback)

        self.knob_state_subscriber = rospy.Subscriber("/knob_state", KnobState, self.knob_state_callback)

        self.currentCommandId = 0

        self.mStatus = Status()
        self.mStatusMutex = Lock()

        self.robot_tcp_position_current = [0, 0, 0]
        self.robot_tcp_orientation_current = [0, 0, 0]
        self.robot_tcp_force_current = [0, 0, 0]

        self.robot_tcp_position_target = [0, 0, 0]
        self.robot_tcp_orientation_target = [0, 0, 0]
        self.robot_tcp_force_threshold = [5, 5, 5]

        self.knob_force = 0
        self.knob_position = 0
        self.knob_position_delta = 0

        self.position_factor = 0.01
        self.force_factor = 1
        self.velocity_factor = 1

        self.ROBOT_STATE_INITIALIZED = False

    def knob_state_callback(self, data):
        self.knob_position_delta = data.position.data - self.knob_position
        self.knob_position = data.position.data
        self.knob_force = data.force.data

        #print("self.knob_position_delta, self.knob_position, self.knob_force", self.knob_position_delta, self.knob_position, self.knob_force)

    def tool_frame_subscribe_callback(self, data) -> None:
        self.robot_tcp_position_current = [data.x+0.004, data.y - 0.005, data.z-0.03]
        self.robot_tcp_orientation_current = [data.alpha, data.beta, data.gamma]

        #print('self.robot_tcp_position_current, self.robot_tcp_orientation_current', self.robot_tcp_position_current, self.robot_tcp_orientation_current)

        self.ROBOT_STATE_INITIALIZED = True
        
    def tcp_wrench_subscribe_callback(self, data) -> None:
        self.robot_tcp_force_current = [data.wrench.force.x, data.wrench.force.y, data.wrench.force.z]

        #print('self.robot_tcp_force_current', self.robot_tcp_force_current)
        
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
        command.command_type = "LINIMPEDENCE"
        command.pose_type = "EULER_INTRINSIC_ZYX"

        command.pose.extend([float(val) for val in position])
        command.pose.extend([float(val) for val in angles])
        command.velocity.append(float(0.1))
        command.blending = [0, 0.0001]

        commandList = CommandList()
        commandList.commands.append(command)
        commandList.replace_previous_commands = True

        self.iiwa_driver_command_list.publish(commandList)
        print("Sending impedence move position to robot: {}, {}, {}, with speed {}".format(position[0], position[1], position[2], speed_data))

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

    def sendMoveCartesianLinImpedence(self, position, angles, force) -> bool:
        """

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
        command.command_type = "LINIMPEDENCE"
        command.pose_type = "EULER_INTRINSIC_ZYX"

        command.pose.append(float(position[0]))
        command.pose.append(float(position[1]))
        command.pose.append(float(position[2]))
        command.pose.append(float(angles[0]))
        command.pose.append(float(angles[1]))
        command.pose.append(float(angles[2]))
        command.velocity.append(float(speed_data))
        command.force_threshold = [float(val) for val in force]

        commandList = CommandList()
        commandList.commands.append(command)
        commandList.replace_previous_commands = True

        rospy.loginfo("Sending move position (impedence) to robot: {}, {}, {}, with speed {}".format(position[0], position[1], position[2], speed_data))

        self.iiwa_driver_command_list.publish(commandList)

        return True

    def run(self) -> None:
        feq = 100
        rate = rospy.Rate(feq) # 10hz
        
        # Sample data for testing
        # position_target = [-0.08374, 0.5547, 0.05418]
        # position_home = [-0.08374, 0.4547, 0.25418]
        # angles = [0.0, -0.0, 3.14]
        # joints = [1.5710205516437281, 0.26206090357094514, -2.6964464278686393e-05, -1.2529596421209066, 7.200128936299848e-05, 1.6281237054938813, -1.570994186372798]
        # force = [5, 5, 5]
        
        #while not rospy.is_shutdown():
            # if count == 2:
            #     self.sendMoveCartesianLinForce(position, angles, force)
            # Test sendMoveCartesianLin
            # self.sendMoveCartesianLin(position, angles)
            # if count%20 == 0:
            #     position_new = position
            #     count = 0
            # position_new[2] = position_new[2] + 0.002
            # self.sendMoveCartesianLinImpedence(position_new, angles, force)
            # count += 1

            # Test sendMoveCartesianLinForce
            # self.sendMoveCartesianLinForce(position, angles, force)
            # if count%10 == 0:
            #     position_new = [-0.08374, 0.5547, 0.23418]
            # position_new[2] = position_new[2] - 0.09

            #self.sendMoveCartesianLinImpedence(position_target, angles, force)
            # count += 1
            # rate.sleep()

            #rospy.sleep(10)
            #self.sendMoveCartesianLinImpedence(position_home, angles, force)
            #rospy.sleep(10)
        
        #rospy.spin()

        while not rospy.is_shutdown():
            if self.knob_position_delta != 0 and self.ROBOT_STATE_INITIALIZED:
                print('knob position delta:',  self.knob_position_delta)
                print('robot_tcp_position_current:',  self.robot_tcp_position_current)
                position_change = self.position_factor * self.knob_position_delta
                print("Posiiton change,", position_change)
                self.robot_tcp_position_target[0] = self.robot_tcp_position_current[0] + position_change
                self.robot_tcp_position_target[1] = self.robot_tcp_position_current[1]
                self.robot_tcp_position_target[2] = self.robot_tcp_position_current[2]
                print('robot_tcp_position_target:',  self.robot_tcp_position_target)
                
                #self.sendMoveCartesianLin(self.robot_tcp_position_current, self.robot_tcp_orientation_current)
                self.sendMoveCartesianLinImpedence(self.robot_tcp_position_current, self.robot_tcp_orientation_current, self.robot_tcp_force_threshold)

if __name__ == '__main__':
    controller = RobotController()
    controller.run()

