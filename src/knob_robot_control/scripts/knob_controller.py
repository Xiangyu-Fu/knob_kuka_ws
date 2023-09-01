#!/usr/bin/env python3
import rospy
from robot_movement_interface.msg import Command, CommandList
from robot_movement_interface.msg import Result
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Int32, Float32

class RobotController:
    SPEED_FACTOR = 1.0  # Replace with appropriate value if needed
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

        # subscribe for the Knob's State
        self.knob_pos_sub = rospy.Subscriber("/knob_position", Int32, self.knob_position_callback)
        self.knob_force_sub = rospy.Subscriber("/knob_force", Float32, self.knob_force_callback)

        self.knob_current_pos = None
        self.knob_current_force = None

        self.currentCommandId = 0

    def joint_states_callback(self, data) -> None:
        # rospy.loginfo("Joint states received: ...")  
        pass

    def knob_position_callback(self, data) -> None:
        self.knob_current_pos = data.data


    def knob_force_callback(self, data) -> None:   
        self.knob_current_force = data.data
       
        
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
        speed_data = 1.0
        return True

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
        command.command_type = "LIN"
        command.pose_type = "EULER_INTRINSIC_ZYX"

        command.pose.extend([float(val) for val in position])
        command.pose.extend([float(val) for val in angles])
        command.velocity.append(float(speed_data))
        command.blending = [0, 0.0001]

        commandList = CommandList()
        commandList.commands.append(command)
        commandList.replace_previous_commands = True

        rospy.loginfo("Sending move position to robot: ...")  # Complete the log message

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

        command.pose.extend([float(val) for val in position])
        command.pose.extend([float(val) for val in angles])
        command.velocity.append(float(speed_data))
        command.blending = [0, 0.0001]
        command.force_threshold = [float(val) for val in force]

        commandList = CommandList()
        commandList.commands.append(command)
        commandList.replace_previous_commands = True

        rospy.loginfo("Sending move position (force) to robot: ...")  # Complete the log message

        self.iiwa_driver_command_list.publish(commandList)

        return True

    def run(self) -> None:
        feq = 1
        rate = rospy.Rate(feq) # 10hz
        
        # Sample data for testing
        position = [0.1, 0.2, 0.3]
        angles = [0.1, 0.2, 0.3]
        force = [0.5, 0.5, 0.5]

        while not rospy.is_shutdown():
            # Test sendMoveCartesianLin
            self.sendMoveCartesianLin(position, angles)
            rate.sleep()
        
        # Test sendMoveCartesianLinForce
        self.sendMoveCartesianLinForce(position, angles, force)


if __name__ == '__main__':
    controller = RobotController()
    controller.run()

