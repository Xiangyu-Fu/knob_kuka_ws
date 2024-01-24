#!/usr/bin/env python3
# Date: 2024-01-24
import sys
import math
import rospy
from knob_robot_control.msg import KnobState, KnobCommand
from robot_movement_interface.msg import EulerFrame
from std_msgs.msg import String, Int32, Float32
from threading import Lock, Thread
from robot_controller import RobotController
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState



class knob_haptic_feedback_controller():
    def __init__(self):
        self.knob_current_pos = None
        self.knob_current_force = None
        self.tcp_wrench = None

        # init kmob command
        self.num_positions = 100
        self.position = 0
        self.position_width_radians = 10 * math.pi / 180
        self.detent_strength_unit = 0
        self.endstop_strength_unit = 0
        self.snap_point = 1.1

        self.KNOB_MODE = "NORMAL" # NORMAL, FORCE, BOUND


    def setup_ROS(self):
        rospy.init_node('knob_haptic_feedback_controller', anonymous=True)

        self.knob_state_sub = rospy.Subscriber("/knob_state", KnobState, self.knob_state_callback)
        # self.joint_state_sub = rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)
        self.tcp_wrench_sub = rospy.Subscriber("/tcp_wrench", WrenchStamped, self.tcp_wrench_callback)
        # self.tcp_state_sub = rospy.Subscriber("/tool_frame", EulerFrame, self.tcp_state_callback) 

        self.knob_command_pub = rospy.Publisher("/knob_command", KnobCommand, queue_size=10)


        knob_command = KnobCommand()
        knob_command.header.stamp = rospy.Time.now()
        knob_command.num_positions.data = 100
        knob_command.position.data = 0
        knob_command.position_width_radians.data = 10 * math.pi / 180
        knob_command.detent_strength_unit.data = 1.0
        knob_command.endstop_strength_unit.data = 2.0
        knob_command.snap_point.data = 1.1
        knob_command.text.data = "Bounded 0-10\nNo detents"
        self.knob_command_pub.publish(knob_command)

    def publish_force(self, force=1.0) -> None:
        knob_command = KnobCommand()
        knob_command.text.data = "force"
        knob_command.tcp_force.data = float(force)
        self.knob_command_pub.publish(knob_command)

    def publish_knob_command(self) -> None:  
        knob_command = KnobCommand()
        knob_command.header.stamp = rospy.Time.now()
        knob_command.num_positions.data = int(self.num_positions.value())
        knob_command.position.data = int(self.position.value())
        knob_command.position_width_radians.data = float(self.position_width_radians.value())
        knob_command.detent_strength_unit.data = float(self.detent_strength_unit.value())
        knob_command.endstop_strength_unit.data = float(self.endstop_strength_unit.value())
        knob_command.snap_point.data = float(self.snap_point.value())
        knob_command.text.data = self.text_line_edit.text()
        knob_command.tcp_force.data = float(1.0)    
        self.knob_command_pub.publish(knob_command)

    def change_knob_state(self, position=0, position_width_radians = 10 * math.pi / 180, detent_strength_unit = 1.0, endstop_strength_unit=2.0, snap_point=1.1) -> None:
        # publish example
        knob_command = KnobCommand()
        knob_command.header.stamp = rospy.Time.now()
        knob_command.num_positions.data = self.num_positions
        knob_command.position.data = position
        knob_command.position_width_radians.data = position_width_radians
        knob_command.detent_strength_unit.data = detent_strength_unit
        knob_command.endstop_strength_unit.data = endstop_strength_unit
        knob_command.snap_point.data = snap_point
        knob_command.text.data = "Bounded 0-10\nNo detents"
        self.knob_command_pub.publish(knob_command)

    def tcp_wrench_callback(self, data) -> None:    
        """
        tcp_wrench_callback
          force: 
            x: (-30, 30)
            y: (-30, 30)
            z: (-60, 30)

        """
        FORCE_THRESHOLD = 24
        BOUND_THRESHOLD = 24
        BUFFER_ZONE = 1

        current_force = data.wrench.force.z
        clamp_force = max(1, min(abs(current_force)/8, 4))

        print("\r", current_force, "                      ", end="")

        if FORCE_THRESHOLD - BUFFER_ZONE < current_force < BOUND_THRESHOLD + BUFFER_ZONE:
            self.publish_force(clamp_force)
        else:
            if current_force <= FORCE_THRESHOLD - BUFFER_ZONE:
                if self.KNOB_MODE != "NORMAL":
                    self.num_positions = 50
                    self.change_knob_state(position=self.knob_current_pos, position_width_radians=10 * math.pi / 180, snap_point=1.1)
                    self.KNOB_MODE = "NORMAL"
                self.publish_force(1.0)

            elif FORCE_THRESHOLD + BUFFER_ZONE <= current_force < BOUND_THRESHOLD + BUFFER_ZONE:
                if self.KNOB_MODE != "FORCE":
                    self.change_knob_state(position=self.knob_current_pos, position_width_radians=15 * math.pi / 180, detent_strength_unit=2.0)
                    self.KNOB_MODE = "FORCE"
                self.publish_force(clamp_force)

            else: # current_force >= BOUND_THRESHOLD + BUFFER_ZONE
                if self.KNOB_MODE != "BOUND":
                    self.num_positions = self.knob_current_pos + 2
                    print(self.num_positions)
                    self.change_knob_state(position=self.knob_current_pos, position_width_radians=20 * math.pi / 180, endstop_strength_unit=2.0, snap_point=1.5)
                    self.KNOB_MODE = "BOUND"
                # self.publish_force(clamp_force)
            self.publish_force(clamp_force)

    def knob_state_callback(self, data) -> None:

        self.knob_current_pos = data.position.data
        self.knob_current_force = data.force.data

        # if knob state changed, then send the command to the robot
        if self.knob_current_pos != data.position.data:
            CONTROL_MODE, TCP_AXIS, CONTROL_JOINT = self.check_current_selections()
            if CONTROL_MODE == "JOINT":
                rospy.logwarn("Future work: joint mode is not supported yet.")
                return
            elif CONTROL_MODE == "TCP":
                self.knob_current_pos = data.position.data
                self.knob_current_force = data.force.data
                position = [-0.0827, 0.7009, 0.2761]
                position[int(TCP_AXIS)] = position[int(TCP_AXIS)] + 0.001 * self.knob_current_pos
                print("\r",position, "                      ", end="")
                # if not self.sendMoveCartesianLin(position, [0.0, -0.0, 3.14]):
                #     rospy.loginfo("Failed to send move position to robot: ...")
            else:
                rospy.logerr("Unknown mode: {}".format(CONTROL_MODE))
                return
            
    def run(self) -> None:
        self.change_knob_state()
        while rospy.is_shutdown():
            pass
        rospy.spin()
            
if __name__ == "__main__":
    knob_haptic_feedback_controller = knob_haptic_feedback_controller()

    # Setup ROS
    knob_haptic_feedback_controller.setup_ROS()

    knob_haptic_feedback_controller.run()

