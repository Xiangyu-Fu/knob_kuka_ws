#!/usr/bin/env python3
# Author: Xiangyu Fu
# Date: 2023-09-01
# Description: This script is used to control the robot arm to move to a specific pose.
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

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))
    
from moveit_commander.conversions import pose_to_list

def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class MoveGroupPythonInterfaceTutorial:
    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

        robot = moveit_commander.RobotCommander()

        scene = moveit_commander.PlanningSceneInterface()
        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        
        # create a move group
        group_name = "full_lwr"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        # Attach a small box to the robot arm
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = robot.get_planning_frame()
        box_pose.pose.position.x = 0.5
        box_pose.pose.position.y = 0.0
        box_pose.pose.position.z = 0.5
        box_pose.pose.orientation.w = 1.0

        box_name = "box"
        # scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))

        self.go_to_joint_state()
        self.home_pose = self.move_group.get_current_pose().pose

        while not rospy.is_shutdown():
            # start pose
            plan, fraction = self.plan_cartesian_path(euler_angle = 20)
            self.execute_plan(plan)

            # end pose
            plan, fraction = self.plan_cartesian_path(x = -0.2, z = -0.1, euler_angle = -40)
            self.execute_plan(plan)
        
            rospy.sleep(1)  # Let ROS do some housekeeping.

        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


    def plan_cartesian_path(self, x = 0.2, z = 0, euler_angle = 0):
        """
        Plans a Cartesian path in 3-D space.
        args:
            x: x-axis offset based on the home pose in world frame
            z: z-axis offset based on the home pose in world frame
            euler_angle: rotation angle around y-axis
        """
        move_group = self.move_group

        waypoints = []

        euler_angles = tf.transformations.euler_from_quaternion(
            [self.home_pose.orientation.w, self.home_pose.orientation.x, self.home_pose.orientation.y, self.home_pose.orientation.z], axes='sxyz'
        )

        # Rotate by 45 degrees around the y-axis
        rotation_angle = np.radians(euler_angle)  # Convert 45 degrees to radians
        rotation_axis = np.array([0, 1, 0])  # Y-axis
        new_euler_angles = euler_angles + rotation_angle * rotation_axis

        # Convert Euler angles back to quaternion
        new_quaternion = tf.transformations.quaternion_from_euler(
            new_euler_angles[0], new_euler_angles[1], new_euler_angles[2], axes='sxyz'
        )

        # start with the current pose
        wpose = move_group.get_current_pose().pose
        wpose.position.x = self.home_pose.position.x + x  
        wpose.position.z = self.home_pose.position.z + z 
        wpose.orientation.w = new_quaternion[0]
        wpose.orientation.x = new_quaternion[1]
        wpose.orientation.y = new_quaternion[2]
        wpose.orientation.z = new_quaternion[3]
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        return plan, fraction
    
    def execute_plan(self, plan):
        move_group = self.move_group
        move_group.execute(plan, wait=True)

    def joint_constraints(self):
        constraints = moveit_msgs.msg.Constraints()

        jcm = moveit_msgs.msg.JointConstraint()
        jcm.joint_name = "lwr_a1_joint"
        jcm.position = 0
        jcm.tolerance_above = 0.1
        jcm.tolerance_below = 0.1
        jcm.weight = 0.5
        constraints.joint_constraints.append(jcm)

    def go_to_joint_state(self):
        move_group = self.move_group

        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -0.602
        joint_goal[2] = 0
        joint_goal[3] = -1.744
        joint_goal[4] = 0
        joint_goal[5] = -0.602
        joint_goal[6] = 0

        move_group.go(joint_goal, wait=True)
        move_group.stop()

        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)


if __name__ == "__main__":
    try:
        MoveGroupPythonInterfaceTutorial()
    except rospy.ROSInterruptException:
        pass