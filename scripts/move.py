#!/usr/bin/env python3

import rospy
# import the moveit_commander, which allows us to control the arms
import moveit_commander
import math
# import the custom message
from q_learning_project.msg import ArmMove

class Robot(object):

    def __init__(self):

        # initialize this node
        rospy.init_node('turtlebot3_dance')

        # Traffic status subscriber
        rospy.Subscriber("/arm_status", ArmMove, self.traffic_dir_received)

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator arm
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator gripper
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        # Reset arm position
        self.move_group_arm.go([0,0,0,0], wait=True)
        print("ready")

    def traffic_dir_received(self, data: ArmMove):
        # array of arm joint locations for joint 0

        # Joint 1: -162 to 162, 0
        # Joint 2: -103 to 90, 70
        # Joint 3: -53 to 79, -50
        # Joint 4: -100 to 117, -20
        # JOINTS 2, 3, 4 ADD UP TO 0

        # Predefined movements
        gripper_joint_close = [-0.01, -0.01]
        gripper_joint_open = [0, 0]

        # ________________________MY CODE______________________________________
        # TEST CODE: THIS IS THE MOVEMENT TO PICKUP
        # Move the arm forward into position to close gripper
        self.move_group_arm.go([0, math.radians(70), math.radians(-50), math.radians(-20)], wait=True)
        self.move_group_arm.stop()
        # Close gripper
        self.move_group_gripper.go(gripper_joint_close, wait = True)
        self.move_group_gripper.stop()
        # Lift item
        self.move_group_arm.go([0, math.radians(60), math.radians(-50), math.radians(-10)], wait=True)
        self.move_group_arm.stop()

        # ________________________MY CODE______________________________________


    def run(self):
        rospy.spin()

if __name__ == "__main__":
    robot = Robot()
    robot.run()