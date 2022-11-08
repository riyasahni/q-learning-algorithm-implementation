#!/usr/bin/env python3

import rospy
from q_learning_project.msg import ArmMove
import numpy as np

class TrafficNode(object):
    def __init__(self):
        # Set up traffic status publisher
        self.traffic_status_pub = rospy.Publisher("/arm_status", ArmMove)

        # Counter to loop publishing direction with
        self.direction_counter = 0
        rospy.sleep(1)

    def run(self):
        # Once every 10 seconds
        rate = rospy.Rate(0.1)
        while (not rospy.is_shutdown()):
            trafficMsg = ArmMove()
            trafficMsg.direction = self.direction_counter % 3
            self.direction_counter += 1
            self.traffic_status_pub.publish(trafficMsg)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node("/arm_control")
    traffic_node = TrafficNode()
    traffic_node.run()