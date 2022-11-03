#!/usr/bin/env python3

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Perception:

        def __init__(self):
                
                rospy.init_node("perception")
                # NAME FUNCTION HERE ^
                # set up ROS / cv bridge
                self.bridge = cv_bridge.CvBridge()

                # subscribe to the robot's RGB camera data stream
                self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                        Image, self.image_callback)

                self.cmd_vel_pub = rospy.Publisher('cmd_vel',
                        Twist, queue_size=1)

                self.twist = Twist()

        def identify_objects(self):
                return
        # make two sides, object side and tag side
        # when on tag side, we can only go to a new object
        # when on object side, we must pick up and object

        def image_callback(self, msg):

                # converts the incoming ROS message to cv2 format and HSV (hue, saturation, value)
                image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

                # defines the upper and lower bounds for what should be considered 'orange'
                lower_orange = numpy.array([ 10, 50, 150])
                upper_orange = numpy.array([20, 255, 255])
                mask = cv2.inRange(hsv, lower_orange, upper_orange)

                # we now erase all pixels that aren't orange
                h, w, d = image.shape
                search_top = int(3*h/4)
                search_bot = int(3*h/4 + 20)
                mask[0:search_top, 0:w] = 0
                mask[search_bot:h, 0:w] = 0

                # using moments() function, determine the center of the orange pixels
                M = cv2.moments(mask)
                # if there are any orange pixels found
                if M['m00'] > 0:
                        # determine the center of the orange pixels in the image
                        cx = int(M['m10']/M['m00'])
                        cy = int(M['m01']/M['m00'])

                        # visualize a red circle in our debugging window to indicate
                        # the center point of the orange pixels
                        cv2.circle(image, (cx, cy), 20, (0,0,255), -1)

                        # TODO: based on the location of the line (approximated
                        #       by the center of the orange pixels), implement
                        #       proportional control to have the robot follow
                        #       the orange line
                        err = w/2 - cx
                        k_p = 1.0 / 200.0
                        self.twist.linear.x = 0.2
                        self.twist.angular.z = k_p * err
                        self.cmd_vel_pub.publish(self.twist)

                # show the debugging window
                cv2.imshow("window", image)
                cv2.waitKey(3)

if __name__ == '__main__':

        rospy.init_node('line_follower')
        follower = Perception()
        rospy.spin()