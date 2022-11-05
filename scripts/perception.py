#!/usr/bin/env python3

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Perception:

        def __init__(self):
                print("in init!")
                # set up ROS / cv bridge
                self.bridge = cv_bridge.CvBridge()

                # subscribe to the robot's RGB camera data stream
                self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                        Image, self.image_callback)
                # publish robot's velocity to move robot
                self.cmd_vel_pub = rospy.Publisher('cmd_vel',
                        Twist, queue_size=1)

                # save the object color given to us from the action
                # right now I am hard-coding it to be pink just to test my program
                # but eventually this will be extracted from the action the robot must take
                self.object_color = "green"

                # do the same for tag
                # self.tag_id = "1" ...(extract from given action)

                self.twist = Twist()
                print("just finished init!")
        # Identify_images should take in a frame image from a ROS topic that we subscribe to (bringup cam or something)
        # and it should be able to give the directions in an angle or eventually "go towards" an item in the
        # image of a specific color

        def identify_objects(self):
                return
        # make two sides, object side and tag side
        # when on tag side, we can only go to a new object
        # when on object side, we must pick up and object


        ####################################
        # We're given an action based on the state we're in
        # the action tells us which color object to pick up and which tag to drop it to
        # given the color of the object, i.e. "pink", check if camera detects the colored object
        # given the tag of the object, i.e. 1, check if camera detects the tag

        # my plan for robot object detection:
        # 1. robot should be able to identify if it is closely in front of the correct object given object's color
        #    (like maybe it can print out "__ object found!" if the camera can detect the right color object in front of it
        # 2. robot should also be able to detect correct color object from farther away (i.e. 4-6 meters away)
        # 3. Next, robot should be able to keep rotating until it detects the correct color object and stop once it does.

        ####################################

        def image_callback(self, msg):
                # print("in image_callback")
                # converts the incoming ROS message to cv2 format and HSV (hue, saturation, value)
                image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                # defines the upper and lower bounds for what should be considered 'blue'
                if self.object_color == "blue":
                    lower_color_bound = numpy.array([80, 155, 55])
                    upper_color_bound = numpy.array([110, 255, 255])
                # defines the upper and lower bounds for what is considered "pink"
                elif self.object_color == "pink":
                    lower_color_bound = numpy.array([130, 130, 150])
                    upper_color_bound = numpy.array([180, 220, 220])
                # defines the upper and lower bounds for what is considered "green"
                else:
                    lower_color_bound = numpy.array([10, 135, 120])
                    upper_color_bound = numpy.array([50, 215, 190])

                mask = cv2.inRange(hsv, lower_color_bound, upper_color_bound)

                # we now erase all pixels that aren't for given color
                h, w, d = image.shape
                search_top = int(3*h/4)
                search_bot = int(3*h/4 + 20)
                # mask[0:search_top, 0:w] = 0
                # mask[search_bot:h, 0:w] = 0

                # using moments() function, determine the center of the selected color's pixels
                M = cv2.moments(mask)
                # if there are any select color's pixels found
                if M['m00'] > 0:
                        # determine the center of the selected color's pixels in the image
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
                        # self.cmd_vel_pub.publish(self.twist)
                        print("selected color object found! -- ", self.object_color)
                # show the debugging window
                cv2.imshow("window", image)
                cv2.waitKey(3)

if __name__ == '__main__':

        rospy.init_node('perception')
        follower = Perception()
        rospy.spin()
