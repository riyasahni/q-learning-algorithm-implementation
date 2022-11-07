#!/usr/bin/env python3

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, Vector3
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class Perception:
    def __init__(self):
        print("in init!")
        # set up ROS / cv bridge
        self.bridge = cv_bridge.CvBridge()

        # initialize debugging window:
        cv2.namedWindow("window", 1)

        # subscribe to the robot's RGB camera data stream
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                Image, self.image_callback)
        # publish robot's velocity to move robot
        self.cmd_vel_pub = rospy.Publisher('cmd_vel',
            Twist, queue_size=1)
        # subscribe to lidar
        # self.scanner = rospy.Subscriber('/scan', LaserScan, self.identify_objects)
        # publish arm movements
        self.arm_pub = rospy.Publisher('joint_state_controller', JointTrajectory, queue_size = 2)


        # save the object color given to us from the action
        # right now I am hard-coding it to be pink just to test my program
        # but eventually this will be extracted from the action the robot must take
        self.object_color = "green"
        # do the same for tag
        self.tag = 1

        # set a flag for if we're dropping or picking up color object
        # so that robot knows what to look for in camera
        # SETTING IT TO FALSE RIGHT NOW JUST FOR TESTING (BUT WILL SET TO TRUE TO START OFF WITH FOR THE REAL THING)
        self.picking_up = False

        self.twist = Twist()

        print("just finished init!")

        self.pickup()

#        def move_to_target(self, data : LaserScan):
#                dataList = data.ranges
#                target = dataList.index(min(dataList[0:45], dataList[315:359]))
#                dist_to_obst = data[target]
#                if self.target_distance_from_center < -10: # If x is to the left,
#                        ang = .0348889 * self.target_distance_from_center
#                        lin = .1 * dist_to_obst
#                elif self.target_distance_from_center > 10: # x is right
#                        ang = .0348889 * self.target_distance_from_center
#                        lin = .1 * dist_to_obst
#                else: # x is center-ish
#                        ang = 0
#                        lin = 1 * dist_to_obst
#
#
#                turn = Twist(
#                        linear=Vector3(lin, 0, 0),
#                        angular=Vector3(0, 0, ang)
#                )
#
#                self.robot_movement_pub.publish(turn)

    def pickup(self):
        # JointTrajectory() // THIS IS THE MESSAGE FOR MOVING THE ARM
            # Args: 
            #      string[] joint_names
            #      JointTrajectoryPoint[] points

            # Type: JointTrajectoryPoint 
                
            # Args:
            #     float64[] positions
            #     float64[] velocities
            #     float64[] accelerations
            #     float64[] effort
            #     duration time_from_start
            arm_movement_point = JointTrajectoryPoint(
                positions = [.1, 1, 0, 0],
                velocities = [1, 1, 0, 0]
            )

            arm_movement_msg = JointTrajectory(
                points = arm_movement_point
            )

            print(arm_movement_msg.points)
            print(arm_movement_msg.points.positions)

            self.arm_pub.publish(arm_movement_msg)
        
            # https://emanual.robotis.com/docs/en/platform/openmanipulator_x/quick_start_guide_basic_operation/
            # http://docs.ros.org/en/melodic/api/trajectory_msgs/html/msg/JointTrajectory.html

    def putdown(self):
        return

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
    # The robot will proceed to the nearest object of specified color and will pick it up
    def image_callback(self, msg):
        print("in image callback!")

        # COLOR OBJECT PERCEPTION/DETECTION STUFF
        # converts the incoming ROS message to cv2 format, HSV (hue, saturation, value), and grayscale
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
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

        # TAG_ID PERCEPTION/DETECTION STUFF
        # search for tags from DICT_4x4_50 in a GRAYSCALE image
        # load DICT_4x4_50
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        # convert image to grayscale
        grayscale = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected_points = cv2.aruco.detectMarkers(grayscale, aruco_dict)
        print("ids: ", ids)

        # find index in ids array for given tag id
        n = 0
        tag_found = False
        if ids is not None:
            for tag in ids:
                if tag[0] == self.tag: # look for tag with given tag id
                    tag_found = True
                    break
                n += 1

        # take the averages of the corner coordinates of the tag to find the center of the tag
        # find the average of all the coordinates to find the center of the tag (ctag)
        # then use the center of the camera screen w/2 to make sure ctag  doesn't sway too much from it


        # when the camera scans the tags in the frame, then it will order the tags that it scans by assigning them indexes
        # you can find out the corresponding tag_ids for each tag by looking through index[0].
        sum_x = 0
        sum_y = 0
        if tag_found:
            coords = corners[n][0]
            sum_x = 0
            sum_y = 0
            for c in coords:
                sum_x = sum_x + c[0]
                sum_y = sum_y + c[1]

            avg_x = int(sum_x/4)
            avg_y = int(sum_y/4)

        else:
            print("tag not found")

        # THIS IS JUST TO TEST IF I CAN SUCCESSFULLY IDENTIFY THE TAG!
        if tag_found:
            print("avg_x coord: ", avg_x)
            print("avg_y coord: ", avg_y)


        # extract dimensions from camera
        h, w, d = image.shape

        # MOVEMENT STUFF (MOVING TOWARDS A COLOR OBJECT TO PICK IT UP...)

        if self.picking_up: #TODO -- ADD MOVEMENTS FOR WHEN WE NEED TO PICK UP OBJECT: ROTATE, MOVE TO OBJECT, ALIGN, AND PICK UP
            cx = 0
            cy = 0

            # using moments() function, determine the center of the selected color's pixels
            M = cv2.moments(mask)
            # if there are any select color's pixels found
            if M['m00'] > 0:
                # determine the center of the selected color's pixels in the image
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])

                # visualize a red circle in our debugging window to indicate
                # the center point of the orange pixels
                cv2.circle(image, (cx, cy), 20, (0,0,255), hsv) # THIS SAID -lgw (unsure what that means)

                # self.cmd_vel_pub.publish(self.twist)
                print("selected color object found! -- ", self.object_color)
                cv2.imshow("window", image)
                cv2.waitKey(3)

                # return [cx, cy] # RETURNS AN ARRAY OF X AND Y VALUE OF CENTER

        #                # print("in image_callback")
        #                image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        #                coords = identify_objects(image)
        #                x = coords[0]
        #                self.target_distance_from_center = x - (image.width / 2)

        # MOVING TOWARDS A TAG_ID TO DROP OFF A COLOR OBJECT
        else: #TODO -- ROTATE TOWARDS TAG, MOVE, ALIGN, DROP OBJECT
            # visualize a red circle in our debugging window to indicate
            # the center point of the orange pixels
            cv2.circle(image, (avg_x, avg_y), 20, (0,0,255), -1)

            # self.cmd_vel_pub.publish(self.twist)
            print("selected tag found! -- ", self.tag)
            cv2.imshow("window", image)
            cv2.waitKey(3)


if __name__ == '__main__':
        rospy.init_node('perception')
        Perception()
        rospy.spin()
