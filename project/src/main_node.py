#! /usr/bin/env python

# import ros stuff
import rospy
from std_msgs.msg import Header, String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry

from geometry_msgs.msg import TransformStamped, PoseStamped, Point
from geometry_msgs.msg import Quaternion, Vector3
from tf2_geometry_msgs import PointStamped, do_transform_point, do_transform_pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker

import actionlib
import tf2_ros
import tf
import tf_conversions
import math
import numpy as np
import sys
import time

class LibraryHelp:

    # landmark_list here has (x, y)s used for localization, default_list has
    # the default positions for the robot
    def __init__(self, landmark_list, books_list, default_list, angular_speed):
        self.books_list = books_list
        self.default_list = default_list

        self.angular_speed = angular_speed
        
        self.seen_book = []

        rospy.init_node('arm_status')

        # make the twist type
        self.t = Twist()
        (self.t.linear.x, self.t.linear.y, self.t.linear.z) = (0, 0, 0)
        (self.t.angular.x, self.t.angular.y, self.t.angular.z) = (0, 0, 0)

        (self.odo_x, self.odo_y, self.odo_yaw) = (100, 100, 100)

        # define a tf2 transform buffer and pass it to a listener
        self.tf2_buffer = tf2_ros.Buffer(rospy.Duration(5))
        tf2_ros.TransformListener(self.tf2_buffer)

        # Set the proper rate
        rate = rospy.get_param("/rate", 200)
        self.rate = rospy.Rate(rate)

    
	self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        self.arm_pub = rospy.Publisher('/arm_status', String, queue_size=5)


        # get the velocities that are to be used
        self.v = rospy.get_param("~lin_velocity", 0.3)
        self.omega = rospy.get_param("~ang_velocity", 0.5)


        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.clbk_odom)

        print ("Wait for odometry data....")
        while ( (self.odo_x == 100) or (self.odo_y == 100) or (self.odo_yaw == 100)):
            print("Still not here mate ")
            self.rate.sleep()

        print ("Odom recieved")

        print ("Odom recieved x: %d, y: %d, yaw: %d" %(self.odo_x, self.odo_y, self.odo_yaw))
        print ("Make sure they are all zeros")

        # subscribe to the topic published by the ar_track_alvar node, that observes ar tags
        self.ar_sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers,
                                       self.clbk_marker, queue_size=5)

        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        print ('Waiting for action server ...')
        self.client.wait_for_server()
        print("Connected to action server")


    def go_to_sub(self, x, y): 
        # input values for the x,y of target position
        x = rospy.get_param('~x', x)
        y = rospy.get_param('~y', y)
                        
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
    
        # a different orientation can be specified of course!
        goal.target_pose.pose.orientation.w = 1.0

        print("Going to x:%f,y:%f"%(x, y))

    
        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()
        

    def clbk_odom(self, msg):
        self.odo_x = msg.pose.pose.position.x
        self.odo_y = msg.pose.pose.position.y
        self.odo_yaw = self.q_to_euler(msg)

        self.pose = msg.pose.pose

    def clbk_marker(self, msg):
        for m in msg.markers:
            if m.id < len(self.landmark_list):
                self.seen_books.append(m)

    # get the euclidean distance between two points
    def get_e_distance(self, i, j):
        return np.sqrt(np.square(i[0] - j[0]) + np.square(i[1] - j[1]))


    # get the angular distance between two points
    def get_a_distance(self, a1, a2):
        x1 = np.cos(a1)
        y1 = np.sin(a1)

        x2 = np.cos(a2)
        y2 = np.sin(a2)

        diff = np.arctan2(y1*x2 - y2*x1,  x1*x2 + y1*y2)

        return diff


    def q_to_euler(self, msg):
        q = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(q)
        return yaw

    def make_point(self, x, y, frame_id):
        # create a PointStamped object to encapsulate the point to reach since
        # do_transform_point requires such an object
        point_stamped = PointStamped()
        point_stamped.point = Point(x, y, 0)
        point_stamped.header.frame_id = frame_id
        point_stamped.header.stamp = rospy.Time.now()
        point_stamped.header.seq = 0
        #print '\n ***> Point stamped: \n', point_stamped

        return point_stamped

    def default_behavior(self):
        # Goes throught the default positions, as long as there are no books
        for i in self.default_list:
            # Make sure to add a orientation if you are using a PID
            self.go_to_sub(i[0], i[1])
            self.current_yaw = (self.odo_yaw)
            self.t.angular.z = 0.4

            while(abs(self.get_a_distance(self.odo_yaw, self.current_yaw)) < 3):
                print(abs(self.get_a_distance(self.odo_yaw, self.current_yaw)))
                self.t.angular.z = 0.4
                self.vel_pub.publish(self.t)
                self.rate.sleep()

            self.t.angular.z = 0
            self.vel_pub.publish(self.t)
        
        for i in self.seen_book:
            # Go to the book that you just saw
            book_id = m.id
            (x, y) = (m.pose.pose.position.x, m.pose.pose.position.y)
            (shelf_x, shelf_y) = self.books_list[book_id]
            if (self.get_e_distance((shelf_x, shelf_y), (x, y)) >= 1):
                self.go_to_sub((x, y))
                self.arm_pub.publish("Holding book")
                print("Book %d grabbed at (%.15f, %.15f)" 
                    % (book_id, self.odo_x, self.odo_y))
                self.go_to_sub(shelf_pos[0], shelf_pos[1])
                self.arm_pub.publish("Placed book")
            
   
if __name__ == '__main__':

    # landmark should be of format (x, y) where its index is its id
    # books should be a dict with id:(x, y) format, where id is the shelf
    # defaults are just (x, y) for default positions of robot
    shelves = [(3.3, 1.2), (4.45, -0.37), (5.45, -1.9)]
    books = {6:(0.93, -1.62) , 8:(4.45, -0.37), 4:(5.45, -1.9)}
    defaults = [(1.5, -0.5), (3, -3), (4, -4)]
    rotation_speed = 0.3
    node = LibraryHelp(shelves, books, 
                       defaults, rotation_speed)
    while not rospy.is_shutdown():
        node.default_behavior()
  

    node.t.linear.x = 0
    node.t.angular.z = 0
    node.vel_pub.publish(node.t)

    node.rate.sleep()
        
        
            
    #except:
    msg = '[{}] ERROR: Node terminated'.format(rospy.get_name())
    rospy.loginfo(msg)
    error_type = sys.exc_info()[0]
    error_msg = sys.exc_info()[1]
    rospy.logerr(error_type)
    rospy.logerr(error_msg)
