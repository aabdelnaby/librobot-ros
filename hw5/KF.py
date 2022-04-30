#! /usr/bin/env python

# import ros stuff
import rospy
from std_msgs.msg import Header 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry

import tf2_ros
import tf
import math
import numpy as np
import sys
import time

class KF:

    def __init__(self):

        rospy.init_node('kf_scan')

        # use 'mobile_base' for turtlebot2?
        self.robot_model = rospy.get_param('~robot_name', 'turtlebot3_waffle')
        
        print 'Robot model: ', self.robot_model

        self.rate = rospy.Rate(100)
        self.scan_received = 0
        
        # where to read laser scan data
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.clbk_laser)

        print ("Wait for Laser data...")
        while (self.scan_received == 0):
            self.rate.sleep()
            

        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.clbk_odom)

        self.kf_pub = rospy.Publisher('/kf_scan', LaserScan, queue_size = 5)

        self.x = 100
        self.y = 100
        self.yaw = 100

        

        print ("Wait for odometry data....")
        while ( (self.x == 100) or (self.y == 100) or (self.yaw == 100)):
            self.rate.sleep()

        print ("Odom recieved")

        print ("Odom recieved x: %d, y: %d, yaw: %d" %(self.x, self.y, self.yaw))
        print ("Make sure they are all zeros")
        
        # Making a variable of type LaserScan
        self.kf_scan = LaserScan()
        self.kf_scan.header = Header()
        self.kf_scan.header.seq = 0
        self.kf_scan.header.stamp = rospy.Time.now()
        self.kf_scan.header.frame_id = self.laser_scan.header.frame_id

        self.kf_scan.angle_min = self.laser_scan.angle_min
        self.kf_scan.angle_max = self.laser_scan.angle_max
        self.kf_scan.angle_increment = self.laser_scan.angle_increment
        
        self.kf_scan.scan_time = self.laser_scan.scan_time
        self.kf_scan.time_increment = self.laser_scan.time_increment
        self.kf_scan.range_min = self.laser_scan.range_min
        self.kf_scan.range_max = self.laser_scan.range_max

        # The initial values for our filter
        # Our P has dimension 72
        self.T = np.array(self.regions)
        self.Vk = np.diag([0.00001] * 360) # dimension 72 x 72
        self.P = self.Vk
        self.Wk = np.diag([0.1] * 360)
        
        
    def clbk_laser(self, msg):
        self.scan_received = 1
        self.laser_scan = msg
        self.regions = []

        for i in msg.ranges:
            if i == float('inf'):
                self.regions.append(1000)
            else:
                self.regions.append(i)

        



    def q_to_euler(self, msg):
        q = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(q)
        return yaw
    
    def clbk_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = self.q_to_euler(msg)

        self.pose = msg.pose.pose

    def publish_kf_scan(self):
        # This is where the filetring takes place:

        # The observation equation
        if len(self.regions) == 360:
            zk1 = np.array(self.regions)
            # The covariance matrix without observation
            P1_P = self.P + self.Vk

            # The gain
            G1 = np.matmul(P1_P, np.linalg.inv(P1_P + self.Wk))

            # The new covariance
            self.P = np.add(P1_P, -np.matmul(G1, P1_P))
            self.T = np.add(self.T, np.matmul(G1, (zk1 - self.T)))

        
            self.kf_scan.ranges = self.T
            self.kf_pub.publish(self.kf_scan)
        else:
            pass


            
            
if __name__ == '__main__':

    #try:
    scan_node = KF()
    while True:
        scan_node.publish_kf_scan()
        
        
            
    #except:
    msg = '[{}] ERROR: Node terminated'.format(rospy.get_name())
    rospy.loginfo(msg)
    error_type = sys.exc_info()[0]
    error_msg = sys.exc_info()[1]
    rospy.logerr(error_type)
    rospy.logerr(error_msg)

            
        

        
        

        
