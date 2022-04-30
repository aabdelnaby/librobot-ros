#! /usr/bin/env python

# import ros stuff
import rospy
from std_msgs.msg import Header
from std_msgs.msg import Int64MultiArray
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData

from geometry_msgs.msg import TransformStamped, PoseStamped, Point
from geometry_msgs.msg import Quaternion, Vector3
from tf2_geometry_msgs import PointStamped
from tf2_geometry_msgs import do_transform_point, do_transform_pose

import tf2_ros
import tf
import tf_conversions
import math
import numpy as np
import sys
import time

class Map:

    def __init__(self, target):
        self.target = target

        rospy.init_node('local_map')

        # use 'mobile_base' for turtlebot2?
        self.robot_model = rospy.get_param('~robot_name', 'turtlebot3_waffle')
        
        print 'Robot model: ', self.robot_model

        self.pose_pub = rospy.Publisher('/to_target', PointStamped, queue_size = 5)

        self.map_pub = rospy.Publisher('/local_map', OccupancyGrid, queue_size = 5)

        # Set the proper rate
        rate = rospy.get_param("/rate", 200)
        self.rate = rospy.Rate(rate)

        self.regions = []

        # where to read laser scan data
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.clbk_laser)

        
        print ("Wait for Laser scan data....")
        while ( (self.regions == [])):
            self.rate.sleep()

        print ("Scan recieved")

        # publish twists to cmd_vel
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 5)


        # get the velocities that are to be used
        self.v = rospy.get_param("~lin_velocity", 0.3)
        self.omega = rospy.get_param("~ang_velocity", 0.5)

        # make the twist type
        self.t = Twist()

        self.t.linear.x = 0.0
        self.t.linear.y = 0
        self.t.linear.z = 0

        self.t.angular.x = 0
        self.t.angular.y = 0
        self.t.angular.z = 0


        # define a tf2 transform buffer and pass it to a listener
        self.tf2_buffer = tf2_ros.Buffer(rospy.Duration(5))
        tf2_ros.TransformListener(self.tf2_buffer)

        # Make the object type that is to be sent
        self.map = OccupancyGrid()

        # Fill in the metadata and header here:
        header = Header()
        header.seq = 5555
        header.stamp = rospy.Time.now()
        header.frame_id = 'local_map'

        meta = MapMetaData()
        meta.map_load_time = rospy.Time.now()
        meta.resolution = 0.125 
        meta.width = 64
        meta.height = 64


        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.clbk_odom)

        self.x = 100
        self.y = 100
        self.yaw = 100

        print ("Wait for odometry data....")
        while ( (self.x == 100) or (self.y == 100) or (self.yaw == 100)):
            self.rate.sleep()

        print ("Odom recieved")

        print ("Odom recieved x: %d, y: %d, yaw: %d" %(self.x, self.y, self.yaw))
        print ("Make sure they are all zeros")


        p = Pose()
        p.position.x = 0
        p.position.y = 0
        p.position.z = 0
        meta.origin = p
        self.totalWidth = meta.width

        self.map.header = header
        self.map.info = meta
        self.map.data = [0] * (meta.width * meta.width)

        self.width = meta.width



    
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

    # given a xy , calculates how much the robot needs to turn
    def get_angle(self, step):
        x = step[0]
        y = step[1]

        current_angle = self.yaw
        point_angle = np.arctan2(y - self.y, x - self.x)

        angle_diff = self.get_a_distance(point_angle, current_angle)


        return angle_diff
        

    
    def clbk_laser(self, msg):
        self.regions = msg.ranges

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

    def pose_to_target(self):
        # first step consists in getting relative distance and angle of (x,y) part of pose
        local_in_reference = self.tf2_buffer.lookup_transform('base_footprint', 'odom',
                                                              rospy.Time(0), rospy.Duration(5))

        point_stamped = self.make_point(self.target[0], self.target[1], 'target')

        relative_position_of_point = do_transform_point(point_stamped, local_in_reference)
        
        self.pose_pub.publish(relative_position_of_point)
        
        

    def get_index(self, x, y):
        return int(y*(self.totalWidth) + x)
        

    # The information is published from this function
    def publish_info(self):
        self.map.data = [0] * (self.width * self.width)
        self.vector = [(0, 3.5)] * 360
        for i, value in enumerate(self.regions):
            # for out of range case
            if value==float('inf'):
                self.vector[i] = (0, 3.5)
                pass
            else:
               
                x = int(value*np.cos(np.radians(i))*8 + 32)
                y = int(value*np.sin(np.radians(i))*8 + 32)

                ind = self.get_index(x, y)
                #print(ind)

                P_s_h = float(float(3.5 - value)/float(3.5))

                if ind < 0 or ind >= (self.width * self.width):
                    pass
                else:
                    self.map.data[ind] = float(P_s_h * 100)

                self.vector[i] = (P_s_h, value)



                
                
        br = tf2_ros.TransformBroadcaster()
        t = TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "local_map"
        t.child_frame_id = "base_link"
        t.transform.translation.x = 4
        t.transform.translation.y = 4
        t.transform.translation.z = 0.0

        q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        

        br.sendTransform(t)

        
        self.map_pub.publish(self.map)


    def make_histogram(self):
        # To update the value in self.vector
        self.publish_info()
        distances = self.vector
        histogram = [0] * 36

        for i in range(36):
            current_slice = distances[i*10: (i+1)*10]

            for c in current_slice:
                histogram[i] += ((c[0])**2) * (1 - float(c[1]/3.5))

        return histogram

    def get_valley(self):
        polar_histogram = self.make_histogram()
        threshold = 3

        valleys = []
        # current valley index i.e. one we're inserting currently in
        current = -1
        # true if already inserting in a valley and false otherwise
        insert = False

        for sector, h in enumerate(polar_histogram):
            if not insert and h < threshold:
                insert = True
                valleys.append([(h, sector)])
                current += 1

            if insert and h < threshold:
                (valleys[current]).append((h, sector))

            if insert and h >= threshold:
                insert = False

        target =  self.get_angle(self.target)

        valleys_range = []

        for i in valleys:
            (first_value, first_range) = i[0]
            (last_value, last_range) = i[-1]
            
            valleys_range.append((first_range, last_range))

        best_valley = -1
        best_diff = 360

        for i, ranges in enumerate(valleys_range):
            (valley_start, valley_end) = (ranges[0], ranges[1])
            
            if target in range(valley_start, valley_end + 10):
                best_diff = 0
                best_valley = i
                break
            else:
                min_diff = min(abs(target - valley_start), abs(target - valley_end))

                if min_diff < best_diff:
                    best_diff = min_diff
                    best_valley = i
        # return the best valley (a list of (hs,sector) tuples)
        return valleys[best_valley]
    
    def move(self):
        current_pose = (self.x, self.y)
        target = self.target
        target_angle = self.get_angle(target)
        while (self.get_e_distance(target, current_pose) >= 0.5) and not rospy.is_shutdown():
            # Get best valley
            valley = self.get_valley()

            (h1, valley_start) = valley[0]
            (h2, valley_end) = valley[-1]

            least_diff = 360
            best_sector = -1

            # Case 1 and 2
            if len(valley) >= 5:
                for i, value in enumerate(valley):
                    (h, sector) = value
                    if abs(target_angle - sector) < least_diff:
                        best_sector = i
                        least_diff = abs(target_angle - sector) < least_diff
                    else:
                        pass
            # Case 3
            else:
                best_sector = len(valley)/2

            # For case 2
            if  least_diff > 10 and least_diff != 360:
                if best_sector > len(valley)/2:
                    best_sector = (valley[best_sector] + valley[-1])/2
                else:
                    best_sector = (valley[best_sector] + valley[0])/2

            
            # Publish linear velocity porportional to distance from target
            (h, angle) = valley[best_sector]
            
            if h == 0:
                self.t.linear.x = 0.3
            else:
                mins = []
                for i in self.regions:
                    if i!= 0:
                        mins.append(i)
                self.t.linear.x = (0.3) * (float(min(mins)/3.5))
                
            self.t.angular.z = min(0.3, 0.2*float(self.get_a_distance(angle, self.yaw))/2*np.pi)
            print(self.get_a_distance(angle, self.yaw))

            self.t.linear.x = min(self.t.linear.x, 0.7 * self.get_e_distance(target, current_pose))
            self.t.angular.z = min(self.t.angular.z, 0.9 * self.get_e_distance(target, current_pose))

            print(self.t.linear.x)
            
            self.vel_pub.publish(self.t)
            self.rate.sleep()
            
            while (abs(self.get_a_distance(angle, self.yaw)) >= 0.2):
                if rospy.is_shutdown():
                    self.t.linear.x = 0
                    self.t.angular.z = 0
                    pass
                self.vel_pub.publish(self.t)
                self.rate.sleep()

            self.t.angular.z = 0
            self.t.linear.x = 0
            self.vel_pub.publish(self.t)

            if rospy.is_shutdown():
                pass

            current_pose = (self.x, self.y)
            target_angle = self.get_angle(target)

            self.rate.sleep()
        
        self.t.linear.x = 0
        self.t.angular.z = 0
        self.vel_pub.publish(self.t)
            
   
if __name__ == '__main__':

    #try:
    grid = Map((3, 2)) # takes x and y coordinate tuple
    while not rospy.is_shutdown():
        grid.publish_info()
        grid.pose_to_target()
        #grid.move()

    grid.t.linear.x = 0
    grid.t.angular.z = 0
    grid.vel_pub.publish(grid.t)

    self.rate.sleep()
        
        
            
    #except:
    msg = '[{}] ERROR: Node terminated'.format(rospy.get_name())
    rospy.loginfo(msg)
    error_type = sys.exc_info()[0]
    error_msg = sys.exc_info()[1]
    rospy.logerr(error_type)
    rospy.logerr(error_msg)

            
        

        
        

        
