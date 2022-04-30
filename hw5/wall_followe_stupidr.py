#! /usr/bin/env python

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations

import math
import numpy as np
import sys

# to see scanner's point in rviz when using gazebo
# rosrun robot_state_publisher robot_state_publisher 
# rosrun rviz rviz
# set dimension, set style -> points

class WallFollow():

    def __init__(self, distance_from_wall=1.5):

        rospy.init_node('wall_follower')
    
        # use 'mobile_base' for turtlebot2?
        self.robot_model = rospy.get_param('~robot_name', 'turtlebot3_waffle')
        
        print 'Robot model: ', self.robot_model

        if 'turtlebot2' in self.robot_model:
            self.vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=5)
        else:
	    self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        # where to read laser scan data
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.clbk_laser)
        
        # What function to call when ctrl + c is issued
        rospy.on_shutdown(self.shutdown)

        self.rate = rospy.Rate(20)

        self.regions = {'right': 0,
                        'fright': 0,
                        'front': 0,
                        'fleft': 0,
                        'left': 0 }
        self.state = 0

        self.state_dict = { 0: 'find the wall',
                             1: 'turn left',
                             2: 'follow the wall'}

        # laser scan parameters
        self.delta_angle = 0.0175
        self.n_values = 360
        self.d_min = 0.12
        self.d_max = 1.5
        self.open_ray = 10

        # the distance to keep
        self.d = distance_from_wall

        
    def clbk_laser(self, msg):

        self.regions = {'front':  min( min(min(msg.ranges[0:30]), min(msg.ranges[330:359])), 10),
                        'fleft':   min(min(msg.ranges[31:60]), 10),
                        'left':   min(min(msg.ranges[61:120]), 10),
                        'fright': min(min(msg.ranges[270:329]), 10),
                        'right':  min(min(msg.ranges[210:270]), 10) }

        print self.regions

        self.take_action()
        

    def change_state(self, state):
        if state is not self.state:
            print 'Wall follower - [%s] - %s' % (state, self.state_dict[state])
            self.state = state
        
    def take_action(self):
        msg = Twist()
        linear_x = 0
        angular_z = 0
    
        state_description = ''
    
        if self.regions['front'] > self.d and self.regions['fleft'] > self.d and self.regions['fright'] > self.d:
            state_description = 'case 1 - nothing'
            self.change_state(0)
        elif self.regions['front'] < self.d and self.regions['fleft'] > self.d and self.regions['fright'] > self.d:
            state_description = 'case 2 - front'
            self.change_state(1)
        elif self.regions['front'] > self.d and self.regions['fleft'] > self.d and self.regions['fright'] < self.d:
            state_description = 'case 3 - fright'
            self.change_state(2)
        elif self.regions['front'] > self.d and self.regions['fleft'] < self.d and self.regions['fright'] > self.d:
            state_description = 'case 4 - fleft'
            self.change_state(0)
        elif self.regions['front'] < self.d and self.regions['fleft'] > self.d and self.regions['fright'] < self.d:
            state_description = 'case 5 - front and fright'
            self.change_state(1)
        elif self.regions['front'] < self.d and self.regions['fleft'] < self.d and self.regions['fright'] > self.d:
            state_description = 'case 6 - front and fleft'
            self.change_state(1)
        elif self.regions['front'] < self.d and self.regions['fleft'] < self.d and self.regions['fright'] < self.d:
            state_description = 'case 7 - front and fleft and fright'
            self.change_state(1)
        elif self.regions['front'] > self.d and self.regions['fleft'] < self.d and self.regions['fright'] < self.d:
            state_description = 'case 8 - fleft and fright'
            self.change_state(0)
        else:
            state_description = 'unknown case'
        rospy.loginfo(self.regions)
        

    def find_wall(self):
        msg = Twist()
        msg.linear.x = 0.2
        msg.angular.z = -0.1
        return msg

    def turn_left(self):
        msg = Twist()
        msg.angular.z = 0.2
        return msg

    def follow_the_wall(self):
        msg = Twist()
        msg.linear.x = 0.5
        return msg

    def shutdown(self):
        '''Ensure that on shutdown the robot stops moving.'''
        rospy.loginfo("**** Stopping TurtleBot! ****")
        v = Twist()
        v.linear.x = 0.0
        v.angular.z = 0.0
        self.vel_pub.publish(v)
        rospy.sleep(1)
    
if __name__ == '__main__':

    try:
        wall_follower = WallFollow(1.0)
        
        while not rospy.is_shutdown():
            v = Twist()
            if wall_follower.state == 0:
                v = wall_follower.find_wall()
            elif wall_follower.state == 1:
                v = wall_follower.turn_left()
            elif wall_follower.state == 2:
                v = wall_follower.follow_the_wall()
            else:
                rospy.logerr('Unknown state!')
        
            wall_follower.vel_pub.publish(v)
        
            wall_follower.rate.sleep()
            
    except:
        msg = '[{}] ERROR: Node terminated'.format(rospy.get_name())
        rospy.loginfo(msg)
        error_type = sys.exc_info()[0]
        error_msg = sys.exc_info()[1]
        rospy.logerr(error_type)
        rospy.logerr(error_msg)
        
