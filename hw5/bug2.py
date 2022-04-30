#! /usr/bin/env python

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from collections import deque

import tf
import math
import numpy as np
import sys

class PID():
    '''Implement a generic PID controller for the motion control of a mobile robot.
       Given current error measure, it provides two separate methods for returning 
       the output values of linear and angular velocities.
       The initialization method sets all the parameters of the controller. 
       If some components of the P,I,D controller aren't used, the corresponding gains
       can be left set to their default value, which is 0.    
    '''

    def __init__(self,
                 _p_gain_distance = 0.0, _p_gain_angle = 0.0,
                 _i_gain_distance = 0.0, _i_gain_angle = 0.0 ,
                 _i_err_window_len = 30, _i_err_dt = 0.01,
                 _d_gain_distance = 0.0, _d_gain_angle = 0.0,
                 _v_lin_max = 0.2, _v_ang_max = 0.8):
        '''
           Input: all the gains and parameters that are necessary to set up a PID controller.
           _v_max and _angle_max are bounds to the velocities that can be give as output.
           The input arguments are used to define class variables with the same meaning.

           _p_gain_distance: the P gain for the error in the distance component
           _p_gain_angle: the P gain for the error in the angle component
           _i_gain_distance: the I gain for the error in the distance component
           _i_gain_angle: the I gain for the error in the angle component
           _i_err_window_len: the length, as number of error measures, of the time window used
                              to compute the integral for the I part (both for distance and angle)
           _i_err_dt: this is dt, the numeric differential in the numeric computation of the
                       integral ( i_err = \int_t0^t1 err(t)dt )
           _d_gain_distance: the D gain for the error in the distance component
           _d_gain_angle: the D gain for the error in the angle component
        '''
        
        self.p_gain_distance =  _p_gain_distance
        self.p_gain_angle = _p_gain_angle

        self.i_gain_distance = _i_gain_distance
        self.i_gain_angle = _i_gain_angle

        self.i_err_window_len = _i_err_window_len 
        self.i_err_dt = _i_err_dt

        # Create two circular buffers to hold the error data for computing integral errors
        # separately for distance and angle. Values are initialized to zero.
        self.i_err_window_data_distance = deque( [0]*self.i_err_window_len,
                                                 maxlen=self.i_err_window_len)

        self.i_err_window_data_angle = deque( [0]*self.i_err_window_len,
                                              maxlen=self.i_err_window_len)
                
        self.d_gain_distance = _d_gain_distance
        self.d_gain_angle = _d_gain_angle
                
        self.v_max = _v_lin_max
        self.omega_max = _v_ang_max

        
    def update_integral_error_distance(self, err):
        '''The error err is used to update the integral error on distance'''
        self.i_err_window_data_distance.append(err)
        self.i_err_distance = np.sum(self.i_err_window_data_distance) * self.i_err_dt


    def update_integral_error_angle(self, err):
        '''The error err is used to update the integral error on angle'''
        self.i_err_window_data_angle.append(err)
        self.i_err_angle_integral = np.sum(self.i_err_window_data_angle) * self.i_err_dt

        
    def reset_integral_errors(self):
        '''Reset the integral errors, both for distance and angle.
           This is used when a waypoint is reached and a new one shall start.'''
        self.i_err_window_data_distance = deque( [0]*self.i_err_window_len,
                                                 maxlen=self.i_err_window_len)

        self.i_err_window_data_angle = deque( [0]*self.i_err_window_len,
                                              maxlen=self.i_err_window_len)
        
        
    def get_linear_velocity_P(self, err_d):
        '''Get error in distance and return P component of linear velocity'''
        # Reacting to the error porportionally
        vel =   float(err_d) * float(self.p_gain_distance)
        return vel

    
    def get_linear_velocity_I(self, err_d):
        t0 = self.i_err_window_data_distance.popleft()
        self.i_err_window_data_distance.append(err_d)

        vel = float(self.i_err_dt*float(err_d - t0)*float(self.i_gain_distance))
        return vel


    def get_linear_velocity_D(self, err_d):
        '''Get error in distance and return D component of linear velocity'''
        t_last = self.i_err_window_data_distance.pop()
        self.i_err_window_data_distance.append(t_last)

        de = err_d - t_last
        D = float(de * self.i_err_dt) * self.d_gain_distance
        return D

    
    def get_angular_velocity_P(self, err_angle):
        '''Get error in angle and return P component of angular velocity'''
        vel =  float(err_angle) * float(self.p_gain_angle)
        return vel


    def get_angular_velocity_I(self, err_angle):
        t0 = self.i_err_window_data_angle.popleft()
        self.i_err_window_data_angle.append(err_angle)

        vel = float(self.i_err_dt * float(err_angle - t0)*float(self.i_gain_angle))
        return vel

    def get_angular_velocity_D(self, err_angle):
        '''Get error in distance and return D component of linear velocity'''
        t_last = self.i_err_window_data_angle.pop()
        self.i_err_window_data_angle.append(t_last)

        de = err_angle - t_last
        D = float(de * self.i_err_dt) * self.d_gain_angle 
        return D

    # The following three empty methods are included for completeness, to deal with a car-like
    # vehicle with two parallel front steering wheels.
    # You don't have to complete them (for now).
    
    def get_steering_angle_P(self, err_angle):
        '''Get error in angle and return P component of steering angle'''
        pass
        return gamma_p

    def get_steering_angle_I(self, err_angle):
        '''Get error in angle and return I component of steering angle'''
        pass
        return gamma_i

    def get_steering_angle_D(self, err_angle):
        '''Get error in angle and return D component of steering angle'''
        pass
        return gamma_d

    


# to see scanner's point in rviz when using gazebo
# rosrun robot_state_publisher robot_state_publisher 
# rosrun rviz rviz
# set dimension, set style -> points

class Bug1():

    def __init__(self, xy_tuple):

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

        # get the pose from the odom
        self.odom_sub = rospy.Subscriber("/odom", Odometry,
                                         self.callback_odometry,queue_size = 1)

        self.rate = rospy.Rate(100)

        # make the twist type
        self.t = Twist()

        self.t.linear.x = 0
        self.t.linear.y = 0
        self.t.linear.z = 0

        self.t.angular.x = 0
        self.t.angular.y = 0
        self.t.angular.z = 0

        # get the velocities that are to be used
        self.v = rospy.get_param("~lin_velocity", 0.18)
        self.omega = rospy.get_param("~ang_velocity", 0.5)

        self.x = 100
        self.y = 100
        self.yaw = 100

        print ("Wait for odometry data....")
        while ( (self.x == 100) or (self.y == 100) or (self.yaw == 100)):
            self.rate.sleep()

        print ("Odom recieved")

        print ("Odom recieved x: %d, y: %d, yaw: %d" %(self.x, self.y, self.yaw))
        print ("Make sure they are all zeros")
        
        # What function to call when ctrl + c is issued
        rospy.on_shutdown(self.shutdown)

        

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
        self.d_max = 3.5
        self.open_ray = 10

        # the distance to keep
        self.target = xy_tuple

        
    def clbk_laser(self, msg):

        self.regions = {'front':  min( min(min(msg.ranges[0:30]), min(msg.ranges[330:359])), 10),
                        'fleft':   min(min(msg.ranges[31:60]), 10),
                        'left':   min(min(msg.ranges[61:120]), 10),
                        'fright': min(min(msg.ranges[270:329]), 10),
                        'right':  min(min(msg.ranges[210:270]), 10) }

        #print self.regions


    def callback_odometry(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = self.q_to_euler(msg)

    def q_to_euler(self, msg):
        q = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(q)
        return yaw

    # given a xy , calculates how much the robot needs to turn
    def get_angle(self, target):
        x = target[0]
        y = target[1]

        current_angle = self.yaw
        point_angle = np.arctan2(y - self.y, x - self.x)

        angle_diff = self.get_a_distance(point_angle, current_angle)


        return angle_diff

    # get the euclidean distance between two points
    def get_e_distance(self, i, j):
        return np.sqrt(np.square(i[0] - j[0]) + np.square(i[1] - j[1]))

    # function for moving in straight line
    def rotate_angle_odo(self, angle):
        self.start_yaw = (self.yaw)

        if (angle < 0):
            self.t.angular.z = - self.omega
            angle = -angle
        else:
            self.t.angular.z = self.omega

        while abs(self.get_a_distance(self.start_yaw, self.yaw)) < (angle*0.96):
            
            self.vel_pub.publish(self.t)
            self.rate.sleep()

        print ("Done rotation: %f, Target rotation: %f"
               % (abs(self.get_a_distance(self.start_yaw, self.yaw)), angle))



        

    # get the angular distance between two points
    def get_a_distance(self, a1, a2):
        x1 = np.cos(a1)
        y1 = np.sin(a1)

        x2 = np.cos(a2)
        y2 = np.sin(a2)

        diff = np.arctan2(y1*x2 - y2*x1,  x1*x2 + y1*y2)

        return diff

    def move_to_target(self):
        target = self.target
        # Calculates the angle that it first needs to turn to
        correct_direction = self.get_angle(target)
        print "%f" % correct_direction
        self.rotate_angle_odo(correct_direction)


        self.t.angular.z = 0
        self.vel_pub.publish(self.t)


        start_xy = (self.x, self.y)
        distance = self.get_e_distance(start_xy, target)
        self.t.linear.x = self.v
        self.t.angular.z = 0

        reached = True

        while (self.get_e_distance(start_xy, (self.x, self.y)) < distance*0.95):
            if self.regions["front"] < 0.5:
                reached = False
                break
            self.vel_pub.publish(self.t)
            self.rate.sleep()

            
        self.t.linear.x = 0
        self.vel_pub.publish(self.t)

        print ("Finished reaching point x: %f, y:%f" %(self.x, self.y))

        # Wall following part starts here, we are currently 1 meter away from wall
        # We want to get it to the right of us
        self.t.angular.z = self.omega
        while (self.regions["right"] >= 0.5):
            self.vel_pub.publish(self.t)
            self.rate.sleep()

        count = 0

        # Make use of a simple PID controller
        while (True):
            # error 
            error = self.regions['right'] - 0.5

            omega = -min(error*0.2, 0.5) 
      
            self.t.linear.x = 0.25
            self.t.angular.z = omega

            self.vel_pub.publish(self.t)
            count+= 1
            '''
            if (error >= 5):
                self.rotate_angle_odo(np.radians(90))
                while (error >= 5):
                    error = self.regions['front'] 
                    self.t.angular.z = 0
                    self.t.linear.x = 0.25
                # its now at front '''
                

            if (count % 500 == 0):
                print ("Error is: %.15f, velocity is: %.15f" %(error, omega))

            
            



        

        

    

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
        test = Bug1((2, 2)) # takes x and y coordinate tuple
        test.move_to_target()
        
        
            
    except:
        msg = '[{}] ERROR: Node terminated'.format(rospy.get_name())
        rospy.loginfo(msg)
        error_type = sys.exc_info()[0]
        error_msg = sys.exc_info()[1]
        rospy.logerr(error_type)
        rospy.logerr(error_msg)
