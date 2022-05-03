#! /usr/bin/env python

# Node that implements an EKF for robot pose estimation
# based on the availability of a landmark map (passed as input file).
# Landmarks are AR visual tags in the Alvar format.

# roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
# add landmark i to the scene with: Insert -> MarkerData_i

# Launch the ar_track_alvar with a custom launch file specifying
# markes sizes and expected errors
# roslaunch my_ar_track_alvar ar_track_alvar_gazebo.launch

# roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch
# note: this is for visualization AND for publishing a few transforms
# that are necessary for ar_track_alvar.
# With real robots the turtlebot_bringup shall do the job.

# roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch 
# this is for manually moving the robot


import rospy
import numpy as np 
import tf
from geometry_msgs.msg import Twist, Point, PoseStamped, Quaternion
from std_msgs.msg       import Float64

from nav_msgs.msg import Path

from std_msgs.msg import Empty

from nav_msgs.msg import Odometry 

from tf.transformations import euler_from_quaternion, quaternion_from_euler

from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker

import csv

from copy import deepcopy


class EKF_Localization_LandmarkMap():
    '''This class implements an EKF for robot localization using a landmark map 
       where the landmarks are visual fiducials in the form of AR Alvar tags'''
    
    def __init__(self, initial_pose = '/odom', #\xi_0, either np.array([x,y,t])
                 initial_pose_cov = np.identity(3),  # P_0
                 motion_covariance = 0.2*np.identity(2),  # V, 3x3
                 observation_covariance = 0.1*np.identity(2), # W, 2x2
                 landmark_map_file = 'landmark_map.csv' ):
        '''Input:
             initial_pose: corresponds to xi_0 in the EKF equations. If value is '/odom', 
                           it is assigned by reading the current initial pose from /odom. 
                           In alternative, it can be explicitly passed as a (1,3) np.array([x,y,theta])
             initial_pose_cov: -> P_0, it's a numpy array of shape (3,3)
             motion_covariance: -> V, quantifies errors in (v,omega) controls, numpy array of shape (3,3) 
             observation_covariance: -> W, quantifies sensing errors in range and bearing, array of shape (2,2) 
             landmark_map_file: CSV file with rows of type landmark_identity, \lambda_x, \lambda_y
        '''

        # class variable for pose covariance
        self.pose_covariance = initial_pose_cov

        # class variable for motion covariance
        self.motion_covariance = motion_covariance

        # class variable for observation covariance
        self.observation_covariance = observation_covariance

        # read landmark map from .csv file and store it in a dictionary
        self.landmark_map = {}
        with open(landmark_map_file) as f:
            f_csv = csv.reader(f)
            next(f_csv)
            for r in f_csv:
                self.landmark_map[int(r[0])] = (float(r[1]), float(r[2]))                

        print 'Landmark_Map:', self.landmark_map

        # Frequency for state update, can pass this via command line with _rate:=
        self.frequency_updates = rospy.get_param('~rate', 50)
        self.delta_t = 1. / float(self.frequency_updates)
        
        self.Rate = rospy.Rate(self.frequency_updates)
        print('Rate for updates: {:.2f}/sec'.format(self.frequency_updates))

        # subscribe to odometry for getting the initial pose and for
        # drawing a comparison of pose estimates
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.callback_odometry)

      
        
        # wait for an input from the odometry topic
        self.odo_x = self.odo_y = self.odo_yaw = None
        print 'Wait for first data on odometry topic ...',
        while (self.odo_x == self.odo_y == self.odo_yaw == None):
            self.Rate.sleep()
        print(' done!')

        # pose shall be a (3,1) column vector but for simplicity of notation
        # when doing updates, it has been declared here as a (1,3) row vector 
        self.pose = np.array( [0.0, 0.0, 0.0] ) # x, y, theta

        # initialize the pose using odom + normal noise, if pose is not passed explicitly        
        if initial_pose == '/odom':
            self.pose[0] = self.odo_x 
            self.pose[1] = self.odo_y 
            self.pose[2] = self.odo_yaw
        else: # initialize based on given input value
            self.pose = initial_pose

        # initialize velocity variables related to current and previous step
        self.v_now = self.omega_now = 0.0 

        self.v_before = self.v_now
        self.omega_before = self.omega_now

        # subscribe to /cmd_vel for reading the input velocities when robot moves
        self.cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, self.callback_cmd_vel)

        # initializations for computing the mean squared error of pose estimation
        # compared to pose from /odom, and to compute initial cov determinant
        self.cnt = 1
        self.MSE = 0.0
        self.pose_covariance_det = np.round(np.sqrt(np.linalg.det(self.pose_covariance)), 3)
        
        # subscribe to the topic published by the ar_track_alvar node, that observes ar tags
        self.ar_sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers,
                                       self.observation_correction, queue_size=1)

        # topics for publishing pose estimates from ekf and from odometry as Path messages
        # that can be displayed in rviz using Path plugin
        self.ekf_path_pub = rospy.Publisher('/ekf_path', Path, queue_size=5)
        self.odom_path_pub = rospy.Publisher('/odom_path', Path, queue_size=5)
        self.ekf_pose = rospy.Publisher('/ekf_pose', Pose, queue_size=5)
        self.path_seq = 0


        
    ##################### Callback methods for reading velocities and odometry ####################
    
    def callback_cmd_vel(self, msg):
        '''Read from /cmd_vel the Twist values issued (by some other node) to move robot.
           These v,omega are used to compute \Delta S_k and \Delta \theta_k in prediction_update()'''

        self.v_now = msg.linear.x
        self.omega_now = msg.angular.z
        
        
        #print "Velocities: (%5.2f, %5.2f)" % (self.v_now, self.omega_now)

        
    def callback_odometry(self, msg):
        '''Read from /odom the pose estimate from the odometry ekf, which uses
           wheel encoders and IMU. 
           The only purpose of reading odom's pose is for making a comparison of estimates.'''

        self.odo_x = msg.pose.pose.position.x
        self.odo_y = msg.pose.pose.position.y
        self.last_odo_msg = msg

        quaternion = msg.pose.pose.orientation
        (_r, _p, self.odo_yaw)   = euler_from_quaternion((quaternion.x, quaternion.y, 
                                                          quaternion.z, quaternion.w))

        #print "\nPose: (%5.2f, %5.2f), Yaw (deg): %5.2f" % (self.odo_x, self.odo_y,
        #                                                    np.degrees(self.odo_yaw))
        #print "Velocities: (%5.2f, %5.2f)" % (msg.twist.twist.linear.x, msg.twist.twist.angular.z)


        
    ##################### EKF: prediction and correction updating methods ####################
    
    def prediction_update(self):

        # variations in traveled distance and orientation angle derived from measured v and omega
        self.delta_s = float(self.delta_t * (float(self.v_now + self.v_before)/2.))
        self.delta_theta = float(self.delta_t * (float(self.omega_now + self.omega_before)/2.))

        # store current velocity values as the initial values for the next updating step
        self.v_before, self.omega_before = self.v_now, self.omega_now

        # if the robot didn't move don't make any update (otherwise the covariance
        # will grow unnecessarily large)
        if abs(self.delta_s) < 0.001 and abs(self.delta_theta) < 0.001:
            return

        s, theta = self.delta_s, self.delta_theta
        a = self.pose[2] + float(theta)/2
        Fkv = np.array([[np.cos(a), -s * np.sin(a)],
                        [np.sin(a), s * np.cos(a) ],
                        [  0      ,       1       ]])

        Fktsi = np.array([[1, 0, -s*np.sin(a)],
                          [0, 1,  s*np.cos(a)],
                          [0, 0,      1      ]])
                       
        tsi_delta = np.array([(s * float(np.cos(a))),
                              (s * float(np.sin(a))),
                                       theta       ])
                             
        tsik1_k = np.add(self.pose, tsi_delta)
        self.pose = tsik1_k

        
        first = self.cov_matrix_after_linear_transf(Fktsi, self.pose_covariance)
        second = self.cov_matrix_after_linear_transf(Fkv, self.motion_covariance)
        self.pose_covariance = np.add(first, second)

        # after pose and pose_covariance are updated, the errors measures can be updated too
        # compute determinant of covariance matrix and MSE vs. odometry's pose
        self.update_error_indicators()

        # publish ekf and odom pose estimate as Path messages that can be displayed in rviz
        self.publish_ekf_and_odom_paths()

        self.ekf_pose.publish(self.pose)

    # whenever you see landmark, do this:
    def observation_correction(self, msg):
        '''CALLBACK function that correct state estimatse based on marker observation. 
           AR visual markers are read by the package ar_track_alvar, 
           that publishes the detected markers in /ar_pose_marker.

           -> This method required that ar_track_alvar is up and running for detecting the ar tags:
                 $ roslaunch my_ar_track_alvar ar_track_alvar_gazebo.launch 
           The 'my_' version is to set in the launch file the right values for marker size, as well
           as for the camera topic and for the sensing errors.

           For checking the numeric values of detected markers:
                 $ rostopic echo /ar_pose_marker 

           To visualize (in simulation)
                 $ roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch   
           in rviz, add Markers and set the topic: /visualization_marker

           Note: use the gazebo_models/ar_tags/scripts/generate_markers_model.py 
                 to generate additional markers (up to 65k!)
        '''
        W = np.array([[2, 0], [0, 2]])
        # variations in traveled distance and orientation angle derived from measured v and omega
        self.delta_s = float(self.delta_t * (float(self.v_now + self.v_before)/2.))
        self.delta_theta = float(self.delta_t * (float(self.omega_now + self.omega_before)/2.))

        # store current velocity values as the initial values for the next updating step
        self.v_before, self.omega_before = self.v_now, self.omega_now

        # if the robot didn't move don't make any update (otherwise the covariance
        # will grow unnecessarily large)
        if abs(self.delta_s) < 0.001 and abs(self.delta_theta) < 0.001:
            return

        s, theta = self.delta_s, self.delta_theta
        a = self.pose[2] + float(theta)/2
        Fkv = np.array([[np.cos(a), -s * np.sin(a)],
                        [np.sin(a), s * np.cos(a) ],
                        [  0      ,       1       ]])

        Fktsi = np.array([[1, 0, -s*np.sin(a)],
                          [0, 1,  s*np.cos(a)],
                          [0, 0,      1      ]])

        tsi_delta = np.array([(s * float(np.cos(a))),
                              (s * float(np.sin(a))),
                              theta       ])

        tsik1_k = np.add(self.pose, tsi_delta)
        self.pose = tsik1_k


        first = self.cov_matrix_after_linear_transf(Fktsi, self.pose_covariance)
        second = self.cov_matrix_after_linear_transf(Fkv, self.motion_covariance)
        self.pose_covariance = np.add(first, second)


       
        
        # loop over all the markers that have been currently detected 
        for m in msg.markers:            
            landmark_id = m.id
            print 'Observed landmark: ', landmark_id

            x = m.pose.pose.position.x
            y = m.pose.pose.position.y
            print("x coordinate: %f, y coordinate: %f" %(x, y))

            #### Your code here ###

            # - \rho, range value of the landmark observation
            # - \beta, bearing value of the landmark observation (you shall express it as an angle)
            rho = np.sqrt((x)**2 + (y)**2)
            beta = np.arctan(y/x)
            # - and create z_{k+1} = [\rho, \beta]^T, the observation vector, accordingly
             
            z_k1 = np.array([[rho], [beta]])
            # - \lambda_x^i, the x coordinate of the landmark, from the map
            # - \lambda_y^i, the y coordinate of the landmark, from the map
            lambda_x = ((self.landmark_map).get(m.id))[0]
            lambda_y = ((self.landmark_map).get(m.id))[1]
            
            # - \hat{z}_{k+1}, the predicted observation vector based on estimated pose and given map
            #   (you shall use the sefl.angle_linear_comp() helper method for composing angles)
            hat_rho = np.sqrt((lambda_x - self.pose[0])**2 + (lambda_y - self.pose[1])**2)
            hat_beta = self.angle_linear_comp(np.arctan2((lambda_y - self.pose[1]),(lambda_x - self.pose[0])), self.pose[2], "-")
            hat_zk1 = np.array([[hat_rho], [hat_beta]])
            
            # - \epsilon = z_{k+1} - \hat{z}_{k+1}, the innovation vector
            epsilon = np.array([[rho - hat_rho], [self.angle_linear_comp(beta, hat_beta, "-")]])
            # - All the Jacobians, H_xi, H_w that are necessary
            H_w = np.array([[1, 0],
                            [0, 1]])
            H_k = np.array([[-(lambda_x - self.pose[0])/hat_rho, -(lambda_y - self.pose[1])/hat_rho, 0],
                            [ (lambda_y - self.pose[1])/(hat_rho)**2, -(lambda_x - self.pose[0])/(hat_rho)**2, -1]])
            # - S, the covariance matrix of the innovation vector, which is used as S^{-1} as
            #   the weight for the kalman gain, G
            S = np.add(self.cov_matrix_after_linear_transf(H_k, self.pose_covariance),
                       self.cov_matrix_after_linear_transf(H_w, W))
            
            
            # - G, the Kalman gain
            G = np.matmul(self.pose_covariance, np.matmul(H_k.T, np.linalg.inv(S)))

            print(z_k1)
            print(hat_zk1)
            print(epsilon)
            inovation = np.matmul(G, epsilon)
            print(np.ndarray.flatten(inovation))
            self.pose = np.add(self.pose, np.ndarray.flatten(np.matmul(G, epsilon)))
            cov = np.matmul(G, np.matmul(H_k, self.pose_covariance))
            self.pose_covariance = np.subtract(self.pose_covariance, cov)



        
            # after pose and pose_covariance are updated, the errors measures can be updated too
            # compute determinant of covariance matrix and MSE vs. odometry's pose
            self.update_error_indicators()

            # publish ekf and odom pose estimate as Path messages that can be displayed in rviz
            self.publish_ekf_and_odom_paths()

        self.ekf_pose.publish(self.pose)
            

        
    ##################### Helper methods ####################
        
    def cov_matrix_after_linear_transf(self, A, cov_X):
        '''Given a linear variable transformation Y = AX + C for a 
           multivariate r.v. A, if cov_X is the covariance of A, the covariance
           matrix of the multivariate r.v. Y is A cov_X A.T. 
           This method computes and returns the linearly transformed covariance.
        '''
        return np.matmul(np.matmul(A, cov_X), A.T)
    
    def update_error_indicators(self):
        '''Compute determinant of covariance matrix and update value of MSE vs. odometry's pose'''
        
        self.pose_covariance_det = np.round(np.sqrt(np.linalg.det(self.pose_covariance)), 3)

        self.cnt += 1
        self.MSE += (self.pose[0] - self.odo_x)**2 + (self.pose[1] - self.odo_y)**2

        
    def angle_linear_comp(self, a1, a2, operation='+'):
        '''Helper function to perform addition and subtraction of angles based on formula:

           atan2(y1, x1) +- atan2(y2, x2) = atan2(y1*x2 +- y2*x1,  x1 * x2 -+ y1*y2)
        '''
        x1 = np.cos(a1)
        y1 = np.sin(a1)

        x2 = np.cos(a2)
        y2 = np.sin(a2)
        if operation == '+':
            return np.arctan2(y1*x2 + y2*x1,  x1*x2 - y1*y2)
        else:
            return np.arctan2(y1*x2 - y2*x1,  x1*x2 + y1*y2)
                             
    def mse(self):
        '''Compute the root mean square value of the cumulative estimation errors'''
        return np.sqrt(self.MSE)/float(self.cnt)

    
    def publish_ekf_and_odom_paths(self):
        '''Transform odom poses and ekf poses into Path messages for rviz.
           In rviz, Add a Path and subscribe it to these topics to 
           see the followed paths in the /odom frame.
           For a good view: set all shaft and head values to 0.01, and buffer length > 200'''

        self.path_seq += 1

        # create a path msg from last odometry msg
        path_odom = Path()
        path_odom.header.seq = self.path_seq
        path_odom.header.frame_id = 'odom'
        path_odom.header.stamp = rospy.Time.now()

        pose = PoseStamped() 
        pose.header.seq = self.path_seq
        pose.header.stamp = path_odom.header.stamp
        pose.pose.position = deepcopy(self.last_odo_msg.pose.pose.position)
        pose.pose.orientation = deepcopy(self.last_odo_msg.pose.pose.orientation)
        path_odom.poses.append(pose)

        # publish odometry to the odometry path topic
        self.odom_path_pub.publish(path_odom)

        # create a path msg from ekf pose estimate
        path_ekf = Path()
        path_ekf.header.seq = self.path_seq
        path_ekf.header.frame_id = 'odom'
        path_ekf.header.stamp = rospy.Time.now()

        pose = PoseStamped()
        pose.header.seq = self.path_seq
        pose.header.stamp = path_ekf.header.stamp
        pose.pose.position = Point(self.pose[0], self.pose[1], 0.0) 
        pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, self.pose[2]))
        path_ekf.poses.append(pose)

        # publish pose from ekf to the ekf path topic
        self.ekf_path_pub.publish(path_ekf)


        
    
########## Main loop: state updating when robot moves and makes observations ########
    
if __name__ == '__main__':

    rospy.init_node('ekf_with_ar_tags_landmarks', log_level=rospy.INFO) # DEBUG, INFO, WARN, ERROR, FATAL

    pose_estimator = EKF_Localization_LandmarkMap()
        
    while True:

        # makes a prediction update whenever the robot has moved
        pose_estimator.prediction_update()
        
        print 'Pose: {:.2f}, {:.2f}, {:.2f} - Det: {:.2f}'.format(float(pose_estimator.pose[0]),
                                                                  float(pose_estimator.pose[1]),
                                                                  float(np.degrees(pose_estimator.pose[2])),
                                                                  pose_estimator.pose_covariance_det)

        print 'MSE EKF vs. Odom: {:.3f}'.format(float(pose_estimator.mse())) 
        
        pose_estimator.Rate.sleep()
