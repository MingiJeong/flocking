#!/usr/bin/env python

# import of python modules
import math
import numpy as np
import random

# import of relevant libraries
import rospy # module for ROS APIs
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# import custom modules
from aux_function import *

# Constants 
DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'
DEFAULT_SCAN_TOPIC = 'scan'
DEFAULT_GT_TOPIC = 'ground_truth/state'
DEFAULT_SCAN_TOPIC_STAGE = 'base_scan'
DEFAULT_GT_TOPIC_STAGE = 'base_pose_ground_truth'

MIN_THRESHOLD_DISTANCE_ALL = 1 # m, threshold distance. 0.7
MIN_THRESHOLD_DISTANCE_FRONT = 1 # m, threshold distance. 1
LASER_ANGLE_LEFT = math.pi * 1/4 # Gazebo 1/4  9/16  1/3 
LASER_ANGLE_RIGHT = math.pi * 7/4 # Gazebo 7/4 23 /16  5/3

class FlockingRobot():
    def __init__(self, robot_name, simul_gazebo, robot_total):
        """ Constructor """
        self.robot_name = robot_name # own robot ID (int)
        self.simul_gazebo = simul_gazebo # flag: using gazebo or not (i.e., stage)
        self.robot_total = robot_total # total number of robot on scene (assumption > |# of own robot|)
        self.own_heading = None
        self.own_position = None
        self.other_heading = dict()
        self.other_position = dict()

        self.control_state = 0  # 0: flocking / 1: obstacle avoidance
        
        # setting up publishers/subscribers
        if self.simul_gazebo: # gazebo flag
            self._cmd_pub = rospy.Publisher("tb3_{}".format(str(self.robot_name)) + "/" + DEFAULT_CMD_VEL_TOPIC, Twist, queue_size=1)
            self._laser_sub = rospy.Subscriber("tb3_{}".format(str(self.robot_name)) + "/" + DEFAULT_SCAN_TOPIC, LaserScan, self._laser_call_back, queue_size=1)       
            
            # subscribers of other robot ground truth 
            for i in range(self.robot_total):
                rospy.Subscriber('tb3_{}'.format(str(i)) + "/" +  DEFAULT_GT_TOPIC, Odometry, callback = self._gt_call_back, callback_args=i)

        else: # stage flag
            self._cmd_pub = rospy.Publisher("robot_{}".format(str(self.robot_name)) + "/" + DEFAULT_CMD_VEL_TOPIC, Twist, queue_size=1)
            self._laser_sub = rospy.Subscriber("robot_{}".format(str(self.robot_name)) + "/" + DEFAULT_SCAN_TOPIC_STAGE, LaserScan, self._laser_call_back, queue_size=1)

            # subscribers of other robot ground truth 
            for i in range(self.robot_total):
                rospy.Subscriber('robot_{}'.format(str(i)) + "/" + DEFAULT_GT_TOPIC_STAGE, Odometry, callback = self._gt_call_back, callback_args=i)
        
        # get params from .yaml file via param server
        self.rate = rospy.Rate(rospy.get_param('~FREQUENCY'))
        self.cohesion_coff = rospy.get_param('~COHESION_COEFF')
        self.separation_coff = rospy.get_param('~SEPARATION_COEFF')
        self.alignment_coff = rospy.get_param('~ALIGNMENT_COEFF')
        self.scaling_pos = rospy.get_param('~SCALING_POS')
        self.scailing_ang = rospy.get_param('~SCALING_ANG')

        self.evacuate_spd = random.choice([-0.3,0.3])

    def _laser_call_back(self, msg):
        """Processing of laser message."""
        # Access to the index of the measurement of the robot.
        if self.simul_gazebo: # GAZEBO environment
            i = int((LASER_ANGLE_LEFT - msg.angle_min) / msg.angle_increment)
            j = int((LASER_ANGLE_RIGHT - msg.angle_min) / msg.angle_increment)

            front_left = min(msg.ranges[0:i+1])
            front_right = min(msg.ranges[j:])
            front_dist = min(front_left, front_right)  # min front side distance

        else: # STAGE environment
            i = int((-LASER_ANGLE_LEFT - msg.angle_min) / msg.angle_increment)
            j = int((LASER_ANGLE_RIGHT - msg.angle_min) / msg.angle_increment)

            front_dist = min(msg.ranges[i:j+1]) # min front side distance

        overall_min_dist = min(msg.ranges)  # overall min distance (full angle)

        # overall distance and front distance are less than threshold (consider it will collide)
        # example: robot1 detect robot2 behind it very close, but nothing ahead. 
        #          robot1 does not need to take the evacuate action, i.e., can go ahead
        #          robot2 has robot1 in the front; need to take the evacuate action.
        if overall_min_dist <= MIN_THRESHOLD_DISTANCE_ALL and front_dist <= MIN_THRESHOLD_DISTANCE_FRONT:
            self.control_state = 1  # collision avoidance initiate / flock stop for a while
            self.evacuate(front_dist)

        # condition not met: all clear => flocking
        else:
            self.control_state = 0

    def _gt_call_back(self, msg, robot_name):
        """
        function to subscribe ground truth of robot members (own robot + the rest)
        - arg: robot_name to be used mapping heading data, position data for own (plain) or others (hash-map)
        """
        # Quaterinion to Euler
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        # other robot heading and position update
        if self.robot_name != robot_name:
            self.other_heading[robot_name] = yaw  # if heading available, automatically position avaiable under the same ground truth msg
            self.other_position[robot_name] = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        # own robot heading and position update
        else:
            self.own_heading = yaw  # if heading available, automatically position avaiable under the same ground truth msg
            self.own_position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])

    def alignment(self):
        """
        function to make the robot align with average heading of the rest members
        """
        # average heading vector which will be normalized in the end      
        vec = np.array([0, 0])
        for other_hdg in self.other_heading.values():
            other_unit_vec = np.array([math.cos(other_hdg), math.sin(other_hdg)])
            vec = vec + other_unit_vec  # add
        
        # return normalized (averaging effect) vector for alignment
        return normalize(vec)

    def cohesion(self):
        """
        """
        # average position vector which will be normalized in the end
        vec = np.array([0, 0])

        for other_psn in self.other_position.values():
            vec = vec + other_psn
        cohesion_avg_vec = normalize(vec)
        own_psn_vec = self.get_own_psn_vector() # my current position vector
        cohesion_vec = normalize(cohesion_avg_vec - own_psn_vec)   

        # return normalize(vec)
        return cohesion_vec

    def separation(self):
        # average separation vector which will be normalized in the end
        vec = np.array([0, 0])
        own_psn_vec = self.get_own_psn_vector()
        
        for other_psn in self.other_position.values():
            repulsive_vec = own_psn_vec - other_psn  # repulsive vec to my robot
            dist = np.linalg.norm(repulsive_vec)  # distance
            adj_repulsive_vec = normalize(repulsive_vec) / dist # inverse proportional
            vec = vec + adj_repulsive_vec 
        
        return normalize(vec)

    def get_own_hdg_vector(self):
        """
        function to return 1x2 vector of own heading with unit scale
        """
        return np.array([math.cos(self.own_heading), math.sin(self.own_heading)])

    def get_own_psn_vector(self):
        """
        function to return 1x2 vector of own position on the global frame
        """
        return self.own_position

    def flocking_combination(self):
        """
        function to achieve flocking behaviors consisting of the sub-modules:
            1) alignment 2) cohesion 3) separation
        """
        # twist message initialization
        twist_msg = Twist()

        # All data receiving conditions are met
        if len(self.other_heading) == self.robot_total - 1 and self.own_heading is not None and self.control_state == 0:
            
            # =========================================================
            # 1. Alignment 
            # =========================================================
            alignment_vec = self.alignment()  # other robots' avg heading vector
            own_hdg_vec = self.get_own_hdg_vector()  # my current heading vector

            # =========================================================
            # 2. Cohesion
            # =========================================================
            cohesion_vec = self.cohesion() 

            # =========================================================
            # 3. Separation 
            # =========================================================
            separation_vec = self.separation()
            
            # =========================================================
            # 4. Final output
            # =========================================================
            sum_vec = normalize(
                self.alignment_coff * alignment_vec 
                + self.cohesion_coff * cohesion_vec 
                + self.separation_coff * separation_vec
                )
            
            # angular component
            adjust_angle = check_vector_angle(own_hdg_vec, sum_vec)
            # adjust_angle = check_vector_angle(own_hdg_vec, alignment_vec)

            # linear component
            adjust_position = np.linalg.norm(
                self.cohesion_coff * cohesion_vec 
                + self.separation_coff * separation_vec
                )
            # adjust_position = np.linalg.norm(sum_vec - own_psn_vec)

            # adjust with scaling coefficient
            twist_msg.linear.x = self.scaling_pos * adjust_position
            twist_msg.angular.z = self.scailing_ang * adjust_angle
            
            # final output of cmd_vel message
            self._cmd_pub.publish(twist_msg)

    
    def evacuate(self, front_dist):
        """
        function to make the robot evacuate from colliding with each other while flocking
        """
        twist_msg = Twist()
        twist_msg.angular.z = self.evacuate_spd # randomly chosen direction per robot 

        self._cmd_pub.publish(twist_msg)
        self.rate.sleep()

        # front is clear and the robot can go ahead!
        if front_dist > MIN_THRESHOLD_DISTANCE_FRONT:
            self.control_state = 0 # clear and flocking resume


    def spin(self):
        """
        main looping function
        """
        while not rospy.is_shutdown():
            # flocking
            self.flocking_combination()

            # Sleep to keep the set publishing frequency.
            self.rate.sleep()
