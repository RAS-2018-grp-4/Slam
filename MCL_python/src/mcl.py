#!/usr/bin/env python
'''
Monte carlo localization
Source: https://github.com/pjensfelt/matlab_loc_demos/blob/master/MCL.m
NOTE: variable names are copied from ^ for consistency

Steps:
    - set design variables 
    - initialize particles (either with known prior or uniformly)
        - if reset_flag = 0:    with known prior, and it takes present odometry
        - if reset_flag = 1:    uniformly
    - subscribe to odometry (prediction step)
    - subscribe to raw laser scan. transform laser scan for every particle
    - subscribe to grid_map to get ground truth 
    - find scan correlation for every particle w.r.t. grid_map
    - update weights (resample) depending on scan correlation for every particle

Brainstorm:
    1. particle can be a class?? member variables could be:
        - x
        - y
        - theta
        - laser scan for it
        - resampling weight 
    
    2. odometry should be calculated here!
        - we subsume ras_odom_publisher package. we create a class called odom. odom calculation is handled by this class
        and 
'''
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import tf

# for odom calculation
import std_msgs.msg
import phidgets.msg
import geometry_msgs.msg
import math

class MCL_py():

    def __init__(self):
        ### FOR ODOM CALCULATION
        # design variables 
        self.base = 0.205
        self.wheel_radius = 0.0485
        self.ticks_per_rev = 897.96
        self.control_frequency = 10
        self.dt = 1/self.control_frequency

        # Intermediate variables
        self.encoder_right = 0
        self.encoder_left = 0
        self.v_left = 0             #left wheel velocity 
        self.v_right = 0            #right wheel vel
        self.sita = 0
        self.temp_x = 0
        self.temp_y = 0

        # Variables of interest
        self.tspeed = 0
        self.rspeed = 0 
        self.robot_odom = Odometry()
        self.RESET = False
        
        ### FOR PF
        # design variables
        self.M = 100            # No. of particles
        self.tdStd = 0.1        # std dev. proportional to dist travelled in last iter 
        self.rdaStd = 0.1       # std dev. proportional to delta angle in last iter
        self.rdStd = 0.1        # ??
        
        # variables of interest
        self.robot_scan = LaserScan()
        self.particle_list = []
        self.grid_map = OccupancyGrid()
        # Publisher for robot odometry
        self.pub_odom = rospy.Publisher('/robot_odom', Odometry, queue_size=1)


        # self.received_odom = 0
        # self.received_scan = 0
        # self.received_map = 0

    '''MEMBER FUNCTIONS'''
    # /left_motor/encoder Callback          
    def update_feedback_enc_left(self, feedback_enc):
        self.encoder_left = self.encoder_left + feedback_enc.count_change

    # /right_motor/encoder Callback
    def update_feedback_enc_right(self, feedback_enc):
        # NOTE THE MINUS SIGN!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        self.encoder_right = self.encoder_right - feedback_enc.count_change

    # /odom_reset Callback
    def reset_feedback(self, feedback_reset):
        #global RESET
        if feedback_reset.data == True:
            self.RESET = True
        else:
            pass

    def calc_and_pub_odom(self):
        self.v_left = self.wheel_radius * (self.encoder_left * 2 * np.pi * self.control_frequency) / (self.ticks_per_rev)
        self.v_right = self.wheel_radius*(self.encoder_right * 2 * np.pi * self.control_frequency) / (self.ticks_per_rev)
        self.sita = self.sita + (1/self.base) * self.dt * (self.v_right - self.v_left)

        self.tspeed = 0.5 * (self.v_left + self.v_right)
        self.rspeed = (1/self.base) * (self.v_right - self.v_left)

        self.temp_x = self.temp_x + 0.5 * (self.v_left + self.v_right) * math.cos(self.sita)*self.dt
        self.temp_y = self.temp_y + 0.5 * (self.v_left + self.v_right) * math.sin(self.sita)*self.dt

        if self.RESET:
            x = 0
            y = 0
            self.sita = 0
            self.RESET = False
        else:
            pass

        self.robot_odom.pose.pose.position.x = self.temp_x
        self.robot_odom.pose.pose.position.y = self.temp_x
        
        quaternion = tf.transformations.quaternion_from_euler(0,0,self.sita)
        #ODOM.pose.pose.orientation.z = sita
        #ODOM.pose.pose.orientation.w = sita
        #print(quaternion[3])
        self.robot_odom.pose.pose.orientation.z = quaternion[2]
        self.robot_odom.pose.pose.orientation.w = quaternion[3]
	#ODOM.twist.twist.angular.z = sita
        self.robot_odom.header.stamp = rospy.get_rostime()
        self.robot_odom.header.frame_id = "odom"
        self.robot_odom.child_frame_id = "base_link"
        self.pub_odom.publish(self.robot_odom)

        # flush the encoders
        self.encoder_right = 0
        self.encoder_left = 0
        
    def prediction_step(self):
        for i in range(self.M):
            dnoise = (self.tspeed*self.dt*self.tdStd)*np.random.randn
            arnoise = (self.rspeed*self.dt*self.rdaStd)*np.random.randn
            atnoise = (self.tspeed*self.dt*self.tdStd)*np.random.randn
            
            self.particle_list[i].x = self.particle_list[i].x + (self.tspeed*self.dt+dnoise)*np.cos(self.particle_list[i].theta)
            self.particle_list[i].y = self.particle_list[i].y + (self.tspeed*self.dt+dnoise)*np.sin(self.particle_list[i].theta)
            self.particle_list[i].theta = self.particle_list[i].theta + (self.rspeed*self.dt+arnoise) + atnoise
            # X(2,n) = X(2,n) + (tspeed*dT+dnoise)*sin(X(3,n))
            # X(3,n) = X(3,n) + (rspeed*dT+arnoise) + atnoise



    # def callback_odom(odom_msg):
    #     self.robot_odom = odom_msg
    #     #self.received_odom = 1

    # Callback for laser scan
    def callback_scan(self, scan_msg):
        self.laser_scan = scan_msg
        #self.received_scan = 1

    # callback for occupancy grid
    def callback_grid_map(self, map_msg):
        self.grid_map = map_msg
        #self.received_map = 1

    def reset_particles(self, reset_flag):
        # Erase previous particle list      ### WARNING: do we really delete prev particles?
        self.particle_list = []

        if reset_flag == 0:     #prior known, set all particles acc to present odom
            for i in range(self.M):
                self.particle_list.append(particle(self.robot_odom, self.robot_scan, self.M))





class particle():

    def __init__(self, given_pose, given_laser_scan, M):
        # given pose can be anything, even set manually
        self.particle_odom = given_pose
        self.x = given_pose.pose.pose.position.x
        self.y = given_pose.pose.pose.position.y
        (_,_,self.theta) = tf.transformations.euler_from_quaternion([given_pose.pose.pose.orientation.x, given_pose.pose.pose.orientation.y, given_pose.pose.pose.orientation.z, given_pose.pose.pose.orientation.w])
        self.weight = 1/M
        self.particle_scan = given_laser_scan 




''' MAIN '''
def main():
    rospy.init_node('mcl_py_node', anonymous=True)

    # create an instance of class MCL_py
    mcl_obj = MCL_py() 
    
    rate = rospy.Rate(mcl_obj.control_frequency)

    # Subscribe to Left Encoder 
    rospy.Subscriber('/left_motor/encoder', phidgets.msg.motor_encoder, update_feedback_enc_left)

    # Subscribe to Right Encoder 
    rospy.Subscriber('/right_motor/encoder', phidgets.msg.motor_encoder, update_feedback_enc_right)

    # Subscribe to Reset Flag 
    rospy.Subscriber('/odom_reset', std_msgs.msg.Bool, reset_feedback)

    # Subscribe to raw laser scan
    rospy.Subscriber('/scan', Odometry, mcl_obj.callback_scan)
    
    # Subscribe to grid map
    rospy.Subscriber('/maze_map_node/map', OccupancyGrid, mcl_obj.callback_grid_map)
    
    #rospy.spinOnce()

    # Reset particle_list with known prior
    # In this particular case, subscriber callbacks have not yet been called, so everything initializes to 0
    #if mcl_obj.received_odom == 1 and mcl_obj.received_scan == 1 and mcl_obj.received_map == 1:  
    mcl_obj.reset_particles(0)

    while not rospy.is_shutdown():
        # Calculate robot odometry
        mcl_obj.calc_and_pub_odom()

        # Prediction step
        mcl_obj.prediction_step()

        rate.sleep()

        
if __name__ == '__main__':
    main()