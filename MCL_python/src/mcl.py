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
import matplotlib.pyplot as plt
from geometry_msgs.msg import  Pose, PoseArray
from sklearn.preprocessing import normalize

# for odom calculation
import std_msgs.msg
import phidgets.msg
import geometry_msgs.msg
import math

class State():
    def __init__(self):
        self.x = 0
        self.y = 0
        self.theta = math.pi/2
    def __str__(self):
        return "Pose:({0},{1},{2})".format(self.x, self.y, self.theta)

class particle():
    def __init__(self, given_pose, given_laser_scan, M):
        # given pose can be anything, even set manually
        self.particle_odom = given_pose
        self.x = given_pose.pose.pose.position.x
        self.y = given_pose.pose.pose.position.y
        (_,_,self.theta) = tf.transformations.euler_from_quaternion([given_pose.pose.pose.orientation.x, given_pose.pose.pose.orientation.y, given_pose.pose.pose.orientation.z, given_pose.pose.pose.orientation.w])
        self.theta = self.theta
        self.weight = 1/M
        #self.particle_scan = given_laser_scan
        #self.scan_map = np.zeros 

class MCL_py():
    def __init__(self):
        '''MEMBER VARIABLES'''
        ### FOR ODOM CALCULATION
        # design variables 
        self.base = 0.205
        self.wheel_radius = 0.0485
        self.ticks_per_rev = 897.96
        self.control_frequency = 10
        self.dt = 1.0/self.control_frequency

        # Intermediate variables
        self.encoder_right = 0
        self.encoder_left = 0
        self.v_left = 0             #left wheel velocity 
        self.v_right = 0            #right wheel vel
        self.sita = math.pi/2
        self.temp_x = 0
        self.temp_y = 0

        # Variables of interest
        self.tspeed = 0
        self.rspeed = 0 
        self.robot_odom = Odometry()
        self.RESET = False
        #self.particle_cloud = PoseArray()
        
        ### FOR PF
        # design variables
        self.M = 5000            # No. of particles
        self.tdStd = 0.4        # std dev. proportional to dist travelled in last iter 
        self.rdaStd = 0.2       # std dev. proportional to delta angle in last iter
        self.rdStd = 0.1        # ??
        
        # variables of interest
        self.robot_scan = LaserScan()
        self.particle_list = []
        self.particles = np.zeros((self.M,4))
        self.particles[:,2] = math.pi/2
        self.particles[:,3] = 1.0/self.M
        print(self.particles)
        self.grid_map = OccupancyGrid()
        # 0: redistribute particles according to odom
        # 1: redistribute particles randomly over the map
        self.reset_flag = 0       
        self.LISTENER = tf.TransformListener()
        self.PARTICLE_SCAN  = geometry_msgs.msg.PointStamped()

        # parameters for resampling
        self.has_moved = False      # will only re-sample if has_moved
        self.step_scan_match = 10   # step size betwen scan matching (50 is for "global" localization)

        # consensus pose
        self.state_filter = State()
        self.state_filter_odom = Odometry()

        '''PUBLISHERS'''
        # Publisher for robot odometry
        self.pub_odom = rospy.Publisher('/robot_odom', Odometry, queue_size=1)
        self.pub_filter = rospy.Publisher('/robot_filter', Odometry, queue_size=1)
        
        # Publisher for particle cloud
        self.pub_pc = rospy.Publisher('/particlecloud', PoseArray, queue_size=1)        

        # Publisher for particle scan transform
        self.pub_pscan = rospy.Publisher('/particlescan', PoseArray, queue_size=1)    

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

    # def callback_odom(odom_msg):
    #     self.robot_odom = odom_msg
    #     #self.received_odom = 1

    # Callback for laser scan
    def callback_scan(self, scan_msg):
        self.robot_scan = scan_msg
        #self.received_scan = 1

    # callback for occupancy grid
    def callback_grid_map(self, map_msg):
        self.grid_map = map_msg
        #self.received_map = 1

    def calc_and_pub_odom(self):
        self.v_left = self.wheel_radius * (self.encoder_left * 2 * np.pi * self.control_frequency) / (self.ticks_per_rev)
        self.v_right = self.wheel_radius*(self.encoder_right * 2 * np.pi * self.control_frequency) / (self.ticks_per_rev)
        self.sita = self.sita + (1/self.base) * self.dt * (self.v_right - self.v_left)

        # update has_moved flag
        if abs(self.v_left) > 0.001 or abs(self.v_right) > 0.001:
            self.has_moved = True

        self.tspeed = 0.5 * (self.v_left + self.v_right)
        self.rspeed = (1/self.base) * (self.v_right - self.v_left)

        self.temp_x = self.temp_x + 0.5 * (self.v_left + self.v_right) * math.cos(self.sita)*self.dt
        self.temp_y = self.temp_y + 0.5 * (self.v_left + self.v_right) * math.sin(self.sita)*self.dt

        if self.RESET:
            # robot_odom is reset to initial position
            self.temp_x = 0
            self.temp_y = 0
            self.sita = math.pi/2
        else:
            pass

        self.robot_odom.pose.pose.position.x = self.temp_x
        self.robot_odom.pose.pose.position.y = self.temp_y
        
        quaternion = tf.transformations.quaternion_from_euler(0,0,self.sita)
        #ODOM.pose.pose.orientation.z = sita
        #ODOM.pose.pose.orientation.w = sita
        #print(quaternion[3])
        if not np.any(np.isnan(quaternion)):
            self.robot_odom.pose.pose.orientation.z = quaternion[2]
            self.robot_odom.pose.pose.orientation.w = quaternion[3]
        #ODOM.twist.twist.angular.z = sita
            self.robot_odom.header.stamp = rospy.get_rostime()
            self.robot_odom.header.frame_id = "odom"
            self.robot_odom.child_frame_id = "base_link"
            self.pub_odom.publish(self.robot_odom)

            # now that new odom is published, check if particles need to be reset
            if self.RESET:
                # all particles are set to their initial positions
                # if self.reset_flag is 0, all particles set to initial odom
                # else if its 1, randomize particle poses again
                self.reset_particles()

                self.RESET = False
            else:
                pass

            # flush the encoders
            self.encoder_right = 0
            self.encoder_left = 0
            
    def prediction_step(self):
        # for i in range(self.M):
        #     dnoise = (self.tspeed*self.dt*self.tdStd)*np.random.randn()
        #     arnoise = (self.rspeed*self.dt*self.rdaStd)*np.random.randn()
        #     atnoise = (self.tspeed*self.dt*self.tdStd)*np.random.randn()
            
        #     self.particle_list[i].x = self.particle_list[i].x + (self.tspeed*self.dt+dnoise)*np.cos(self.particle_list[i].theta)
        #     self.particle_list[i].y = self.particle_list[i].y + (self.tspeed*self.dt+dnoise)*np.sin(self.particle_list[i].theta)
        #     self.particle_list[i].theta = self.particle_list[i].theta + (self.rspeed*self.dt+arnoise) + atnoise
        dnoise = (self.tspeed*self.dt*self.tdStd)*np.random.randn()
        arnoise = (self.rspeed*self.dt*self.rdaStd)*np.random.randn()
        atnoise = (self.tspeed*self.dt*self.tdStd)*np.random.randn()
        for i in range(self.M):
            dnoise = (self.tspeed*self.dt*self.tdStd)*np.random.randn()
            arnoise = (self.rspeed*self.dt*self.rdaStd)*np.random.randn()
            atnoise = (self.tspeed*self.dt*self.tdStd)*np.random.randn()      
            self.particles[i,0] = self.particles[i,0] + (self.tspeed*self.dt+dnoise)*np.cos(self.particles[i,2])# + 0.005*np.random.randn()
            self.particles[i,1] = self.particles[i,1] + (self.tspeed*self.dt+dnoise)*np.sin(self.particles[i,2])# + 0.005*np.random.randn()
            self.particles[i,2] = self.particles[i,2] + (self.rspeed*self.dt+arnoise)# + atnoise + 0.05*np.random.randn()


    def reset_particles(self):
        # # Erase previous particle list      ### WARNING: do we really delete prev particles?
        # self.particle_list = []
        # #self.reset_flag = flag

        # if self.reset_flag == 0:     #prior known, set all particles acc to present odom
        #     for i in range(self.M):
        #         self.particle_list.append(particle(self.robot_odom, self.robot_scan, self.M))

        #self.particles = np.zeros((self.M,4))
        self.particles[:,3] = 1.0/self.M

        if self.reset_flag == 0:         #prior known, set all particles acc to present odom
            for i in range(self.M):
                self.particles[i,0] = self.robot_odom.pose.pose.position.x + np.random.uniform(-0.3, 0.3)
                self.particles[i,1] = self.robot_odom.pose.pose.position.y + np.random.uniform(-0.3, 0.3)
                (_,_,theta) = tf.transformations.euler_from_quaternion([self.robot_odom.pose.pose.orientation.x, self.robot_odom.pose.pose.orientation.y, self.robot_odom.pose.pose.orientation.z, self.robot_odom.pose.pose.orientation.w])
                self.particles[i,2] = theta + np.random.uniform(-np.pi/4, np.pi/4)

        elif self.reset_flag == 1: 
            for i in range(self.M):
                rand_x = np.random.randint(0, self.grid_map.info.width)
                rand_y = np.random.randint(0, self.grid_map.info.height)
                rand_theta = np.random.uniform(0, 2 * np.pi)

                if self.grid_map.data[rand_x + rand_y * self.grid_map.info.width] == 0:
                    self.particles[i,0] = rand_x * self.grid_map.info.resolution - 0.2
                    self.particles[i,1] = rand_y * self.grid_map.info.resolution - 0.2
                    self.particles[i,2] = rand_theta


    def weight_update(self):
        #scan_data = np.zeros((len(self.robot_scan.ranges),2))
        scan = self.robot_scan
        # scan = self.robot_scan
   
        # global --> local localization
        if self.M == 5000:
            if np.std(self.particles[range(0,self.M),0] - np.mean(self.particles[range(0,self.M),0])) < 0.5 and np.std(self.particles[range(0,self.M),1] - np.mean(self.particles[range(0,self.M),1])) < 0.5:
                self.M = 500
                self.step_scan_match = 20

        for k in range(self.M):
            count_good = 0.0

            if len(scan.ranges) > 0:
                for i in range(0, len(scan.ranges), self.step_scan_match):
                    if scan.ranges[i] <= scan.range_min or scan.ranges[i] >= scan.range_max or scan.ranges[i] == float("inf"):
                        continue
                    else:
                        current_bearing = scan.angle_min + i * scan.angle_increment
                        # x_map = self.particle_list[k].x + scan.ranges[i] * np.cos(current_bearing + self.particle_list[k].theta)
                        # y_map = self.particle_list[k].y + scan.ranges[i] * np.sin(current_bearing + self.particle_list[k].theta)
                        
                        # TODO: FIX THE 0.2 (ODOM --> TO MAP FRAME) -------------------------------------------------------------------------------------------
                        x_map = 0.2 + self.particles[k,0] + scan.ranges[i] * np.cos(current_bearing + self.particles[k,2])
                        y_map = 0.2 + self.particles[k,1] + scan.ranges[i] * np.sin(current_bearing + self.particles[k,2])
                        
                        temp_val = x_map/self.grid_map.info.resolution
                        #print('TEMP VAL IS' + str(temp_val))
                        if temp_val != float('inf'):
                            x_grid_map = int(x_map/self.grid_map.info.resolution)
                            y_grid_map = int(y_map/self.grid_map.info.resolution)

                            if x_grid_map >= 0 and x_grid_map <= self.grid_map.info.height :
                                if y_grid_map >= 0 and y_grid_map <= self.grid_map.info.width : 
                                    if x_grid_map+y_grid_map*self.grid_map.info.width < self.grid_map.info.width * self.grid_map.info.height:
                                        if self.grid_map.data[x_grid_map+y_grid_map*self.grid_map.info.width] == 100:
                                            count_good = count_good + 1

                #self.particle_list[k].weight = count_good / len(scan.ranges)
                self.particles[k,3] = count_good / len(scan.ranges)

        self.particles[range(0,self.M),3] = self.particles[range(0,self.M),3]/(np.sum(self.particles[range(0,self.M),3]))

        #print('PARTICLE 1 weight is: ' + str(self.particles[0,3]))
        #print("sum", np.sum(self.particles[range(0,self.M),3]))
        #print(self.particles[range(0,self.M),3])
    
    
    def resampling(self):
        # TODO: check why np.sum(self.particles[range(0,self.M),3]) sometimes is NaN 
        if np.isnan(self.particles).any():
            print("NaN detected, skip resampling for this step")
            return

        temp_particles = self.particles[range(0,self.M),:]

        cdf = np.cumsum(temp_particles[:,3])
        if cdf[-1] < 1.0:
            print("cdf too small:", cdf[-1])
            cdf = cdf + (1.0001-cdf[-1])/self.M
            print("cdf corrected:", cdf[-1])

        if cdf[-1] < 1:
            print("CDF NOT 1!:", cdf[-1])
            pass    
        else:
            for i in range(self.M):
                temp_rand = np.random.rand()
                temp_ind = np.where(cdf > temp_rand) 
                ind = temp_ind[0][0]
                self.particles[i,:] = temp_particles[ind,:]
                self.particles[i,3] = 1.0/self.M
            print("RESAMPLE DONE")

                
    def pub_particle_cloud(self):
        particle_cloud = PoseArray()
        particle_cloud.header.stamp = rospy.get_rostime()
        particle_cloud.header.frame_id = "odom"

        for i in range(self.M):
            temp_pose = Pose()
            temp_pose.position.x = self.particles[i,0]
            temp_pose.position.y = self.particles[i,1]
            [temp_pose.orientation.x, temp_pose.orientation.y, temp_pose.orientation.z, temp_pose.orientation.w]  = tf.transformations.quaternion_from_euler(0, 0, self.particles[i,2])
            particle_cloud.poses.append(temp_pose)
        
        self.pub_pc.publish(particle_cloud)
        #self.particle_cloud = []

        # publish particle consensus ##########################################################
        self.state_filter.x = np.mean(self.particles[range(0,self.M),0])
        self.state_filter.y = np.mean(self.particles[range(0,self.M),1])
        self.state_filter.theta = np.mean(self.particles[range(0,self.M),2])
        #print(self.state_filter)

        self.state_filter_odom.pose.pose.position.x = self.state_filter.x
        self.state_filter_odom.pose.pose.position.y = self.state_filter.y

        quaternion = tf.transformations.quaternion_from_euler(0,0,self.state_filter.theta)
        if not np.any(np.isnan(quaternion)):
            self.state_filter_odom.pose.pose.orientation.z = quaternion[2]
            self.state_filter_odom.pose.pose.orientation.w = quaternion[3]
            self.state_filter_odom.header.stamp = rospy.get_rostime()
            self.state_filter_odom.header.frame_id = "odom"
            self.state_filter_odom.child_frame_id = "base_link"
            self.pub_filter.publish(self.state_filter_odom)
        

''' MAIN '''
def main():
    rospy.init_node('mcl_py_node', anonymous=True)

    # create an instance of class MCL_py
    mcl_obj = MCL_py() 
    
    rate = rospy.Rate(mcl_obj.control_frequency)

    # Subscribe to Left Encoder 
    rospy.Subscriber('/left_motor/encoder', phidgets.msg.motor_encoder, mcl_obj.update_feedback_enc_left)

    # Subscribe to Right Encoder 
    rospy.Subscriber('/right_motor/encoder', phidgets.msg.motor_encoder, mcl_obj.update_feedback_enc_right)

    # Subscribe to Reset Flag 
    rospy.Subscriber('/odom_reset', std_msgs.msg.Bool, mcl_obj.reset_feedback)

    # Subscribe to raw laser scan
    rospy.Subscriber('/scan', LaserScan, mcl_obj.callback_scan)
    
    # Subscribe to grid map
    rospy.Subscriber('/maze_map_node/map', OccupancyGrid, mcl_obj.callback_grid_map)
    
    #rospy.spinOnce()

    # Reset particle_list with known prior
    # In this particular case, subscriber callbacks have not yet been called, so everything initializes to 0
    #if mcl_obj.received_odom == 1 and mcl_obj.received_scan == 1 and mcl_obj.received_map == 1:  
    
    #mcl_obj.reset_particles()
    e = 0
    while not rospy.is_shutdown():
        e = e + 1
        
        # Calculate robot odometry
        mcl_obj.calc_and_pub_odom()

        # Prediction step
        mcl_obj.prediction_step()
        
        # resample 10/2 times a second
        if e % 3 == 0:
            # ONLY update and resample if the robot is moving
            if mcl_obj.has_moved:
                mcl_obj.has_moved = False

                # Transform scans and update weights
                mcl_obj.weight_update()
                
                # Resampling step
                mcl_obj.resampling()     

            # Publish particle cloud
            mcl_obj.pub_particle_cloud()        
            e = 0
        rate.sleep()

        
if __name__ == '__main__':
    main()
