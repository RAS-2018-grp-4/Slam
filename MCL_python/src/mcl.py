#!/usr/bin/env python
'''
Monte carlo localization
Source: https://github.com/pjensfelt/matlab_loc_demos/blob/master/MCL.m

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
    - particle can be a class?? member variables could be:
        - x
        - y
        - theta
        - laser scan for it
        - resampling weight 
    
'''
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan

class MCL_py():

    def __init__(self):
        # DESIGN variables
        self.M = 100            # No. of particles
        
        # other variables
        self.robot_odom = Odometry()
        self.robot_scan = LaserScan()
        self.particle_list = []
        self.grid_map = OccupancyGrid()

    def callback_odom(odom_msg):
        self.robot_odom = odom_msg

    def callback_scan(scan_msg):
        self.laser_scan = scan_msg

    def callback_grid_map(map_msg):
        self.grid_map = map_msg

    def reset_particles(reset_flag):
        # Erase previous particle list      ### WARNING: do we really delete prev particles?
        self.particle_list = []

        if reset_flag == 0:     #prior known, set all particles acc to present odom
            for i in range(self.M):
                self.particle_list.append(particle(self.robot_odom, self.robot_scan))


class particle():

    def __init__(self, given_pose, given_laser_scan):
        self.particle_odom = given_pose
        # self.y = 0.0
        # self.theta = 0.0
        self.weight = 0.0
        self.particle_scan = given_laser_scan 


''' MAIN '''
def main():
    rospy.init_node('mcl_py_node', anonymous=True)

    # create an instance of class MCL_py
    mcl_obj = MCL_py() 
    
    # Subscribe to Odometry 
    rospy.Subscriber('/odom', Odometry, mcl_obj.callback_odom)

    # Subscribe to raw laser scan
    rospy.Subscriber('/scan', Odometry, mcl_obj.callback_scan)
    
    # Subscribe to grid map
    rospy.Subscriber('/grid_map', OccupancyGrid, mcl_obj.callback_grid_map)

    # Reset particle_list with known prior
    mcl_obj.reset_particles(0)     

if __name__ == '__main__':
    main()