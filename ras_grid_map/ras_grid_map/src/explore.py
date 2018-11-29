#!/usr/bin/env python

import os
import rospy
import std_msgs.msg
import sensor_msgs.msg
import nav_msgs.msg 
import geometry_msgs.msg
import math
import tf

from random import sample
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
#####################################################
#                 Explore Class                     #
#####################################################
class Explore:
    #####################################################
    #              Initialize Object                    #
    #####################################################
    def __init__(self):



        self.initial_target = [[0.25,2.2],[2.2,0.2],[2.1,2.1],[1.1,1.1]]
        self.next_y = 0.0
        self.next_y = 0.0
        self.seq_initial_target = 0
        self.number_initial_target = 4
        self.flag_path_execution = True                 # True when path following done
 
        self.explored_map = []
        self.ras_map = []
        self.map_height = 0.0
        self.map_width = 0.0
        self.map_resolution = 0.0
        self.unexplored_area = ()
        self.num_unexplored = 0
        self.temp_list = []
        self.largest_area = []
        self.update_map = False
        self.size_area = 0
        self.max_size = 0
    #####################################################
    #             Initialize ROS Parameter              #
    #####################################################
        rospy.init_node('explore_node', anonymous=True)

        self.pub_next_goal = rospy.Publisher('/next_traget', PoseStamped, queue_size=1)
        rospy.Subscriber('/maze_map_node/map', OccupancyGrid, self.callback_map)
        # rospy.Subscriber('/maze_map_node/map_explored', OccupancyGrid, self.callback_map)
        rospy.Subscriber("/flag_done", String, self.callback_flag)
        self.rate = rospy.Rate(10)
        self.listenser = tf.TransformListener()
        

    #####################################################
    #                   Map_Callback                  #
    #####################################################
    def callback_map(self,map):
        self.map_height = map.info.height
        self.map_width = map.info.width
        self.map_resolution = map.info.resolution
        self.ras_map = map.data
        self.update_map = True

    #####################################################
    #                   Flag_Callback                   #
    #####################################################
    def callback_flag(self,flag):
        if flag.data:
            self.flag_path_execution = True
        else:
            pass

    #####################################################
    #                 Publisher Function                #
    #####################################################
    def send_position_message(self, target_position):
        rospy.sleep(2)
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = target_position[0]
        pose.pose.position.y = target_position[1]
        pose.pose.position.z = 0
        (r, p, y, w) = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
        pose.pose.orientation.z = y
        pose.pose.orientation.w = w
        self.pub_next_goal.publish(pose)
        rospy.sleep(2)


    def check_boundary(self,x,y):
        if x < 0:
            return False
        if x >= self.map_height:
            return False
        if y < 0:
            return False
        if y >= self.map_width:
            return False
        if self.explored_map [x+y*self.map_width] != 0:
            return False
        return True


    def bfs(self,x,y):
        q = []
        q.append([x,y])

        while len(q) >0:
            [cur_x, cur_y] = q.pop()
            self.size_area = self.size_area + 1
            self.explored_map[cur_x+cur_y*self.map_width] = 50
            self.temp_list.append([cur_x,cur_y])
            

            temp_x = cur_x + 1 
            temp_y = cur_y
            if self.check_boundary(temp_x,temp_y):
                q.append([temp_x,temp_y])

            temp_x = cur_x - 1 
            temp_y = cur_y
            if self.check_boundary(temp_x,temp_y):
                q.append([temp_x,temp_y])

            temp_x = cur_x
            temp_y = cur_y - 1
            if self.check_boundary(temp_x,temp_y):
                q.append([temp_x,temp_y])

            temp_x = cur_x
            temp_y = cur_y + 1
            if self.check_boundary(temp_x,temp_y):
                q.append([temp_x,temp_y])



    #####################################################
    #                 Analyize Map                      #
    #####################################################
    def analyize_map(self):
        self.update_map = False
        while not self.update_map:
            pass

        self.explored_map = self.ras_map
        self.explored_map = list(self.explored_map)
        #print(self.explored_map)

        flag_found = False

        for i in range(0,self.map_height-1):
            for j in range(0,self.map_width-1):
                #print(i+j*self.map_width)
                if self.explored_map[i+j*self.map_width] == 0:
                    
                    self.size_area = 0
                    self.bfs(i,j)
                    if self.size_area > 10:
                        self.num_unexplored = self.num_unexplored + 1
                        print("new area")
                        print(self.temp_list)
                        self.next_x = i * self.map_resolution
                        self.next_y = j * self.map_resolution
                        flag_found = True
                        break
                    del self.temp_list [:]
                else:
                    pass
            # if flag_found:
            #     break
              
    #####################################################
    #                 Find Next Target                  #
    #####################################################

    def find_next_targe(self):
        self.update_map = False
        while not self.update_map:
            pass

        self.explored_map = self.ras_map
        self.explored_map = list(self.explored_map)
        #print(self.explored_map)

        flag_found = False
        self.max_size = -1
        for i in range(0,self.map_height-1):
            for j in range(0,self.map_width-1):
                #print(i+j*self.map_width)
                if self.explored_map[i+j*self.map_width] == 0:
                    
                    self.size_area = 0
                    self.bfs(i,j)
                    if self.size_area > 10:
                        self.num_unexplored = self.num_unexplored + 1
                        print("new area")
                        # print(self.temp_list)
                        if self.max_size < self.size_area:
                            
                            self.max_size = self.size_area
                            self.largest_area = self.temp_list
                            

                        # self.next_x = i * self.map_resolution
                        # self.next_y = j * self.map_resolution
                    self.temp_list =[]
                else:
                    pass
        [sample_x,sample_y] = sample(list(self.largest_area),1)
        self.next_x = sample_x * self.map_resolution
        self.next_y = sample_y * self.map_resolution
        #print(sample(list(self.largest_area),20))
    #####################################################
    #                   Main_Loop                       #
    #####################################################
    def loop(self):
        rospy.loginfo('Explore Start')
       

        while not rospy.is_shutdown():
            
            if self.flag_path_execution:
                self.flag_path_execution = False
                if self.seq_initial_target < self.number_initial_target:
                    self.next_x = self.initial_target[self.seq_initial_target][0]
                    self.next_y = self.initial_target[self.seq_initial_target][1]
                    self.seq_initial_target = self.seq_initial_target + 1
                else:
                    rospy.loginfo('Finding Next Target')
                    self.find_next_targe()
                    #print(self.num_unexplored)
                    self.num_unexplored = 0
                self.send_position_message([self.next_x, self.next_y])  
                rospy.loginfo('Publish Next Target : %.2lf %.2lf',self.next_x,
                                                                  self.next_y)

            self.rate.sleep()

    
#####################################################
#                Main Function                      #
#####################################################
if __name__ == "__main__":
    try:
        EX = Explore()
        EX.loop()
    except rospy.ROSInterruptException:
        pass