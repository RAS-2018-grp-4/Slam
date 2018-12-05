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
from nav_msgs.msg import Odometry
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



        #self.initial_target = [[0.23,1.9],[2.1,2.1],[2.0,0.2],[0.78,0.3],[1.1,1.1]]
        self.initial_target = [[rospy.get_param("/t2x"), rospy.get_param("/t2y")],
                                [rospy.get_param("/t3x"), rospy.get_param("/t3y")],
                                [rospy.get_param("/t4x"), rospy.get_param("/t4y")],
                                [rospy.get_param("/t5x"), rospy.get_param("/t5y")],
                                [rospy.get_param("/t6x"), rospy.get_param("/t6y")]]
        
        self.next_y = 0.0
        self.next_y = 0.0
        self.seq_initial_target = 0
        self.number_initial_target = 0
        self.flag_path_execution = False                 # True when path following done
        self.flag_path_not_found = False

        self.flag_found_new_target = True
        self.explored_map = []
        self.ras_map = []
        self.original_map = []
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
        self.origin_x = 0
        self.origin_y = 0
        
        self.pass_num = 0
        self.selected_area = []
        self.too_far_area = []
        self.min_dis=0.0
        self.closest_area = []
        self.selected_one = 0
        self.odom_x = 0.0
        self.odom_y = 0.0
    #####################################################
    #             Initialize ROS Parameter              #
    #####################################################
        rospy.init_node('explore_node', anonymous=True)

        self.pub_next_goal = rospy.Publisher('/next_traget', PoseStamped, queue_size=1)
        rospy.Subscriber('/maze_map_node/map', OccupancyGrid, self.callback_map)
        rospy.Subscriber('/maze_map_node/map_explored', OccupancyGrid, self.callback_map_explore)
        rospy.Subscriber("/flag_done", String, self.callback_flag)
        rospy.Subscriber("/flag_pathplanner", String, self.callback_flag_path_planner)
        rospy.Subscriber("/robot_filter", Odometry, self.callback_odom)

        self.rate = rospy.Rate(10)
        self.listenser = tf.TransformListener()
        

    #####################################################
    #            Map_Explore_Callback                  #
    #####################################################
    def callback_map_explore(self,map):
        self.map_height = map.info.height
        self.map_width = map.info.width
        self.map_resolution = map.info.resolution
        self.ras_map = map.data
        self.origin_x = map.info.origin.position.x
        self.origin_y = map.info.origin.position.y
        self.update_map = True


    #####################################################
    #                   Map_Callback                  #
    #####################################################
    def callback_map(self,map):
        self.original_map = map.data


    #####################################################
    #                  Odom_Callback                    #
    #####################################################
    def callback_odom(self,odom):
        self.odom_x = odom.pose.pose.position.x
        self.odom_y = odom.pose.pose.position.y
    #####################################################
    #                   Flag_Callback                   #
    #####################################################
    def callback_flag(self,flag):
        if flag.data:
            self.flag_path_execution = True
        else:
            pass
    #####################################################
    #           Flag_path_planner_Callback              #
    #####################################################
    def callback_flag_path_planner(self,flag):
        if flag.data == "NO_PATH_FOUND":
            self.flag_path_not_found = True
        else:
            pass
    #####################################################
    #                 Publisher Function                #
    #####################################################
    def send_position_message(self, target_position):
        rospy.sleep(1)
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
        if x >= self.map_height - 1:
            return False
        if y < 0:
            return False
        if y >= self.map_width - 1:
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
            if self.check_boundary(temp_x,temp_y) and (self.explored_map [temp_x+temp_y*self.map_width] == 0):
                q.append([temp_x,temp_y])

            temp_x = cur_x - 1 
            temp_y = cur_y
            if self.check_boundary(temp_x,temp_y) and (self.explored_map [temp_x+temp_y*self.map_width] == 0):
                q.append([temp_x,temp_y])

            temp_x = cur_x
            temp_y = cur_y - 1
            if self.check_boundary(temp_x,temp_y) and (self.explored_map [temp_x+temp_y*self.map_width] == 0):
                q.append([temp_x,temp_y])

            temp_x = cur_x
            temp_y = cur_y + 1
            if self.check_boundary(temp_x,temp_y) and (self.explored_map [temp_x+temp_y*self.map_width] == 0):
                q.append([temp_x,temp_y])



    #####################################################
    #                Check Availalbe                    #
    #####################################################
    def check_availalbe(self,x,y):
        q = []
        q.append([x,y,0])
        cur_x = 0
        cur_y = 0
        while len(q) >0:
            [cur_x, cur_y, depth] = q.pop()
            
            cur_x_map = (cur_x* self.map_resolution - self.origin_x)
            cur_y_map = (cur_y* self.map_resolution - self.origin_y)          
            if depth == 5:
                continue
            elif (self.original_map[cur_x+cur_y*self.map_width] == -40 or self.original_map[cur_x+cur_y*self.map_width] == 0) and (((cur_x_map - self.odom_x)**2 + (cur_y_map - self.odom_y)**2) > 0.16):
            #elif (self.original_map[cur_x+cur_y*self.map_width] == -40 or self.original_map[cur_x+cur_y*self.map_width] == -20 or self.original_map[cur_x+cur_y*self.map_width] == 0) and (((cur_x_map - self.odom_x)**2 + (cur_y_map - self.odom_y)**2) > 0.16):
                return [True, cur_x,cur_y]
            
            temp_x = cur_x + 1 
            temp_y = cur_y
            temp_depth = depth + 1
            if self.check_boundary(temp_x,temp_y) and self.original_map[temp_x+temp_y*self.map_width] != 100:
                q.append([temp_x,temp_y,temp_depth])

            temp_x = cur_x - 1 
            temp_y = cur_y
            if self.check_boundary(temp_x,temp_y) and self.original_map[temp_x+temp_y*self.map_width] != 100:
                q.append([temp_x,temp_y,temp_depth])

            temp_x = cur_x
            temp_y = cur_y - 1
            if self.check_boundary(temp_x,temp_y) and self.original_map[temp_x+temp_y*self.map_width] != 100:
                q.append([temp_x,temp_y,temp_depth])

            temp_x = cur_x
            temp_y = cur_y + 1
            if self.check_boundary(temp_x,temp_y) and self.original_map[temp_x+temp_y*self.map_width] != 100:
                q.append([temp_x,temp_y,temp_depth])
        return [False,cur_x, cur_y]



    #####################################################
    #               Calculate Distance                  #
    #####################################################
    def calculate_distance(self,area):
        sum_x = 0
        sum_y = 0
        for i in range(0,len(area)):
            sum_x = sum_x + area[i][0]
            sum_y = sum_y + area[i][1]
        sum_x = (sum_x/len(area)* self.map_resolution - self.origin_x)
        sum_y = (sum_y/len(area)* self.map_resolution - self.origin_y)
        return (sum_x - self.odom_x)**2 + (sum_y - self.odom_y)**2
        

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
        num_area = 0
        self.min_dis = 100000000
        for i in range(0,self.map_height-1):
            for j in range(0,self.map_width-1):
                #print(i+j*self.map_width)
                if self.explored_map[i+j*self.map_width] == 0:
                    self.temp_list =[]
                    self.size_area = 0
                    self.bfs(i,j)
                    if (self.size_area > 10) :
                        if self.too_far_area.count(num_area) == 0:
                            if self.max_size < self.size_area:
                                self.max_size = self.size_area
                                self.selected_area = self.temp_list
                                self.selected_one = num_area
                            # dis = self.calculate_distance(self.temp_list)
                            # if dis < self.min_dis:
                            #     self.selected_one = num_area
                            #     self.min_dis= dis
                            #     self.selected_area = self.temp_list
                        # if self.pass_num ==  num_area:
                        #        self.selected_area = self.temp_list
                        #        flag_found = True
                        #        break
                        num_area = num_area + 1
                        # print(self.pass_num,num_area)
                        # self.next_x = i * self.map_resolution
                        # self.next_y = j * self.map_resolution
                    self.temp_list =[]
                else:
                    pass
            # if flag_found:
            #     break
        #[sample_x,sample_y] = sample(list(self.largest_area),1)
        rospy.loginfo("Found %d areas",num_area)
        #print(self.min_dis)
        num_smaple = min(len(self.selected_area),20)
        sample_point = sample(list(self.selected_area),num_smaple)
        available = False
        for i in range(0,num_smaple):
            print(sample_point[i][0],sample_point[i][1],self.original_map[sample_point[i][0]+sample_point[i][1]*self.map_width])
            [available, sample_point_x,sample_point_y] = self.check_availalbe(sample_point[i][0],sample_point[i][1])
            print(available,sample_point_x,sample_point_y,self.original_map[sample_point_x+sample_point_y*self.map_width])
            if available:
                rospy.loginfo("Found New Target")
                self.flag_found_new_target= True
                self.next_x =  sample_point_x * self.map_resolution - self.origin_x
                self.next_y =  sample_point_y * self.map_resolution - self.origin_y
                break
        #print(sample(list(self.largest_area),20))
    #####################################################
    #                   Main_Loop                       #
    #####################################################
    def loop(self):
        rospy.loginfo('Explore Start')
       

        while not rospy.is_shutdown():
            
            if self.flag_path_execution or not self.flag_found_new_target:
                self.flag_path_execution = False
                if self.flag_found_new_target:
                    self.too_far_area = []
                if self.seq_initial_target < self.number_initial_target:
                    self.next_x = self.initial_target[self.seq_initial_target][0] - self.origin_x
                    self.next_y = self.initial_target[self.seq_initial_target][1] - self.origin_y
                    self.seq_initial_target = self.seq_initial_target + 1
                    self.flag_found_new_target = True
                else:
                    rospy.loginfo('Finding Next Target....')
                    self.flag_found_new_target = False
                    self.find_next_targe()
                    self.flag_path_not_found = False
                    #print(self.num_unexplored)
                    self.num_unexplored = 0

                if self.flag_found_new_target:
                    self.send_position_message([self.next_x, self.next_y])  
                    rospy.loginfo('Publish Next Target : %.2lf %.2lf',self.next_x,
                                                                      self.next_y)
                else:
                    rospy.loginfo('Cannot find new target')
                    self.too_far_area.append(self.selected_one)

                    self.pass_num = self.pass_num + 1
            elif self.flag_path_not_found :

                rospy.loginfo('Abort Path')
                self.too_far_area.append(self.selected_one)
                print(self.too_far_area)
                self.pass_num = self.pass_num + 1
                # rospy.loginfo('Finding Next Target')
                self.flag_found_new_target = False
                # self.find_next_targe()
                self.flag_path_not_found = False
                #     #print(self.num_unexplored)
                # self.num_unexplored = 0
                # if self.flag_found_new_target:
                #     self.send_position_message([self.next_x, self.next_y])  
                #     rospy.loginfo('Publish Next Target : %.2lf %.2lf',self.next_x,
                #                                                   self.next_y)
                # else:
                #     rospy.loginfo('Cannot find new target')
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