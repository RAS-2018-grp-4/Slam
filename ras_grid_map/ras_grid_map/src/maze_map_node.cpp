/*
 *  world_node.cpp
 *
 *
 *  Created on: Sept 18, 2014
 *  Authors:   Rares Ambrus
 *            raambrus <at> kth.se
 */

/* Copyright (c) 2015, Rares Ambrus, CVAP, KTH
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of KTH nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL KTH BE LIABLE FOR ANY
   DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


// ROS includes.
#include <ros/ros.h>
#include <ros/time.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <tf/tf.h>

// Boost includes
#include <stdio.h>
#include <stdlib.h>

// std includes
#include <limits>
#include <iostream>
#include <fstream>

// from ras_grid_map
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include <sstream>
#include <vector>

using namespace std;

typedef std::tuple<int, int> tuple2;

// specify map dimensions
const int map_height = 7;                                 // [m]
const int map_width = 7;                                  // [m]
const float map_resolution = 0.05;                        // [m]
const int n_height = (float)map_height/map_resolution;    // [1]
const int n_width = (float)map_width/map_resolution;      // [1]





class GridMap 
{ 
    public: 

    visualization_msgs::MarkerArray all_markers;
    visualization_msgs::Marker wall_marker;
    int wall_id = 0;


     // matrix representation
    int map_m[n_height][n_width] = {};  

    // vector representation                 
    std::vector<signed char> map_v = std::vector<signed char>(n_height*n_width);    
  
    void add_to_map(int x, int y, int value) 
    { 
        if (is_in_bounds(x,y) == true)
        {
            map_m[y][x] = value;
            map_v[x + y*n_width] = map_m[y][x];
            //ROS_INFO("insert %d", map_m[x][y]);
        }
    } 

    bool is_in_bounds(int x, int y)
    {
        if (x >= 0 && x < n_width)
        {
            if (y >= 0 &&  y < n_height)
            {
                return true;
            }            
        }
        return false;
    }

    void add_ray(int x1, int y1, int x2, int y2)
    {
        tuple2 start = tuple2(x1, y1);
        tuple2 end = tuple2(x2, y2);
        std::vector<tuple2> ray = raytrace(start, end);
        
        for (int i = 0; i < ray.size(); i++){
            int x = std::get<0>(ray[i]);
            int y = std::get<1>(ray[i]);
            add_to_map(x,y,100);

            // create high walls
            float x0 = std::get<0>(ray[i])*map_resolution;
            float y0 = std::get<1>(ray[i])*map_resolution;

            // angle and distance
            double angle = atan2(y0-y0,x0-x0);
            double dist = sqrt(pow(x0-x0,2) + pow(y0-y0,2));

            // set pose
            wall_marker.pose.position.x = (x0+x0)/2 + 0.025;
            wall_marker.pose.position.y = (y0+y0)/2 + 0.025;
            //wall_marker.text=line_stream.str();
            tf::Quaternion quat; quat.setRPY(0.0,0.0,angle);
            tf::quaternionTFToMsg(quat, wall_marker.pose.orientation);

            // add to array
            wall_marker.id = wall_id;
            all_markers.markers.push_back(wall_marker);
            wall_id++;            
        }
    }

    std::vector<tuple2> raytrace(tuple2 start, tuple2 end)
    {
        int start_x = std::get<0>(start);
        int start_y = std::get<1>(start);
        int end_x = std::get<0>(end);
        int end_y = std::get<1>(end);

        int x = start_x;
        int y = start_y;

        float dx = fabs(end_x - start_x);
        float dy = fabs(end_y - start_y);
        float n = dx + dy;

        int x_inc = 1;
        if (end_x <= start_x) x_inc = -1;

        int y_inc = 1;
        if (end_y <= start_y) y_inc = -1;
        
        float error = dx - dy;
        dx = dx*2;
        dy = dy*2;

        std::vector<tuple2> traversed;

        for (int i = 0; i < int(n); i++)
        {
            tuple2 vek = tuple2(int(x), int(y));
            traversed.push_back(vek);

            if (error > 0)
            {
                x += x_inc;
                error -= dy;
            }
            else
            {
                if (error == 0)
                {
                    tuple2 vek = tuple2(int(x + x_inc), int(y));
                    traversed.push_back(vek);
                }
                y += y_inc;
                error += dx;
            }
        }
        traversed.push_back(end);
        return traversed;
    }
}; 

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "maze_map_node");
    ros::NodeHandle n("~");
    ros::Rate r(10);



    // from ras_grid_map-------------------------------------
    // initialize publisher
    ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>("map", 1000);

    // loop rate frequency
    ros::Rate loop_rate(1);

    // initalize objects for map pose
    geometry_msgs::Pose origin_pose;
    geometry_msgs::Point origin_location;
    geometry_msgs::Quaternion origin_orientation;
    origin_pose.position = origin_location;
    origin_pose.orientation = origin_orientation;

    // initialze the occupancy grid object
    nav_msgs::OccupancyGrid grid;                   
    grid.info.resolution = map_resolution;
    grid.info.height = n_height;
    grid.info.width = n_width;
    grid.info.origin = origin_pose;

    // GridMap object
    GridMap map; 

    ROS_INFO("Cells height: %d", n_height);
    ROS_INFO("Cells width: %d", n_width);

    // create some rays
    //map.add_ray(20,20,67,67);
    
    //map.add_ray(100,40,30,40);
    //map.add_ray(2,2,10,50);
    // ras_grid_map end ------------------------------------------.


    string _map_file;
    string _map_frame = "/map";
    string _map_topic = "/maze_map";
    n.param<string>("map_file", _map_file, "maze_map.txt");
    n.param<string>("map_frame", _map_frame, "/map");
    n.param<string>("map_topic", _map_topic, "/maze_map");

    ROS_INFO_STREAM("Loading the maze map from " << _map_file);
    ROS_INFO_STREAM("The maze map will be published in frame " << _map_frame);
    ROS_INFO_STREAM("The maze map will be published on topic " << _map_topic);

    ifstream map_fs; map_fs.open(_map_file.c_str());
    if (!map_fs.is_open()){
        ROS_ERROR_STREAM("Could not read maze map from "<<_map_file<<". Please double check that the file exists. Aborting.");
        return -1;
    }

    ros::Publisher vis_pub = n.advertise<visualization_msgs::MarkerArray>( _map_topic, 0 );
    //visualization_msgs::MarkerArray all_markers;
    //visualization_msgs::Marker wall_marker;
    map.wall_marker.header.frame_id = _map_frame;
    map.wall_marker.header.stamp = ros::Time();
    map.wall_marker.ns = "world";
    map.wall_marker.type = visualization_msgs::Marker::CUBE;
    map.wall_marker.action = visualization_msgs::Marker::ADD;
    map.wall_marker.scale.x = 0.05;
    map.wall_marker.scale.y = 0.05;
    map.wall_marker.scale.z = 0.2;
    map.wall_marker.color.a = 1.0;
    map.wall_marker.color.r = (255.0/255.0);
    map.wall_marker.color.g = (0.0/255.0);
    map.wall_marker.color.b = (0.0/255.0);
    map.wall_marker.pose.position.z = 0.1;

    string line;
    int wall_id = 0;
    while (getline(map_fs, line)){

        if (line[0] == '#') {
            // comment -> skip
            continue;
        }

        double max_num = std::numeric_limits<double>::max();
        double x1= max_num,
               x2= max_num,
               y1= max_num,
               y2= max_num;

        std::istringstream line_stream(line);

        line_stream >> x1 >> y1 >> x2 >> y2;

        if ((x1 == max_num) || ( x2 == max_num) || (y1 == max_num) || (y2 == max_num)){
            ROS_WARN("Segment error. Skipping line: %s",line.c_str());
        }

        // add ray --------------------------------------------------
        map.add_ray((int)(x1/map_resolution),(int)(y1/map_resolution),(int)(x2/map_resolution),(int)(y2/map_resolution));

        // angle and distance
        /*
        double angle = atan2(y2-y1,x2-x1);
        double dist = sqrt(pow(x1-x2,2) + pow(y1-y2,2));

        // set pose
        wall_marker.scale.x = std::max(0.01,dist);
        wall_marker.pose.position.x = (x1+x2)/2;
        wall_marker.pose.position.y = (y1+y2)/2;
        wall_marker.text=line_stream.str();
        tf::Quaternion quat; quat.setRPY(0.0,0.0,angle);
        tf::quaternionTFToMsg(quat, wall_marker.pose.orientation);

        // add to array
        wall_marker.id = wall_id;
        all_markers.markers.push_back(wall_marker);
        wall_id++;
        */
    }
    //ROS_INFO_STREAM("Read "<<wall_id<<" walls from map file.");


    // Main loop.
    while (n.ok())
    {
        ROS_INFO_STREAM("hello");

        // publish high walls
        vis_pub.publish(map.all_markers);

        // publish the grid map
        grid.data = map.map_v;
        map_pub.publish(grid);

        ros::spinOnce();
        r.sleep();
    }

    return 0;
} 
