// ROS includes.
#include <ros/ros.h>
#include <ros/time.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

#include <tf/transform_listener.h>
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
#include "nav_msgs/Odometry.h"
#include "nav_msgs/MapMetaData.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include <sstream>
#include <vector>


using namespace std;

typedef std::tuple<int, int> tuple2;

// specify map dimensions
const int map_height = 3;                                 // [m]
const int map_width = 3;                                  // [m]
const float map_resolution = 0.03;                        // [m]
const int n_height = (float)map_height/map_resolution;    // [1]
const int n_width = (float)map_width/map_resolution;      // [1]

double x,y;

class GridMap 
{ 
    public: 
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

    void inflate_map()
    {
        int radius = 5;
        int inflate_x = 0;
        int inflate_y = 0;
        int cell_state = 0;

        for (int x = 0; x < n_width; x++)
        {
            for (int y = 0; y < n_height; y++)
            {  
                // if the cell is occupied_space, begin the fill
                if (map_m[y][x] == 100)
                {    
                    // scan a square around the point with side length 2*self.radius
                     for (int i = -radius; i < radius+1; i++)
                     {
                        for (int j = -radius; j < radius+1; j++)
                        {
                            inflate_x = x + i;
                            inflate_y = y + j;

                            // make sure that the point is within the disk of radius self.radius
                            if (sqrt(pow(inflate_x - x, 2) + pow(inflate_y - y, 2)) <= radius)
                            {
                                cell_state = map_m[inflate_y][inflate_x];

                                // make sure that the cell we want to fill in isn't occupied (or already c_space)
                                if ((cell_state != 100) && (cell_state != -2))
                                {
                                    add_to_map(inflate_x, inflate_y, -2); 
                                } 
                            }   
                        }                      
                    }                                               
                }
            }
        }
    }

}; 

GridMap ras_map;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    tf::TransformListener listener(ros::Duration(10));
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    ras_map.add_to_map((int)(x/map_resolution),(int)(y/map_resolution),50);
}


int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "discovery_node");
    ros::NodeHandle n("~");
    ros::Rate r(10);
    
    // from ras_grid_map-------------------------------------
    // initialize publisher
    ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>("discovery", 1000);
    ros::Subscriber sub = n.subscribe("/odom_filter", 1, odomCallback);
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
        ras_map.add_ray((int)(x1/map_resolution),(int)(y1/map_resolution),(int)(x2/map_resolution),(int)(y2/map_resolution));
       
    }
    //ROS_INFO_STREAM("Read "<<wall_id<<" walls from map file.");

    // inflate the map
    ras_map.inflate_map();

    // Main loop.
    while (n.ok())
    {
        ROS_INFO_STREAM("!hello!");

        // publish high walls
        //vis_pub.publish(ras_map.all_markers);

        // publish the grid map
        grid.data = ras_map.map_v;
        map_pub.publish(grid);

        ros::spinOnce();
        r.sleep();
    }

    return 0;
} 
