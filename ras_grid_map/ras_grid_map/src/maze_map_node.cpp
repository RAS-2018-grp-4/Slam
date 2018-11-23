
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
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include <sstream>
#include <vector>

using namespace std;

typedef std::tuple<int, int> tuple2;
float x;
float y;

class GridMap 
{ 
    public: 

    visualization_msgs::MarkerArray all_markers;
    visualization_msgs::Marker wall_marker;
    int wall_id = 0;
    int n_height = 0;
    int n_width = 0;
    float map_resolution = 0;
    float min_x = 0;
    float min_y = 0;

     // matrix representation
    //int map_m[][];  

    // vector representation                 
    std::vector<signed char> map_v = std::vector<signed char>();    

    void set_map_settings(float map_width, float map_height, float temp_map_resolution, float m_x, float m_y)
    {
        n_height = ceil(map_height/temp_map_resolution);    // [1]
        n_width = ceil(map_width/temp_map_resolution);      // [1]
        map_resolution = temp_map_resolution;
        map_v = std::vector<signed char>(n_height*n_width);  
        min_x = m_x;
        min_y = m_y;
    }

    void add_to_map(int x, int y, int value, string flag) 
    { 
        if (is_in_bounds(x,y) == true)
        {
            //map_m[y][x] = value;
            //map_v[x + y*n_width] = map_m[y][x];
            map_v[x + y*n_width] = value;
            
            if (value == 100){
                inflate_map_local(x, y, flag);
            }
            
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
            add_to_map(x,y,100, "");

            // create high walls
            float x0 = std::get<0>(ray[i])*map_resolution + min_x;
            float y0 = std::get<1>(ray[i])*map_resolution + min_y;

            // angle and distance
            double angle = atan2(y0-y0,x0-x0);
            double dist = sqrt(pow(x0-x0,2) + pow(y0-y0,2));

            // set pose
            wall_marker.pose.position.x = x0 + map_resolution/2;
            wall_marker.pose.position.y = y0 + map_resolution/2;
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

    void inflate_map_local(int x, int y, string flag){
        int radius = 5;
        if (flag == "added_wall") radius = 4;
        int inflate_x = 0;
        int inflate_y = 0;
        int cell_state = 0;

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
                    cell_state = map_v[inflate_x + inflate_y*n_width];

                    // make sure that the cell we want to fill in isn't occupied (or already c_space)
                    if ((cell_state != 100) && (cell_state != -2))
                    {
                        add_to_map(inflate_x, inflate_y, -2, ""); 
                    } 
                }   
            }                      
        }   
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
                if (map_v[x + y*n_width] == 100)
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
                                cell_state = map_v[inflate_x + inflate_y*n_width];

                                // make sure that the cell we want to fill in isn't occupied (or already c_space)
                                if ((cell_state != 100) && (cell_state != -2))
                                {
                                    add_to_map(inflate_x, inflate_y, -2, ""); 
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

void wallCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    int x_int = 0;
    int y_int = 0;
    int count = 0;
  for (int i = 0; i < msg->poses.size(); i++)
  {
    x_int = (int)((msg->poses[i].position.x-ras_map.min_x)/ras_map.map_resolution);
    y_int = (int)((msg->poses[i].position.y-ras_map.min_y)/ras_map.map_resolution);

    if(ras_map.is_in_bounds(x_int,y_int) == false)
    {
        continue;
    }


    if (ras_map.map_v[x_int+y_int*ras_map.n_width] == 0 || ras_map.map_v[x_int+y_int*ras_map.n_width] == -2)
    {
        count ++;
    }
    else
    {
        if (count > 15)
        {
            for(int j = i; i > i -count; j--)
            {
                ras_map.add_to_map(x_int,y_int,100,"added_wall");
            }

        }
        count = 0;
    }

    
  }
}


int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "maze_map_node");
    ros::NodeHandle n("~");
    ros::Rate r(10);
    
    // from ras_grid_map-------------------------------------
    // initialize publisher
    ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>("map", 1000);
    ros::Subscriber sub = n.subscribe("/wall_position", 1, wallCallback);
    // loop rate frequency
    ros::Rate loop_rate(1);

    string _map_file;
    string _map_frame = "/map";
    string _map_topic = "/maze_map";
    n.param<string>("map_file", _map_file, "maze_map.txt");
    n.param<string>("map_frame", _map_frame, "/map");
    n.param<string>("map_topic", _map_topic, "/maze_map");

    ROS_INFO_STREAM("Loading the maze map from " << _map_file);
    ROS_INFO_STREAM("The maze map will be published in frame " << _map_frame);
    ROS_INFO_STREAM("The maze map will be published on topic " << _map_topic);

    ifstream map_fs1; map_fs1.open(_map_file.c_str());
    if (!map_fs1.is_open()){
        ROS_ERROR_STREAM("Could not read maze map from "<<_map_file<<". Please double check that the file exists. Aborting.");
        return -1;
    }

    // GET MAP SPECIFICATIONS
    float x_min = 9999;
    float y_min = 9999;
    float x_max = -9999;
    float y_max = -9999;
    float width = 0;
    float height = 0;

  
    string line_;
    while (getline(map_fs1, line_)){
        if (line_[0] == '#') {
            // comment -> skip
            continue;
        }

        double max_num = std::numeric_limits<double>::max();
        double x1= max_num,
               x2= max_num,
               y1= max_num,
               y2= max_num;

        std::istringstream line_stream(line_);
        line_stream >> x1 >> y1 >> x2 >> y2;
        if ((x1 == max_num) || ( x2 == max_num) || (y1 == max_num) || (y2 == max_num)){
            ROS_WARN("Segment error. Skipping line: %s",line_.c_str());
        }

        if (x1 < x_min) x_min = x1;
        if (x2 < x_min) x_min = x2;
        if (y1 < y_min) y_min = y1;
        if (y2 < y_min) y_min = y2;
        if (x1 > x_max) x_max = x1;
        if (x2 > x_max) x_max = x2;
        if (y1 > y_max) y_max = y1;
        if (y2 > y_max) y_max = y2;    
    }

    width = float(x_max - x_min);
    height = float(y_max - y_min);
   
    //x_min = -0.45;
    ras_map.set_map_settings(width,height,0.03,x_min,y_min);
    //ras_map.set_map_settings(5,5,0.03,0,0);

    // initalize objects for map pose
    geometry_msgs::Pose origin_pose;
    geometry_msgs::Point origin_location;
    geometry_msgs::Quaternion origin_orientation;
    origin_pose.position = origin_location;
    origin_pose.position.x = x_min;
    origin_pose.position.y = y_min;
    origin_pose.orientation = origin_orientation;

    // initialze the occupancy grid object
    nav_msgs::OccupancyGrid grid;                   
    grid.info.resolution = ras_map.map_resolution;
    grid.info.height = ras_map.n_height;
    grid.info.width = ras_map.n_width;
    grid.info.origin = origin_pose;

    // GridMap object
    ROS_INFO("Cells height: %d", ras_map.n_height);
    ROS_INFO("Cells width: %d", ras_map.n_width);


    ros::Publisher vis_pub = n.advertise<visualization_msgs::MarkerArray>( _map_topic, 0 );
    //visualization_msgs::MarkerArray all_markers;
    //visualization_msgs::Marker wall_marker;
    ras_map.wall_marker.header.frame_id = _map_frame;
    ras_map.wall_marker.header.stamp = ros::Time();
    ras_map.wall_marker.ns = "world";
    ras_map.wall_marker.type = visualization_msgs::Marker::CUBE;
    ras_map.wall_marker.action = visualization_msgs::Marker::ADD;
    ras_map.wall_marker.scale.x = 0.03;
    ras_map.wall_marker.scale.y = 0.03;
    ras_map.wall_marker.scale.z = 0.2;
    ras_map.wall_marker.color.a = 1.0;
    ras_map.wall_marker.color.r = (0.0/255.0);
    ras_map.wall_marker.color.g = (255.0/255.0);
    ras_map.wall_marker.color.b = (0.0/255.0);
    ras_map.wall_marker.pose.position.z = 0.1;


    ifstream map_fs2; map_fs2.open(_map_file.c_str());
    if (!map_fs2.is_open()){
        ROS_ERROR_STREAM("Could not read maze map from "<<_map_file<<". Please double check that the file exists. Aborting.");
        return -1;
    }

    string line;
    //int wall_id = 0;
    while (getline(map_fs2, line)){

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
        ras_map.add_ray((int)((x1-x_min)/ras_map.map_resolution),(int)((y1-y_min)/ras_map.map_resolution),(int)((x2-x_min)/ras_map.map_resolution),(int)((y2-y_min)/ras_map.map_resolution));
    }
    //ROS_INFO_STREAM("Read "<<wall_id<<" walls from map file.");

    // inflate the map
    //as_map.inflate_map();

    // Main loop.
    while (n.ok())
    {
        ROS_INFO_STREAM("!hello!");

        // publish high walls
        vis_pub.publish(ras_map.all_markers);

        // publish the grid map
        grid.data = ras_map.map_v;
        map_pub.publish(grid);

        ros::spinOnce();
        r.sleep();
    }

    return 0;
} 
