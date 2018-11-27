#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"

#include <sstream>
#include <vector>

typedef std::tuple<int, int> tuple2;

// specify map dimensions
const int map_height = 3;                                 // [m]
const int map_width = 3;                                  // [m]
const float map_resolution = 0.05;                        // [m]
const int n_height = (float)map_height/map_resolution;    // [1]
const int n_width = (float)map_width/map_resolution;      // [1]

class GridMap 
{ 
    public: 
     // matrix representation
    int map_m[n_height][n_width] = {};  

    // vector representation                 
    std::vector<signed char> map_v = std::vector<signed char>(n_height*n_width);    
  
    void add_to_map(int x, int y, int value) 
    { 
        if (is_in_bounds(x,y) == true)
        {
            map_m[x][y] = value;
            map_v[y + x*n_width] = map_m[x][y];
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
    ros::init(argc, argv, "ras_grid_map_node");
    ros::NodeHandle n;

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
    map.add_ray(20,20,67,67);
    map.add_ray(100,40,30,40);
    map.add_ray(2,2,10,50);

    while (ros::ok())
    {
        // publish the map
        grid.data = map.map_v;
        map_pub.publish(grid);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
