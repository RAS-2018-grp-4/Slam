#!/usr/bin/env python3

"""
    # Viktor Tuul
    # 950530-2613
    # tuul@kth.se
"""

# Python standard library
from math import cos, sin, atan2, fabs, sqrt

# Numpy
import numpy as np

# "Local version" of ROS messages
from local.geometry_msgs import PoseStamped, Quaternion
from local.sensor_msgs import LaserScan
from local.map_msgs import OccupancyGridUpdate

from grid_map import GridMap


class Mapping:
    def __init__(self, unknown_space, free_space, c_space, occupied_space,
                 radius, optional=None):
        self.unknown_space = unknown_space
        self.free_space = free_space
        self.c_space = c_space
        self.occupied_space = occupied_space
        self.allowed_values_in_map = {"self.unknown_space": self.unknown_space,
                                      "self.free_space": self.free_space,
                                      "self.c_space": self.c_space,
                                      "self.occupied_space": self.occupied_space}
        self.radius = radius
        self.__optional = optional

    def get_yaw(self, q):
        """Returns the Euler yaw from a quaternion.
        :type q: Quaternion
        """
        return atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))

    def raytrace(self, start, end):
        """Returns all cells in the grid map that has been traversed
        from start to end, including start and excluding end.
        start = (x, y) grid map index
        end = (x, y) grid map index
        """
        (start_x, start_y) = start
        (end_x, end_y) = end
        x = start_x
        y = start_y
        (dx, dy) = (fabs(end_x - start_x), fabs(end_y - start_y))
        n = dx + dy
        x_inc = 1
        if end_x <= start_x:
            x_inc = -1
        y_inc = 1
        if end_y <= start_y:
            y_inc = -1
        error = dx - dy
        dx *= 2
        dy *= 2

        traversed = []
        for i in range(0, int(n)):
            traversed.append((int(x), int(y)))

            if error > 0:
                x += x_inc
                error -= dy
            else:
                if error == 0:
                    traversed.append((int(x + x_inc), int(y)))
                y += y_inc
                error += dx

        return traversed

    def add_to_map(self, grid_map, x, y, value):
        """Adds value to index (x, y) in grid_map if index is in bounds.
        Returns weather (x, y) is inside grid_map or not.
        """
        if value not in self.allowed_values_in_map.values():
            raise Exception("{0} is not an allowed value to be added to the map. "
                            .format(value) + "Allowed values are: {0}. "
                            .format(self.allowed_values_in_map.keys()) +
                            "Which can be found in the '__init__' function.")

        if self.is_in_bounds(grid_map, x, y):
            grid_map[x, y] = value
            return True
        return False

    def is_in_bounds(self, grid_map, x, y):
        """Returns weather (x, y) is inside grid_map or not."""
        if x >= 0 and x < grid_map.get_width():
            if y >= 0 and y < grid_map.get_height():
                return True
        return False

    def update_map(self, grid_map, pose, scan):
        """Updates the grid_map with the data from the laser scan and the pose.
        
        For E: 
            Update the grid_map with self.occupied_space.

            Return the updated grid_map.

            You should use:
                self.occupied_space  # For occupied space

                You can use the function add_to_map to be sure that you add
                values correctly to the map.

                You can use the function is_in_bounds to check if a coordinate
                is inside the map.

        For C:
            Update the grid_map with self.occupied_space and self.free_space. Use
            the raytracing function found in this file to calculate free space.

            You should also fill in the update (OccupancyGridUpdate()) found at
            the bottom of this function. It should contain only the rectangle area
            of the grid_map which has been updated.

            Return both the updated grid_map and the update.

            You should use:
                self.occupied_space  # For occupied space
                self.free_space      # For free space

                To calculate the free space you should use the raytracing function
                found in this file.

                You can use the function add_to_map to be sure that you add
                values correctly to the map.

                You can use the function is_in_bounds to check if a coordinate
                is inside the map.

        :type grid_map: GridMap
        :type pose: PoseStamped
        :type scan: LaserScan
        """

        # Current yaw of the robot
        robot_yaw = self.get_yaw(pose.pose.orientation)
        # The origin of the map [m, m, rad]. This is the real-world pose of the
        # cell (0,0) in the map.
        origin = grid_map.get_origin()
        # The map resolution [m/cell]
        resolution = grid_map.get_resolution()

        """
        Fill in your solution here
        """

        # get the current robot position
        robot_x = pose.pose.position.x
        robot_y = pose.pose.position.y

        # scan ranges and corresponding bearings
        scan_ranges = scan.ranges
        n_scans = len(scan_ranges)
        scan_bearings = [scan.angle_min + i*scan.angle_increment for i in range(n_scans)]

        # pre-allocate scan positions (map frame)
        x_scan = []
        y_scan = []

        for i in range(n_scans):
            # disregard infinity range, or those which are too close or far away
            if (scan_ranges[i] == float("inf")) or (scan_ranges[i] <= scan.range_min) or (scan_ranges[i] >= scan.range_max):
                continue
            else:
                # acquire the map frame coordinates of the radar scans
                x_scan.append(robot_x + cos(scan_bearings[i] + robot_yaw)*scan_ranges[i])
                y_scan.append(robot_y + sin(scan_bearings[i] + robot_yaw)*scan_ranges[i])

        # update the number of (valid) scans
        n_scans = len(x_scan)

        # convert the map coordinates to map indicies
        origin_x = origin.position.x
        origin_y = origin.position.y

        # define robot position in the grid
        idx_robot_x = int((robot_x - origin_x)/resolution)
        idx_robot_y = int((robot_y - origin_y)/resolution)
        start = (idx_robot_x, idx_robot_y)

        # initalize max and min coordinates of ray
        min_x, min_y = grid_map.get_width(), grid_map.get_width()
        max_x, max_y = 0, 0

        occupied_new = []

        for i in range(n_scans):
            idx_scan_x = int((x_scan[i] - origin_x)/resolution)
            idx_scan_y = int((y_scan[i] - origin_y)/resolution)

            if self.is_in_bounds(grid_map, idx_scan_x, idx_scan_y):
                if (grid_map[idx_scan_x, idx_scan_y] != self.occupied_space):
                    min_x, min_y = min(min_x, idx_scan_x), min(min_y, idx_scan_y)
                    max_x, max_y = max(max_x, idx_scan_x), max(max_y, idx_scan_y)

                # define scan end point as occupied space
                self.add_to_map(grid_map, idx_scan_x, idx_scan_y, self.occupied_space)
                occupied_new.append((idx_scan_x, idx_scan_y))
   
                # define the free space
                end = (idx_scan_x, idx_scan_y)
                ray = self.raytrace(start, end)
           
                for (free_x, free_y) in ray:
                    # define the points on the ray as free spac                    
                    if (grid_map[free_x, free_y] != self.free_space) and (grid_map[free_x, free_y] != self.c_space) and not (free_x, free_y) in occupied_new:
                        self.add_to_map(grid_map, free_x, free_y, self.free_space)
                        min_x, min_y = min(min_x, free_x), min(min_y, free_y)
                        max_x, max_y = max(max_x, free_x), max(max_y, free_y)
                
        """
        For C only!
        Fill in the update correctly below.
        """
      
        # Only get the part that has been updated
        update = OccupancyGridUpdate()    
        # The minimum x index in 'grid_map' that has been updated
        update.x = min_x
        # The minimum y index in 'grid_map' that has been updated
        update.y = min_y
        # Maximum x index - minimum x index + 1
        update.width = max_x - min_x + 1
        # Maximum y index - minimum y index + 1
        update.height = max_y - min_y + 1
        # The map data inside the rectangle, in row-major order.

        map_state = []
        for y in range(update.y, update.y + update.height):
            for x in range(update.x, update.x + update.width):
                map_state.append(grid_map[x,y])
        update.data = map_state

        # Return the updated map together with only the
        # part of the map that has been updated
        
        return grid_map, update

    def inflate_map(self, grid_map):
        """For C only!
        Inflate the map with self.c_space assuming the robot
        has a radius of self.radius.
        
        Returns the inflated grid_map.

        Inflating the grid_map means that for each self.occupied_space
        you calculate and fill in self.c_space. Make sure to not overwrite
        something that you do not want to.


        You should use:
            self.c_space  # For C space (inflated space).
            self.radius   # To know how much to inflate.

            You can use the function add_to_map to be sure that you add
            values correctly to the map.

            You can use the function is_in_bounds to check if a coordinate
            is inside the map.

        :type grid_map: GridMap
        """

        """
        Fill in your solution here
        """
        

        for x in range(grid_map.get_width()):
            for y in range(grid_map.get_height()):
                
                # if the cell is occupied_space, begin the fill
                if grid_map[x,y] == self.occupied_space:    

                    # scan a square around the point with side length 2*self.radius
                     for i in range(-self.radius, self.radius+1):
                        for j in range(-self.radius, self.radius+1):
                            inflate_x = x + i
                            inflate_y = y + j

                            # make sure that the point is within the disk of radius self.radius
                            if sqrt((inflate_x - x)**2 + (inflate_y - y)**2) <= self.radius:
                                cell_state = grid_map[inflate_x,inflate_y]

                                # make sure that the cell we want to fill in isn't occupied (or already c_space)
                                if (cell_state != self.occupied_space) and (cell_state != self.c_space): 
                                    self.add_to_map(grid_map, inflate_x, inflate_y, self.c_space)

        # Return the inflated map
        return grid_map

