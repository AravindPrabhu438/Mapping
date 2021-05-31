#!/usr/bin/env python
""" Simple occupancy-grid-based mapping without localization. 

Subscribed topics:
/scan

Published topics:
/map 
/map_metadata

"""
import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

import numpy as np

class Map(object):
    """ 
    The Map class stores an occupancy grid as a two dimensional
    numpy array. 
    
    Public instance variables:

        width      --  Number of columns in the occupancy grid.
        height     --  Number of rows in the occupancy grid.
        resolution --  Width of each grid square in meters. 
        origin_x   --  Position of the grid cell (0,0) in 
        origin_y   --    in the map coordinate system.
        grid       --  numpy array with height rows and width columns.
        
    
    Note that x increases with increasing column number and y increases
    with increasing row number. 
    """

    def __init__(self, origin_x=-5, origin_y=-5, resolution=.1, 
                 width=100, height=100):
        """ Construct an empty occupancy grid.
        
        Arguments: origin_x, 
                   origin_y  -- The position of grid cell (0,0) in the
                                map coordinate frame.
                   resolution-- width and height of the grid cells 
                                in meters.
                   width, 
                   height    -- The grid will have height rows and width
                                columns cells.  width is the size of
                                the x-dimension and height is the size
                                of the y-dimension.
                                
         The default arguments put (0,0) in the center of the grid. 
                                
        """
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.resolution = resolution
        self.width = width 
        self.height = height 
        self.grid = np.zeros((height, width))

    def to_message(self):
        """ Return a nav_msgs/OccupancyGrid representation of this map. """
     
        grid_msg = OccupancyGrid()

        # Set up the header.
        grid_msg.header.stamp = rospy.Time.now()
        grid_msg.header.frame_id = "map"

        # .info is a nav_msgs/MapMetaData message. 
        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.width
        grid_msg.info.height = self.height
        
        # Rotated maps are not supported... quaternion represents no
        # rotation. 
        grid_msg.info.origin = Pose(Point(self.origin_x, self.origin_y, 0),
                               Quaternion(0, 0, 0, 1))

        # Flatten the numpy array into a list of integers from 0-100.
        # This assumes that the grid entries are probalities in the
        # range 0-1. This code will need to be modified if the grid
        # entries are given a different interpretation (like
        # log-odds).
        

        flat_grid = self.grid.reshape((self.grid.size,)) * 100
        #print('-----------')
        
        #print (flat_grid)
        flat_grid=flat_grid.astype('int8')
      
        ##########


        grid_msg.data = list(np.round(flat_grid))
       

        return grid_msg

    def set_cell(self, x, y, val):
        """ Set the value of a cell in the grid. 

        Arguments: 
            x, y  - This is a point in the map coordinate frame.
            val   - This is the value that should be assigned to the
                    grid cell that contains (x,y).

        This would probably be a helpful method!  Feel free to throw out
        point that land outside of the grid. 
        """
        pass

class Mapper(object):
    """ 
    The Mapper class creates a map from laser scan data.
    """
    
    def __init__(self):
        """ Start the mapper. """

        rospy.init_node('mapper')
        self._map = Map()

        # Setting the queue_size to 1 will prevent the subscriber from
        # buffering scan messages.  This is important because the
        # callback is likely to be too slow to keep up with the scan
        # messages. If we buffer those messages we will fall behind
        # and end up processing really old scans.  Better to just drop
        # old scans and always work with the most recent available.
        rospy.Subscriber('scan',
                         LaserScan, self.scan_callback, queue_size=1)
        rospy.Subscriber('odom',
                         Odometry, self.my_odom_call, queue_size=1)

        # Latched publishers are used for slow changing topics like
        # maps.  Data will sit on the topic until someone reads it. 
        self._map_pub = rospy.Publisher('map', OccupancyGrid, latch=True)
        self._map_data_pub = rospy.Publisher('map_metadata', 
                                             MapMetaData, latch=True)
        
        rospy.spin()

    def my_odom_call(self,msg):
        global roll, pitch, yaw
        global pos

        my_orientation = msg.pose.pose.orientation
        orientation_list = [my_orientation.x, my_orientation.y, my_orientation.z, my_orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        # print("yaw",yaw)
        pos = msg.pose.pose.position
        # print("pos", pos)
        # print("pos.x", pos.x)
        # print("pos.y", pos.y)
    
    # def get_indix(self, x, y):
    #     x=x-self._map.origin_x
    #     y=y-self._map.origin_y
    #     i=int(round(x/self._map.resolution))
    #     j=int(round(y/self._map.resolution))
    #     return i, j


    def scan_callback(self, scan):
        """ Update the map on every scan callback. """
        # print('--------------------------------')
        # print ('the length of the range array is: ')
        scanMax = 360
        scanMin = 0
        robot_radius = 0.6
        r = scan.ranges
        x = [None] * len(r)
        y = [None] * len(r)
        # print(scan.ranges[90])
        # print("scan_value", r)

        (x_r, y_r, yaw_r) = [pos.x, pos.y, yaw]
        # print("xr, yr, yaw", x_r, y_r, yaw_r)
        # print("xr =", x_r)
        # print("yr =", y_r)
        # print("yaw =", yaw_r)
        for i in range(len(r)):
            if r[i] != np.Inf and r[i] != np.Inf:
                x[i] = math.cos(math.radians(i))*(r[i] + robot_radius)
                y[i] = math.sin(math.radians(i))*(r[i] + robot_radius)
                # print("x_s", x)
                # print("y_s", y)
                x1 = (math.cos(yaw_r)*x[i]) + (-math.sin(yaw_r) * y[i])
                y1 = (math.sin(yaw_r)*x[i]) + (math.cos(yaw_r) * y[i])
                # print("X1 = ", x1)
                # print("Y1 = ", y1)
                x[i] = (x1 + x_r)
                y[i] = (y1 + y_r)
                # print("X1 = ", x)
                # print("Y1 = ", y)
                # print("pos_x =", x[i],"pos_y =", y[i])
                x2 =x[i]-self._map.origin_x
                y2 =y[i]-self._map.origin_y
                Xs=int(round(x2/self._map.resolution))
                Ys=int(round(y2/self._map.resolution))
                print("Xs ", Xs)
                print("Ys", Ys)
                if scan.ranges[i] > scanMin and scan.ranges[i] < scanMax:
                    self._map.grid[Xs, Ys] = .9

             
            
        

        # Now that the map wass updated, so publish it!
        rospy.loginfo("Scan is processed, publishing updated map.")
        self.publish_map()

    

 


    def publish_map(self):
        """ Publish the map. """
        grid_msg = self._map.to_message()
        self._map_data_pub.publish(grid_msg.info)
        self._map_pub.publish(grid_msg)


if __name__ == '__main__':
    try:
        m = Mapper()
    except rospy.ROSInterruptException:
        pass

"""
https://w3.cs.jmu.edu/spragunr/CS354_S14/labs/mapping/mapper.py
https://www.youtube.com/watch?v=K1ZFkR4YsRQ
"""