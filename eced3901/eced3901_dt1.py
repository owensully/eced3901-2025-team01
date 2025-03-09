import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from nav_msgs.msg import Odometry

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist

from sensor_msgs.msg import LaserScan

from collections import deque

from math import pi
from time import sleep

import numpy as np
import os

"""
 ========== DEMO CLASS DEFINITIONS ==========
"""

class LaserDataInterface(object):

    def __init__(self, storage_depth=4, logger=None):
        self.laser_data = deque()
        self.depth = storage_depth
        self.logger = logger

    def get_logger(self):
        if self.logger:
            return self.logger

    def get_range_array(self, center_deg, left_offset_deg=-5, right_offset_deg=+5, invalid_data=None):

        if center_deg < -180 or center_deg > 180:
            raise ValueError("Invalid range: %d"%center_deg)

        if left_offset_deg < -180 or left_offset_deg > 0:
            raise ValueError("Invalid left offset: %d"%left_offset_deg)

        if right_offset_deg > 180 or right_offset_deg < 0:
            raise ValueError("Invalid right offset: %d"%right_offset_deg)

        # No data yet
        if len(self.laser_data) == 0:
            return None

        angleset_deg = (self.anglestep * 180.0) / 3.14159
        offset_i = round(center_deg / angleset_deg)
        offset_pos = round(right_offset_deg / angleset_deg)
        offset_neg = round(left_offset_deg / angleset_deg)

        #Get absolute values, which may be negative!
        start = offset_i + offset_neg
        end = offset_i + offset_pos
        
        # Remap to -180 to 0 range
        if start > 180:
            start = start - 360

        if end > 180:
            end = end - 360

        # Remap to 183 to 0 range
        if start < -180:
            start = start + 360
        
        if end < -180:
            end = end + 360

        tempdata = self.laser_data[0]
        data = list(map(lambda x: None if x < self.minrange or x > self.maxrange else x, tempdata))
        
        return data
        
    def process_laser_msg(self, msg):
        self.anglestep = msg.angle_increment
        self.minrange = msg.range_min
        self.maxrange = msg.range_max
        ranges = msg.ranges[0:]

        # Index 0 is front of robot & counts clockwise from there
        num_points = len(ranges)

        self.laser_data.append(msg.ranges)

        # Get rid of old data
        if len(self.laser_data) > self.depth:
            self.laser_data.popleft()

"""
 ========== DEMO FUNCTION DEFINITIONS ==========
"""

"""
 - highpassFilter()
    function used to apply a high-pass filter
    to the lidar data. the low distance values
    must be filtered to apply accurate data processing.
"""
def highpassFilter(value):

    if value < cutoff_distance: # filter low distance values

        return 0

    else: # pass-band for all other values

        return value

"""
 - NoneFilter()
    function used to apply a filter that
    removes 'None' values from the lidar data.
    the 'None' values are generated from lidar
    distances that are too large, and must be
    filtered to apply accurate data processing.
"""
def NoneFilter(value):

    if value is None: # filter 'None' values

        return 0

    else: # pass-band for all non-'None' values

        return value

"""
 - min_lidar_distance()
    function used to find the minimum value in the lidar array,
    using the high-pass filter to avoid false readings.
"""
def min_lidar_distance(data):

    data_remove_none = [x for x in data if NoneFilter(x)] # apply NoneFilter()

    data_highpass = [x for x in data_remove_none if highpassFilter(x)] # apply highpassFilter()

    x = min(data_highpass) # find minimum values after filtering

    return x

"""
 ========== FINAL CLASS DEFINITIONS ==========
"""

class NavigateCourse(Node):

    def __init__(self):
        
        super().__init__('navigate_course')

        self.cell = start_cell

        self.attribute_check_complete = 0

        self.lidar_tolerance = 0.02

        # Subscribe to odometry
        self.sub_odom = self.create_subscription(
            Odometry, #Matches the import at the top (nav_msgs.msg import Odometry)
            'odom', #odom topic
            self.odom_callback, #Call this function
            10) #Queue size 10

        # Publish to velocity
        self.pub_vel = self.create_publisher(
            Twist, #Expects the twist message format
            'cmd_vel',
            10) #Queue size 10

        # Subscribe to lidar - this requires a policy, otherwise you won't see any data
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

        self.sub_lidar = self.create_subscription(
            LaserScan,
            'scan',
            self.range_callback,
            qos_policy
        )

        self.ldi = LaserDataInterface(logger=self.get_logger())
    
        # 10 Hz timer 0.1
        self.timer = self.create_timer(2, self.timer_callback)

    def timer_callback(self):
        """Timer callback for 10Hz control loop"""

        self.primaryProcess()
        self.lidar_test() 

    def odom_callback(self, msg):
        """Callback on 'odom' subscription"""
 
        self.y_now = msg.pose.pose.position.y# - self.odom_start

    def range_callback(self, msg):
        """Callback on 'range' subscription"""
        #self.get_logger().info('Scan Data: "%s"' % msg)
        #self.get_logger().info('Scan Data: "%s"' % msg.angle_increment)

        anglestep = msg.angle_increment
        self.minrange = msg.range_min
        maxrange = msg.range_max
        ranges = msg.ranges[1:]

        self.laser_range = max(ranges[0:5])
        #self.get_logger().info('Scan Data: "%d"' % len(ranges))

        self.ldi.process_laser_msg(msg)

    def destroy_node(self):
        msg = Twist()
        self.pub_vel.publish(msg)
        super().destroy_node()

    """
     - lidar_test()
        function used to test lidar functionality
        and view range array.
    """
    def lidar_test(self):

        lr = self.ldi.get_range_array(0.0)
        
        if lr is None:
            return

        i = [1, 95, 190, 285]

        lr2 = [lr[j] for j in i]

        self.get_logger().info(str(lr2))

    """
     - pullLidar()
        function used to select and return values from
        lidar range array.
    """
    def pullLidar(self, *args):

        # read 380 lidar values
        laser_ranges = self.ldi.get_range_array(0.0)

        # select pull values from range array
        pull_values = [laser_ranges[arg] for arg in args if 0 <= arg < 380]
        
        return pull_values
    """
     - primaryProcess()
        function repeated by the timer callback
        which directs the primary process of the
        robot.
    """
    def primaryProcess(self):

        # determine if the attributes have been checked
        if not self.attribute_check_complete:

            checkCell(self.cell)

        # determine the current NWSE lidar values
        NWSE_lidar = self.pullLidar(D["NORTH"], D["WEST"], D["SOUTH"], D["EAST"])

        # ensure proper values were returned
        if not NWSE_lidar:

            return

        # compare current NWSE lidar values with neighbour lidar values
        new_cell = checkNN(self.cell, NWSE_lidar, nearest_neighbour)

        if new_cell:

            self.cell = new_cell

            self.attribute_check_complete = 0


"""
 ========== FINAL FUNCTION DEFINITIONS ==========
"""

"""
 - read_map()
    function used to load map based on the specified
    layers in the 'layers' file.
"""
def read_map():
    
    with open(map_directory + "/layers", "r") as file:

        map_layers = file.read().split()

    return np.array([np.loadtxt(map_directory + filename) for filename in map_layers]), map_layers

"""
 - checkLidar()
    function used to compare two lidar array
    within a set tolerance.
"""
def checkLidar(lidar1, lidar2): 

    for l1, l2 in zip(lidar1, lidar2):
        
        if l1 is None or l2 is None:

            return False

        if abs(l1 - l2) > self.lidar_tolerance:

            return False

    return True


"""
 - checkNN()
    function used to determine the next cell location
    by comparing current lidar values to the lidar
    values stored in the nearest neighbours.
"""
def checkNN(cell, lidar, directions):

    # loop through provided directions
    for direction in directions:

        # find the neighbour in each direction
        new_cell = list(map(lambda i, j: i + j, cell, direction))
            
        # query the lidar values for the new cell
        new_cell_lidar = [course_map[map_layers.index(d)][new_cell[0]][new_cell[1]] for d in NWSE]

        # determine if current lidar values match map lidar values
        if checkLidar(lidar, new_cell_lidar):

            # update current cell
            return new_cell

    return False

"""
 - checkCell()
    function used to check the cell attributes
    at given coordinates.
"""
def checkCell(cell):

    for attribute in attribute_priority:

        if attribute in map_layers:

            if course_map[map_layers.index(attribute)][cell[0]][cell[1]]:

                subProcess[attribute_priority.index(attribute)]()
    
    self.attribute_check_complete = 1

subProcess = [lootProcess, magnetProcess, rfidProcess, qrProcess, safeProcess, indianaProcess, cageProcess]

def lootProcess():

    pass

def magnetProcess():

    pass

def rfidProcess():

    pass

def qrProcess():

    pass

def safeProcess():

    pass

def indianaProcess():

    pass

def cageProcess():

    pass

"""
 ========== FINAL GLOBAL VARIABLES ==========
"""

"""
 read map from sub-directory.
 map/ must sit parallel to eced3901_dt1.py.
 map/ must contain a 'layers' file, with
 a list of map layers.
"""
# current_directory = os.getcwd()
current_directory = "/home/student/ros2_ws/src/eced3901/eced3901" 

map_directory = current_directory + "/map/"

course_map, map_layers = read_map()

"""
 enum to align cardinal directions
 with 380 element LIDAR array.
"""
D = {
    "NORTH": 1,
    "NORTH-WEST": 47,
    "WEST": 95,
    "SOUTH-WEST": 142,
    "SOUTH": 190,
    "SOUTH-EAST": 237,
    "EAST": 285,
    "NORTH-EAST": 332
}
#NWSE = [ "NORTH", "WEST", "SOUTH", "EAST" ]
NWSE = [ "north", "west", "south", "east" ]

"""
 direction tables to check surrounding cells
"""
nearest_neighbour = [ [0, -1], [-1, 0], [0, 1], [1, 0] ]
next_nearest_neighbour = [ [-1, -1], [1, -1], [1, 1], [-1, 1] ]

start_cell = [8, 9]

attribute_priority = [ "loot", "magnet", "rfid", "qr", "safe", "indiana", "cage"]

"""
 ========== MAIN ==========
"""

def main(args=None):
    
    rclpy.init(args=args)

    navigate_course = NavigateCourse()

    rclpy.spin(navigate_course)

    navigate_square.destroy_node()

    rclpy.shutdown()
    
if __name__ == '__main__':

    main()
