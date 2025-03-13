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

import cv2

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

        self.lidar_tolerance = 0.1

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
        self.test()
        #self.primaryProcess()
        #self.lidar_test() 

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

    def test(self):

        print(directions)

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

        # ensure proper values were returned
        if not NWSE_lidar:

            return

        # compare current NWSE lidar values with neighbour lidar values
        new_cell = checkNN(self.cell, nearest_neighbour)

        # if new cell is found, update current cell
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
 - read_directions()
    function used to load the direction table file
    into a 2D array.
"""
def read_directions():

    with open(current_directory + "/dir_table", "r") as file:

        return np.array(np.loadtxt(file))

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
def checkNN(cell, directions):

    # loop through provided directions
    for direction in directions:

        # determine the current lidar values
        lidar = self.pullLidar(cell_directions(angles(course_map[map_layers.index("direction")][new_cell[0]][new_cell[1]])))

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
 - cell_directions()
    function used to convert NWSE to align with a given
    cell direction.
"""
def cell_directions(d):

    return (A["NORTH"] + d) % 380, (A["WEST"] + d) % 380, (A["SOUTH"] + d) % 380, (A["EAST"] + d) % 380

"""
 - checkCell()
    function used to check the cell attributes
    at given coordinates.
"""
def checkCell(cell):

    # all possible attributes
    for attribute in attribute_priority:

        # if attribute exists in current map
        if attribute in map_layers:

            # if this cell has this attribute
            if course_map[map_layers.index(attribute)][cell[0]][cell[1]]:

                # call corresponding subprocess
                subProcess[attribute_priority.index(attribute)]()
    
    # toggle attribute check flag
    self.attribute_check_complete = 1

#subProcess = [lootProcess, magnetProcess, rfidProcess, qrProcess, safeProcess, indianaProcess, cageProcess]

def lootProcess():

    pass

def magnetProcess():

    pass

def rfidProcess():

    pass

def qrProcess():
 
    # open camera
    cam = cv2.VideoCapture(0)
    
    # 1280 x 720
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    cv2.waitKey(1000)

    # take picture
    ret, pic = cam.read()

    if ret:

        # determine destination path
        dst = pic_dst + "qr.jpg"

        # save picture at destination
        cv2.imwrite(dst, pic)
        
    # close camera
    cam.release()

    # read picture
    qr_pic = cv2.imread(dst)

    # initialize QR detector
    qr_detector = cv2.QRCodeDetector()
    
    # decode QR
    qr_data, bbox, _ = qr_detector.detectAndDecode(qr_pic)

    # place holder print, this is where the qr data is passed through serial
    print(qr_data)

def safeProcess():

    pass

def indianaProcess():

    pass

def cageProcess():

    pass

"""
 ========== FINAL GLOBAL VARIABLES ==========
"""

spin_velocity = 0.5

start_cell = [8, 9]

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

pic_dst = "/home/student/ros2_ws/src/eced3901/eced3901/pictures/"

"""
 enum to align directions
 with 380 element LIDAR array.
"""
angles = {
    0: 1,
    1: 47,
    2: 95,
    3: 142,
    4: 190,
    5: 237,
    6: 285,
    7: 332
}

"""
 enum to correspond the cardinal
 directions to the 380 element LIDAR
 array.
"""
A = {
    "NORTH": 1,
    "WEST": 95,
    "SOUTH": 190,
    "EAST": 285
}

"""
 list of NWSE file names.
"""
NWSE = [ "north", "west", "south", "east" ]

"""
 direction tables to check surrounding cells.
"""
nearest_neighbour = [ [0, -1], [-1, 0], [0, 1], [1, 0] ]
next_nearest_neighbour = [ [-1, -1], [1, -1], [1, 1], [-1, 1] ]

"""
 read dir_table from working directory.
"""
direction_map = read_directions()

"""
 corresponding velocity values for a
 given direction.
"""
decode_direction_map = {
    0: [forward_velocity, 0.0],
    1: [0.0, -spin_velocity],
    2: [0.0, spin_velocity]
}

# decode_direction_map[direction_map[self.old_dir][self.new_dir]]

D = {
    "N/A": -1,
    "NORTH": 0,
    "NORTH-WEST": 1,
    "WEST": 2,
    "SOUTH-WEST": 3,
    "SOUTH": 4,
    "SOUTH-EAST": 5,
    "EAST": 6,
    "NORTH-EAST", 7
}

"""
 list of possible file names that form the map,
 logged in their respective priority.
"""
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
