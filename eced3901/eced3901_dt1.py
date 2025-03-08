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
    Function used to apply a high-pass filter
    to the lidar data. The low distance values
    must be filtered to apply accurate data processing.
"""
def highpassFilter(value):

    if value < cutoff_distance: # filter low distance values

        return 0

    else: # pass-band for all other values

        return value

"""
 - NoneFilter()
    Function used to apply a filter that
    removes 'None' values from the lidar data.
    The 'None' values are generated from lidar
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
    Function used to find the minimum value in the lidar array,
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

    def lidar_test(self):

        lr = self.ldi.get_range_array(0.0)

        if lr is None:
            return

        self.get_logger().info(str(lr))


"""
 ========== FINAL FUNCTION DEFINITIONS ==========
"""

def read_map():
    
    with open(map_directory + "/layers", "r") as file:

        map_layers = file.read().split()

    return np.array([np.loadtxt(map_directory + filename) for filename in map_layers])

"""
 ========== FINAL GLOBAL VARIABLES ==========
"""

current_directory = os.getcwd()
map_directory = current_directory + "/map/"

course_map = read_map()

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
