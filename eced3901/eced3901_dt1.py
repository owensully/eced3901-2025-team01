# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from nav_msgs.msg import Odometry

# http://docs.ros.org/en/noetic/api/geometry_msgs/html/index-msg.html
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist

from sensor_msgs.msg import LaserScan

from collections import deque

from math import pi
from time import sleep

dir_table = [['LEFT', 'RIGHT'], ['RIGHT', 'LEFT']] 

class LaserDataInterface(object):

    def __init__(self, storage_depth=4, logger=None):
        self.laser_data = deque()
        self.depth = storage_depth
        self.logger = logger

    def get_logger(self):
        if self.logger:
            return self.logger

    def get_range_array(self, center_deg, left_offset_deg=-5, right_offset_deg=+5, invalid_data=None):
        """
        This function should return an array of lidar data. Set center_deg (0 = front), the array starts at left_offset_deg
        and ends at right_offset_deg. You can choose to select specific points within the offset for example.

        WARNING: THIS FUNCTION IS NOT TESTED AND MAY NOT WORK AS INTENDED. NOT SUGGESTED TO USE AS-IS.
        """

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

        #self.get_logger().info('Scan Data: "%s"' % str(data))
                
        # Index 0 maps to 0, Index len/2 maps to 180, Index len maps to -180
        # Remap this so we have array as expected from -180 to 180
        zero_offset = int(len(data) / 2)
        new_slice = data[zero_offset:] + data[:(zero_offset-1)]

        # Uncomment this to see scan data in console (chatty)
        #self.get_logger().info('Scan Data: "%d"' % len(new_slice))
        
        # Normal - we just take a slice
        start_index = round(start / angleset_deg)
        end_index = round(end / angleset_deg)

        start_index += zero_offset
        end_index += zero_offset

        if end_index > len(data):
            end_index = len(data)-1
            raise ValueError("Oops?!")
        
        if start_index < 0:
            start_index = 0
            raise ValueError("Oops?!")

        if end > start:
            lidar_data = new_slice[start_index:end_index]
            lidar_data = lidar_data[::-1]
            #self.get_logger().info('Scan Data: "%d:%d"' % (start_index, end_index))
            #self.get_logger().info('Scan Data: "%s"' % str(lidar_data))
            return lidar_data
        else:
            raise NotImplementedError("Function cannot deal with splitting array (typically 180 / -180)")


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


def noneIsInfinite(value):
    if value is None:
        return float("inf")
    else:
        return value

def min_ignore_None(data):
    return min(data, key=noneIsInfinite)

class NavigateSquare(Node):
    """Simple class designed to navigate a square"""
    
    def __init__(self):
        #This calls the initilization function of the base Node class
        super().__init__('navigate_square')

        # We can either create variables here by declaring them
        # as part of the 'self' class, or we could declare them
        # earlier as part of the class itself. I find it easier
        # to declare them here

        # Ensure these are obviously floating point numbers and not
        # integers (python is loosely typed)

        # WARNING: Check for updates, note this is set and will run backwards
        #          on the physical model but correctly in simulation.
        self.x_vel = -0.05

        self.d = 0.0
        self.min = 0.15
        self.max = 0.25

        self.new_index = 0

        self.first_run = 0

        self.state = 'START'
        self.ang_vel = pi/4    # pi/2 rad/si
        self.turn_count = 0
        self.current_dir = 'LEFT'

        self.laser_range = 1

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

        self.timer = self.create_timer(0.1, self.timer_callback)

    def control_square(self, turn_index):

        msg = Twist()
        
        self.get_logger().info("-------------" + str(self.d))
        
        in_range = self.d > self.min and self.d < self.max

        if not in_range:
           
            self.state = 'TURN'

            msg.linear.x = 0.0
            msg.angular.z = self.ang_vel
            
            # 1 if min is on left
            # 0 if min is on right
            turn_dir = turn_index > 0 and turn_index < 190

            self.current_dir = dir_table[self.d > self.max][turn_dir]

            self.new_index = (turn_index - 47) % 380 if self.current_dir == 'LEFT' else (turn_index + 47) % 380

        else:

            msg.linear.x = self.x_vel
            msg.angular.z = 0.0

        self.pub_vel.publish(msg)

            
    def control_start(self, turn_index):
        
        self.get_logger().info("control_start()" + str(self.d))        

        msg = Twist()

        msg.linear.x = self.x_vel
        msg.angular.z = 0.0

        if self.d < self.max and self.d > self.min:
            self.state = 'SQUARE'
            self.first_run = 1

        if self.d > 0.5 and self.first_run:

            self.state = 'TURN'

            turn_dir = turn_index > 0 and turn_index < 190

            self.current_dir = dir_table[self.d > self.max][turn_dir]

            self.new_index = (turn_index - 95) % 380 if self.current_dir == 'LEFT' else (turn_index + 95) % 380 


        self.pub_vel.publish(msg)

    def control_spin(self, lidar):

        msg = Twist()
        
        msg.linear.x = 0.0
        msg.angular.z = self.ang_vel if self.current_dir == 'LEFT' else -self.ang_vel
        # right neg vel
        minimum = min_ignore_None(lidar)

        self.get_logger().info(str(lidar.index(minimum)) + " " + str(self.new_index))

        if lidar.index(minimum) > self.new_index - 6 and lidar.index(minimum) < self.new_index + 6: 
            self.state = 'START'
            msg.linear.x = self.x_vel
            msg.angular.z = 0.0

        self.pub_vel.publish(msg)

    def state_check(self):
        
        laser_ranges = self.ldi.get_range_array(0.0)

        if laser_ranges is None:
            return

        laser_ranges_min = min_ignore_None(laser_ranges)

        self.d = laser_ranges_min

        index = laser_ranges.index(self.d)

        if self.d is None:
            return

        #self.get_logger().info("state_check()" + str(laser_ranges))

        if self.state == 'START':

            self.control_start(index)

        elif self.state == 'SQUARE':

            self.control_square(index)

        elif self.state == 'TURN':

            self.control_spin(laser_ranges)

    def lidar_test(self):

        laser_ranges = self.ldi.get_range_array(0.0)

        if laser_ranges is None:
            return

        laser_ranges_min = min_ignore_None(laser_ranges)
        
        index = laser_ranges.index(laser_ranges_min)

        self.get_logger().info(str(index))

    def timer_callback(self):
        """Timer callback for 10Hz control loop"""
        #self.lidar_test()
        self.state_check() 
    

    def odom_callback(self, msg):
        """Callback on 'odom' subscription"""
        #self.get_logger().info('Msg Data: "%s"' % msg)        
        self.x_now = msg.pose.pose.position.x
        self.y_now = msg.pose.pose.position.y

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
        

def main(args=None):
    rclpy.init(args=args)

    navigate_square = NavigateSquare()

    rclpy.spin(navigate_square)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    navigate_square.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
