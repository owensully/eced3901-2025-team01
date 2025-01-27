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

"""
 - dir_table[][]
    Direction table used to determine angular direction
    when the lidar triggers a state change.
"""
dir_table = [['LEFT', 'RIGHT'], ['RIGHT', 'LEFT']] 

"""
 - cutoff_distance
    Distance used to filter out values below the cutoff,
    effectively eliminating false readings.
 - bot_lost
    Distance used to check if bot has left the box range
    while in 'START' state.
"""
cutoff_distance = 0.128
bot_lost = 1

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
        
        """
        The following section is omitted, but is kept for documentation. Data is
        instead returned unprocessed, for precise control over the 380 values in
        the list.

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
        """

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
 - highpassFilter()
    Function used to apply a high-pass filter
    to the lidar data. The 'None' and low distance values
    must be filtered to apply accurate data processing.
"""
def highpassFilter(value):

    if value is None: # filter 'None' values

        return float("inf")

    elif value < cutoff_distance: # filter low distance values

        return float("inf")

    else: # pass-band for all other values

        return value

"""
 - min_lidar_distance()
    Function used to find the minimum value in the lidar array,
    using the high-pass filter to avoid false readings.
"""
def min_lidar_distance(data):

    x = min(data, key=highpassFilter)
    return x
"""
 - NavigateSquare()
    Class used to navigate the square/box obstacle
    in ECED3901: Lab 1
"""
class NavigateSquare(Node):
        
    def __init__(self):
        
        super().__init__('navigate_square')

        self.odom_start = 0.0

        """
         - DEFAULT VELOCITY PARAMETERS
        """
        self.x_vel = -0.05
        self.ang_vel = pi/4

        """
         - DEFAULT ODOMETER PARAMETERS
        """
        self.x_now = 0.0
        self.x_init = 0.0
        self.y_now = 0.0
        self.y_init = 0.0
        self.d_now = 0.0
        self.d_aim = -0.14
        self.d = 0.0
    
        """
         - DEFAULT NAVIGATION PARAMETERS
        """
        self.min = 0.14
        self.max = 0.2
        self.angle = 28

        """
         - LIDAR INDECIES
        """
        self.new_index = 0
        self.final_index = 0

        """
         - RECOVERY PROTOCOL
        """
        self.recovery = 0
        self.recovery_d = 0.24
        self.recovery_angle = 45
        
        """
         - SYSTEM STATES
        """
        self.end = 0
        self.state = 'START'
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
    
        # 10 Hz timer
        self.timer = self.create_timer(0.1, self.timer_callback)

    """
     - control_square()
        Function used while system is in 'SQUARE' state,
        which directs the bot to navigate the square.
    """
    def control_square(self, turn_index):

        msg = Twist()
        
        # debug
        self.get_logger().info("-------------" + str(self.d))
        
        # determine if bot is still in box range
        in_range = self.d > self.min and self.d < self.max

        if in_range:

            # determine if bot has travelled away from starting point
            if abs(self.y_now) > 5 * self.d_aim:

                self.end = 1

            # drive straight
            msg.linear.x = self.x_vel
            msg.angular.z = 0.0

        else:
            
            # toggle state if bot left range
            self.state = 'TURN'

            # termiante movement
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            
            # 1 if min is on left
            # 0 if min is on right
            # determine turn direction using minimum lidar distance index
            turn_dir = turn_index > 0 and turn_index < 190

            # use direction table to determine direction
            self.current_dir = dir_table[self.d > self.max][turn_dir]

            # high turn angle if bot is close to box, otherwise low turn angle
            self.angle = 45 if self.d < self.min else 28

            # determine turn radius using angle
            self.new_index = (turn_index - self.angle) % 380 if self.current_dir == 'LEFT' else (turn_index + self.angle) % 380

        # publish state
        self.pub_vel.publish(msg)
     
    """
     - control_start()
        Function used while system is in 'START' state,
        which directs the bot to drive straight until
        box is in close range.
    """
    def control_start(self, turn_index):
        
        # debug
        self.get_logger().info("control_start()" + str(self.d))        

        msg = Twist()
        
        # drive straight
        msg.linear.x = self.x_vel
        msg.angular.z = 0.0

        # check if box is within range
        if self.d < self.max and self.d > self.min:

            # toggle state if box is in range
            self.state = 'SQUARE'

            # enable recovery flag
            self.recovery = 1

        # check if recovery protocol needs to be activated
        if self.d > self.recovery_d and self.recovery or self.d > bot_lost:
            
            self.get_logger().info("rec")


            # toggle state to turn bot
            self.state = 'TURN'

            # determine turn direction using minimum lidar distance index
            turn_dir = turn_index > 0 and turn_index < 190

            # use direction table to determine direction
            self.current_dir = dir_table[self.d > self.max][turn_dir]

            # high turn angle if bot is in recovery mode, otherwise bot is lost and needs to return to box
            self.recovery_angle = 45 if self.d < bot_lost else 0
            
            # determine turn radius using angle
            self.new_index = (turn_index - self.recovery_angle) % 380 if self.current_dir == 'LEFT' else (turn_index + self.recovery_angle) % 380 

            # toggle recovery flag
            self.recovery = 0

            # terminate movement
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        """
         # check if box is within range
        if self.d < self.max and self.d > self.min:

            # toggle state if box is in range
            self.state = 'SQUARE'

            # enable recovery flag
            self.recovery = 1 
        """
        # publish state
        self.pub_vel.publish(msg)

    """
     - control_spin()
        Function used while system is in 'TURN' state,
        which directs the bot to spin until desired angle
        has been reached.
    """
    def control_spin(self, lidar):
        
        msg = Twist()
        
        # spin
        msg.linear.x = 0.0
        msg.angular.z = self.ang_vel if self.current_dir == 'LEFT' else -self.ang_vel
        # right neg vel

        # determine minimum lidar distance
        minimum = min_lidar_distance(lidar)

        # debug
        self.get_logger().info(str(lidar.index(minimum)) + " " + str(self.new_index))

        # if the but is facing the desired index, the turn is complete
        if lidar.index(minimum) > self.new_index - 6 and lidar.index(minimum) < self.new_index + 6: 
        
            # toggle state to drive straight
            self.state = 'START'

        elif lidar.index(minimum) > self.final_index - 6 and lidar.index(minimum) < self.final_index + 6 and self.final_index:

            self.get_logger().info("toggle final")


            # toggle state to initiate final sequence
            self.state = 'FINAL'

            # drive straight
            msg.linear.x = self.x_vel
            msg.angular.z = 0.0

        # publish state
        self.pub_vel.publish(msg)

    """
     - state_check()
        Function used to check system state and direct the bot
        accordingly.
    """
    def state_check(self):

        # pull lidar array
        laser_ranges = self.ldi.get_range_array(0.0)

        # determine minimum lidar distance
        laser_ranges_min = min_lidar_distance(laser_ranges)
        
        # assign minimum lidar distance to current distance
        self.d = laser_ranges_min

        # find index of minimum distance
        index = laser_ranges.index(self.d)

        # if bot has complete trip and has not been given a final index
        if not self.odom_check() and not self.final_index:

            self.get_logger().info("setting final index")


            # toggle state to spin
            self.state == 'TURN'

            # set final index as closest location
            self.final_index = index

            # determine turn direction
            turn_dir = index > 0 and index < 190
            
            # set currect direction
            self.current_dir = 'LEFT' if turn_dir else 'RIGHT'

        # initiate process based on state
        if self.state == 'START':
            
            self.control_start(index)

        elif self.state == 'SQUARE':

            self.control_square(index)

        elif self.state == 'TURN':

            self.control_spin(laser_ranges)

        elif self.state == 'FINAL':

            self.control_final()

    """
     - lidar_test()
        Inactive function used to test the lidar abilities of
        the bot.
    """
    def lidar_test(self):

        laser_ranges = self.ldi.get_range_array(0.0)

        if laser_ranges is None:
            return

        laser_ranges_min = min_ignore_None(laser_ranges)
        
        index = laser_ranges.index(laser_ranges_min)

        self.get_logger().info(str(index))

    """
     - odom_check()
        Function used to determine if bot has complete trip around
        box.
    """
    def odom_check(self):

        # check if end flag is toggled and bot is beyond stop point
        if self.end and self.y_now > self.d_aim:
            self.get_logger().info("odom_check") 
            return False

        else:

            return True

    """
     - control_final()
        Function used while system is in 'FINAL' state,
        which terminates the process when the bot has
        circled the box.
    """
    def control_final(self):

        msg = Twist()

        # check if bot is close to box
        if self.d < self.min:

            # terminate movement
            msg.linear.x = 0.0
            msg.angular.z = 0.0

            # publish state
            self.pub_vel.publish(msg)

            # end process
            exit(0)

    def timer_callback(self):
        """Timer callback for 10Hz control loop"""

        #self.lidar_test()
        self.state_check() 
    

    def odom_callback(self, msg):
        """Callback on 'odom' subscription"""
        #self.get_logger().info('Msg Data: "%s"' % msg) 
        self.y_now = msg.pose.pose.position.y - self.odom_start

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
