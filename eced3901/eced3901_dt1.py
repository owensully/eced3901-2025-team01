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
import sys
import cv2
import serial

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

        self.subProcess = [self.lootProcess, self.magnetProcess, self.rfidProcess, self.qrProcess, self.safeProcess, self.indianaProcess, self.cageProcess, self.sleepProcess]
     
        self.cell = start_cell

        self.attribute_check_complete = 0
        
        self.has_init = 0

        self.tolerance = 0.10

        self.old_dir = 0
        self.new_dir = 0

        self.velocity = [0.0, 0.0]

        self.sub = 0
        self.subprocess = 'no'

        self.c_yaw = 0
        self.t_yaw = 0

        self.ox = 0
        self.oy = 0
        self.oz = 0
        self.ow = 0

        self.y = 0
        self_x = 0

        self.mag = 0
        self.rfid = 0
        self.safe = 0
        self.qr = 0
        self.i = 0
        self.c = 0

        self.kondo = 0

        self.start = 0

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
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        """Timer callback for 10Hz control loop"""
              
        if not self.start:

            self.startProcess()

            return
    
        #self.safeProcess()
        #self.test()
        #return
        #self.kondoBitch()

        #return

        if not self.has_init:

            self.init()

        
        self.primaryProcess()
        #self.lidar_test() 
        

    def odom_callback(self, msg):
        """Callback on 'odom' subscription"""
 
        self.y = msg.pose.pose.position.y# - self.odom_start
        self.x = msg.pose.pose.position.x

        self.ox = msg.pose.pose.orientation.x
        self.oy = msg.pose.pose.orientation.y
        self.oz = msg.pose.pose.orientation.z
        self.ow = msg.pose.pose.orientation.w 

        a = 2 * (self.ow * self.oz + self.ox * self.oy)
        b = 1 - 2 * (self.oy * self.oy + self.oz * self.oz)
        self.c_yaw = np.arctan2(a, b)

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

    def setVel(self, v):

        vel = Twist()

        vel.linear.x = v[0]
        vel.angular.z = v[1]

        self.pub_vel.publish(vel)

    def startProcess(self):

        lidar = self.pullLidar(cell_directions(0))

        if lidar is None:

            return

        if lidar[0] is None:

            return

        if lidar[0] > 0.5:

            self.start = 1

    def test(self):
        
        print(self.pullLidar(cell_directions(0)))

        ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # Windows
        ser.write("EXTEND".encode())
        sleep(2)
        response = ser.readline().decode('utf-8').strip()
        while response != "DONE":
        
            response = ser.readline().decode('utf-8').strip()
            print(f"{response}")

        ser.write("RETRACT".encode())
        sleep(2)
        response = ser.readline().decode('utf-8').strip()
        while response != "DONE":
        
            response = ser.readline().decode('utf-8').strip()
            print(f"{response}")


        ser.close() 

    def kondoBitch(self):

        if self.kondo == 0:

            self.velocity = decode_direction_map[0]

            self.setVel(self.velocity)

            self.kondo = 1

        elif self.kondo == 1:

            lidar = self.pullLidar(cell_directions(0))

            if lidar[2] is None:

                return

            if lidar is None:

                return

            if lidar[2] > 0.65:

                self.velocity = decode_direction_map[1]

                self.setVel(self.velocity)

                self.t_yaw = self.c_yaw - (pi / 8)
                
                self.t_yaw = (self.t_yaw + pi) % (2 * pi) - pi

                self.kondo = 2

        elif self.kondo == 2:

            self.checkSpin()

            if self.velocity[0]:

                self.velocity = decode_direction_map[2]

                self.setVel(self.velocity)

                self.kondo = 3

                self.t_yaw = self.c_yaw + (pi / 8)
                
                self.t_yaw = (self.t_yaw + pi) % (2 * pi) - pi

        elif self.kondo == 3:

            self.checkSpin()

            if self.velocity[0]:

                self.velocity = decode_direction_map[0]

                self.setVel(self.velocity)

                self.kondo = 4

        elif self.kondo == 4:

            lidar = self.pullLidar(cell_directions(0))

            if lidar[2] is None:

                return

            if lidar is None:

                return

            if lidar[2] > 0.80:

                self.velocity = decode_direction_map[2]

                self.setVel(self.velocity)

                self.kondo = 5

                self.t_yaw = self.c_yaw + (pi / 8)
                
                self.t_yaw = (self.t_yaw + pi) % (2 * pi) - pi
 
        elif self.kondo == 5:

            self.checkSpin()

            if self.velocity[0]:

                self.velocity = decode_direction_map[1]

                self.setVel(self.velocity)

                self.kondo = 6

                self.t_yaw = self.c_yaw - (pi / 8)
                
                self.t_yaw = (self.t_yaw + pi) % (2 * pi) - pi
                
        elif self.kondo == 6:

            return

    def init(self):

        print(self.cell)

        self.setVel(decode_direction_map[0])

        self.has_init = 1

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

        return lr2


    """
     - checkLidar()
        function used to compare two lidar array
        within a set tolerance.
    """
    def checkLidar(self, lidar1, lidar2, t): 

        x = 4

        for l1, l2 in zip(lidar1, lidar2):
        
            if l1 is None or l2 is None:

                x = x - 1

                continue
                            
            if abs(l1 - l2) > t:

                x = x - 1
        
        if x >= 3:

            return True



    """
     - pullLidar()
        function used to select and return values from
        lidar range array.
    """
    def pullLidar(self, args):

        for arg in args:

            if arg is None:

                return

        # read 380 lidar values
        laser_ranges = self.ldi.get_range_array(0.0)

        if laser_ranges is None:

            return

        # select pull values from range array
        pull_values = [laser_ranges[arg] for arg in args if 0 <= arg < 380]
        
        return pull_values
   
    """
     - checkNN()
        function used to determine the next cell location
        by comparing current lidar values to the lidar
        values stored in the nearest neighbours.
    """
    def checkNN(self, cell, directions):

        # loop through provided directions
        for direction in directions:
            
            # find the neighbour in each direction
            new_cell = list(map(lambda i, j: i + j, cell, direction))
             
            if new_cell[0] < 0 or new_cell[0] > 10:

                continue

            if new_cell[1] < 0 or new_cell[1] > 10:

                continue

            try:
                
                index = blacklist.index(new_cell)
            
            except:

                # query the lidar values for the new cell
                new_cell_lidar = [course_map[map_layers.index(d)][new_cell[0]][new_cell[1]] for d in NWSE]

                n = course_map[map_layers.index("direction")][cell[0]][cell[1]]

                d = (course_map[map_layers.index("direction")][new_cell[0]][new_cell[1]] - n + 8) % 8

                # determine the current lidar values
                lidar = self.pullLidar(cell_directions(angles[d]))

                # ensure proper values were returned
                if lidar is None:

                    return


                # determine if current lidar values match map lidar values
                if self.checkLidar(lidar, new_cell_lidar, self.tolerance):

                    if abs(self.cell[0] - new_cell[0]) > 1 or abs(self.cell[1] - new_cell[1]) > 1:

                        if self.cell[0] == 3 and self.cell[1] == 7:

                            continue

                    blacklist.append(self.cell)

                    # update current cell
                    return new_cell

        return False



    """
     - checkSpin()
        function used to check if the bot should still
        be spinning by comparing current lidar values
        to cell lidar values.
    """
    def checkSpin(self):
        
        diff = self.t_yaw - self.c_yaw
        diff = (diff + pi) % (2 * pi) - pi

        if abs(diff) < 0.045:
            
            self.velocity = decode_direction_map[0]

            self.setVel(self.velocity)

    """
     - checkCell()
        function used to check the cell attributes
        at given coordinates.
    """
    def checkCell(self, cell):

        # all possible attributes
        for attribute in attribute_priority:

            # if attribute exists in current map
            if attribute in map_layers:

                print(attribute)

                # if this cell has this attribute
                if course_map[map_layers.index(attribute)][cell[0]][cell[1]]:
                   
                    print("found")

                    self.sub = 1
                    
                    self.subprocess = attribute
                    
                    return

    def lootProcess(self):

        pass

    def magnetProcess(self):
        
        print(self.mag, self.velocity)

        if self.mag == 0: # align 

            sleep(1)

            self.mag = 1

        if self.mag == 1: # begin right turn
 
            self.t_yaw = self.c_yaw - (pi / 2)
            
            self.t_yaw = (self.t_yaw + pi) % (2 * pi) - pi

            self.mag = 2

            self.velocity = decode_direction_map[1]

            self.setVel(self.velocity)

        elif self.mag == 2: # begin reverse once turn is finished

            self.checkSpin()

            if self.velocity[0]:

                self.mag = 3

                self.velocity = decode_direction_map[4]

                self.setVel(self.velocity)

        elif self.mag == 3: # reverse until magnet is placed

            lidar = self.pullLidar(cell_directions(0))

            if lidar[2] is None:

                return

            if lidar[2] < 0.17:

                sleep(1)

                self.velocity = decode_direction_map[0]

                self.setVel(self.velocity)

                self.mag = 4

        elif self.mag == 4: # forward until re-aligned on grid, begin left turn

            lidar = self.pullLidar(cell_directions(0))
            
            if lidar[2] is None:

                return

            if lidar[2] > 0.22:

                self.mag = 5

                self.velocity = decode_direction_map[2]

                self.setVel(self.velocity)

                self.t_yaw = self.c_yaw + (pi / 2)
            
                self.t_yaw = (self.t_yaw + pi) % (2 * pi) - pi

        elif self.mag == 5: # resume when turn is finished

            self.checkSpin()

            if self.velocity[0]:

                self.sub = 0

                self.velocity = decode_direction_map[0]

                self.setVel(self.velocity)

    def rfidProcess(self):

        print(self.rfid)

        if self.rfid == 0: # align

            self.rfid = 1

            sleep(2)

        elif self.rfid == 1: # begin left turn

            self.rfid = 2

            self.velocity = decode_direction_map[2]
            
            self.setVel(self.velocity)

            self.t_yaw = self.c_yaw + (pi / 2)
            
            self.t_yaw = (self.t_yaw + pi) % (2 * pi) - pi

        elif self.rfid == 2: # reverse step

            self.checkSpin()

            if self.velocity[0]:

                self.rfid = 3

                self.velocity = decode_direction_map[4]

                self.setVel(self.velocity)

        elif self.rfid == 3: # reverse into wall when turn is finished

            lidar = self.pullLidar(cell_directions(0))

            if lidar[2] is None:

                return

            if lidar[2] < 0.17:
                
                self.velocity = [0.02, 0.0]

                self.setVel(self.velocity)

                sleep(1)

                self.velocity = [0.0, 0.0]
                
                self.t_yaw = self.c_yaw                

                self.setVel(self.velocity)

                sleep(1)

                self.velocity = [-0.01, 0.0]#decode_direction_map[0]

                self.setVel(self.velocity)

                self.rfid = 4

                sleep(0.05)

                self.velocity = yawVel(self.t_yaw, self.c_yaw)

                self.setVel(self.velocity)

        elif self.rfid == 4:

            self.checkSpin()

            if self.velocity[0]:

                self.velocity = decode_direction_map[0]

                self.setVel(self.velocity)
                
                self.rfid = 5

        elif self.rfid == 5: # turn right once off wall

            lidar = self.pullLidar(cell_directions(0))

            if lidar[2] is None:

                return

            if lidar[2] > 0.17:

                sleep(0.25)

                self.rfid = 6

                self.velocity = decode_direction_map[1]
                
                self.setVel(self.velocity)

                self.t_yaw = self.c_yaw - (pi / 2)
                
                self.t_yaw = (self.t_yaw + pi) % (2 * pi) - pi

        elif self.rfid == 6: # forward once turn is complete

            self.checkSpin()

            if self.velocity[0]:

                self.rfid = 7

                self.velocity = decode_direction_map[0]

                self.setVel(self.velocity)

        elif self.rfid == 7: # turn left when safe is approaching

            lidar = self.pullLidar(cell_directions(0))

            if lidar[0] is None:

                return

            if lidar[0] < 0.37:

                self.velocity = decode_direction_map[2]

                self.setVel(self.velocity)

                self.rfid = 8

                self.t_yaw = self.c_yaw + (pi / 2)
            
                self.t_yaw = (self.t_yaw + pi) % (2 * pi) - pi

        elif self.rfid == 8:

            self.checkSpin()

            if self.velocity[0]:

                self.velocity = decode_direction_map[0]

                self.setVel(self.velocity)

                self.rfid = 9

        elif self.rfid == 9: # resume at cell [0, 2]

            self.velocity = decode_direction_map[0]

            self.setVel(self.velocity)

            #self.setVel(decode_direction_map[0])
            
            self.cell = [0, 2]

            # update the new and old directions

            self.old_dir = course_map[map_layers.index("direction")][self.cell[0]][self.cell[1]] 

            self.new_dir = course_map[map_layers.index("direction")][self.cell[0]][self.cell[1]] 
            
            self.sub = 1

            self.subprocess = "cage"

    def qrProcess(self):
    
        return

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
            #dst = pic_dst + "qr.jpg"
            dst = "home/student/ros2_ws/src/eced3901/eced3901/qr.jpg"
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

        if qr_data:

            return qr_data

        else:

            return False

    def safeProcess(self):

        if self.safe == 0:

            self.safe = 3

            return

            self.velocity = decode_direction_map[0]

            self.setVel(self.velocity)

            self.safe = 2

        elif self.safe == 1:

            lidar = self.pullLidar(cell_directions(0))

            if lidar[2] is None:

                return

            if lidar[2] > 0.83: # --------------------------------------------- place holder south value

                self.safe = 2

                self.velocity = decode_direction_map[1]
                
                self.setVel(self.velocity)

                self.t_yaw = self.c_yaw - (pi / 2)
                
                self.t_yaw = (self.t_yaw + pi) % (2 * pi) - pi

        elif self.safe == 2:

            self.checkSpin()

            if self.velocity[0]:

                self.safe = 3

                self.velocity = decode_direction_map[0]

                self.setVel(self.velocity)

                sleep(4)

                self.velocity = [-0.01, 0.0]

                self.setVel(self.velocity)

                """

                uncomment this to stop the burnout
                """
                #self.velocity = decode_direction_map[3]

                #self.setVel(self.velocity)


        elif self.safe == 3:

            #self.qr = self.qrProcess()

            self.qr = "QR: 5-4-3"
            if self.qr:
            
                # Open serial port - adjust COM port as needed
                ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # Windows
                # ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # Linux
                # ser = serial.Serial('/dev/cu.usbmodem14101', 115200, timeout=1)  # macOs              
                try:
                    # Allow time for connection to establish
                    sleep(1)
                    
                    # Send data to Pico
                    #data_to_send = "5-4-3"
                    self.qr = "QR: 5-4-3"
                    ser.write(self.qr.encode())
                    response = ser.readline().decode('utf-8').strip()
                    while response != "DONE":
                    
                        # Optional: Read response
                        response = ser.readline().decode('utf-8').strip()
                        print(f"{response}")
                    #if response == 'ALARM" ------------------------------------------------------

                finally:

                    ser.close()
                    
                    #self.safe = 4
            
        elif self.safe == 4:

            self.velocity = decode_direction_map[4]

            self.setVel(self.velocity)

            self.safe = 5

        elif self.safe == 5:

            lidar = self.pullLidar(cell_directions(0))

            if lidar[0] is None:

                return

            if lidar[0] > 0.22:

                self.velocity = decode_direction_map[1]

                self.setVel(self.velocity)

                self.safe = 6

                self.t_yaw = self.c_yaw - (pi / 2)
                
                self.t_yaw = (self.t_yaw + pi) % (2 * pi) - pi
        
        elif self.safe == 6:
            
            self.checkSpin()

            if self.velocity[0]:

                self.velocity = decode_direction_map[0]

                self.setVel(self.velocity)

                self.safe = 7

        elif self.safe == 7:

            lidar = self.pullLidar(cell_directions(0))

            if lidar[0] is None:

                return

            if lidar[0] < 0.277:

                self.velocity = decode_direction_map[2]

                self.setVel(self.velocity)

                self.safe = 8

                self.t_yaw = self.c_yaw + (pi / 2)
                
                self.t_yaw = (self.t_yaw + pi) % (2 * pi) - pi

        elif self.safe == 8:

            self.checkSpin()

            if self.velocity[0]:

                self.velocity = decode_direction_map[0]

                self.setVel(self.velocity)

                self.safe = 9

        elif self.safe == 9:

            lidar = self.pullLidar(cell_directions(0))

            if lidar[0] is None:

                return

            if lidar[0] < 0.22:

                self.velocity = decode_direction_map[4]

                self.setVel(self.velocity)

                self.safe = 10

        elif self.safe == 10:

            lidar = self.pullLidar(cell_directions(0))

            if lidar[2] is None:

                return

            if lidar[2] < 0.17:

                sleep(2)

                exit(0)

    def indianaProcess(self):

        if self.i == 0:

            self.velocity = decode_direction_map[0]

            self.setVel(self.velocity)

            self.i = 1

        elif self.i == 1:

            lidar = self.pullLidar(cell_directions(0))

            if lidar[0] is None:

                return

            if lidar[0] > 0.757: # ==========================================================place holder

                self.velocity = decode_direction_map[1]

                self.setVel(self.velocity)

                self.i = 2

                self.t_yaw = self.c_yaw - (pi / 2)
                
                self.t_yaw = (self.t_yaw + pi) % (2 * pi) - pi
 

        elif self.i == 2:

            self.checkSpin()

            if self.velocity[0]:

                self.velocity = [-0.01, 0.0]#decode_direction_map[0]

                self.setVel(self.velocity)

                self.i = 3

        elif self.i == 3:

            lidar = self.pullLidar(cell_directions(0))

            if lidar[0] is None:

                return

            if lidar[0] > 0.54: # =================================================================== place holder

                self.velocity = decode_direction_map[3]

                self.setVel(self.velocity)

                self.i = 4

        elif self.i == 4:


            ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # Windows
            ser.write("EXTEND".encode())
            sleep(2)
            response = ser.readline().decode('utf-8').strip()
            while response != "DONE":
            
                response = ser.readline().decode('utf-8').strip()
                print(f"{response}")

            ser.write("RETRACT".encode())
            sleep(2)
            response = ser.readline().decode('utf-8').strip()
            while response != "DONE":
            
                response = ser.readline().decode('utf-8').strip()
                print(f"{response}")


            ser.close() 

            sleep(1)
            # this is where i write the servo

            self.i = 5

        elif self.i == 5:

            self.velocity = decode_direction_map[4]

            self.setVel(self.velocity)

            self.i = 6

        elif self.i == 6:

            lidar = self.pullLidar(cell_directions(0))

            if lidar[2] is None:

                return

            if lidar[2] < 0.32:

                self.velocity = decode_direction_map[1]

                self.t_yaw = self.c_yaw - (pi / 2)
                
                self.t_yaw = (self.t_yaw + pi) % (2 * pi) - pi
 
                self.i = 7

        elif self.i == 7:

            self.checkSpin()

            if self.velocity[0]:

                self.velocity = decode_direction_map[0]

                self.setVel(self.velocity)

                self.i = 8

        elif self.i == 8:

            lidar = pullLidar(cell_directions(0))

            if lidar[0] is None:

                return

            if lidar[0] < 0.67:

                self.velocity = decode_direction_map[1]

                self.setVel(self.velocity)

                self.i = 9

                self.t_yaw = self.c_yaw - (pi / 2)
                
                self.t_yaw = (self.t_yaw + pi) % (2 * pi) - pi
 
        elif self.i == 9:

            self.checkSpin()

            if self.velocity[0]:

                self.velocity = decode_direction_map[0] 

                self.setVel(self.velocity)

                self.i = 10

        elif self.i == 10:

            lidar = self.pullLidar(cell_directions(0))

            if lidar[2] is None:

                return

            if lidar[2] < 1.25:

                self.velocity = decode_direction_map[1]

                self.setVel(self.velocity)

                self.i = 11

                self.t_yaw = self.c_yaw - (pi / 2)
                
                self.t_yaw = (self.t_yaw + pi) % (2 * pi) - pi
 

        elif self.i == 11:

            self.checkSpin()

            if self.velocity[0]:

                self.velocity = decode_direction_map[0]

                self.setVel(self.velocity)

                self.i = 12

        elif self.i == 12:

            lidar = self.pullLidar(cell_directions(0))

            if lidar[0] is None:

                return

            if lidar[0] < 0.25:

                self.velocity = decode_direction_map[0]

                self.setVel(self.velocity)

                exit(0)


    def cageProcess(self):

        if self.c == 0:

            self.c = 1

            self.velocity = decode_direction_map[0]

            self.setVel(self.velocity)

        elif self.c == 1:

            lidar = self.pullLidar(cell_directions(0))

            if lidar[0] is None:

                return

            if lidar[0] < 0.422:

                self.velocity = decode_direction_map[1]
                                
                self.setVel(self.velocity)

                self.c = 2

                self.t_yaw = self.c_yaw - (pi / 2)
                
                self.t_yaw = (self.t_yaw + pi) % (2 * pi) - pi
 
        elif self.c == 2:

            self.checkSpin()

            if self.velocity[0]:

                self.velocity = decode_direction_map[0]

                self.setVel(self.velocity)

                self.c = 3

        elif self.c == 3:

            lidar = self.pullLidar(cell_directions(0))

            if lidar[0] is None:

                return

            if lidar[0] < 0.25:

                self.velocity = decode_direction_map[4]

                self.setVel(self.velocity)

                self.c = 4

        elif self.c == 4:

            lidar = self.pullLidar(cell_directions(0))

            if lidar[2] is None:

                return

            if lidar[2] < 0.32:

                self.velocity = decode_direction_map[1]

                self.setVel(self.velocity)

                self.c = 5

                self.t_yaw = self.c_yaw - (pi / 2)
                
                self.t_yaw = (self.t_yaw + pi) % (2 * pi) - pi

        elif self.c == 5:

            self.checkSpin()

            if self.velocity[0]:

                self.velocity = decode_direction_map[0]

                self.setVel(self.velocity)

                self.c = 6

        elif self.c == 6:

            lidar = self.pullLidar(cell_directions(0))

            if lidar[0] is None:

                return

            if lidar[0] < 0.67:

                self.velocity = decode_direction_map[1]

                self.setVel(self.velocity)

                self.c = 7

                self.t_yaw = self.c_yaw - (pi / 2)
                
                self.t_yaw = (self.t_yaw + pi) % (2 * pi) - pi

        elif self.c == 7:

            self.checkSpin()

            if self.velocity[0]:

                self.velocity = decode_direction_map[0]

                self.setVel(self.velocity)

                self.c = 8

        elif self.c == 8:

            lidar = self.pullLidar(cell_directions(0))

            if lidar[2] is None:

                return

            if lidar[2] < 1.25:

                self.velocity = decode_direction_map[1]

                self.setVel(self.velocity)

                self.c = 9

                self.t_yaw = self.c_yaw - (pi / 2)
                
                self.t_yaw = (self.t_yaw + pi) % (2 * pi) - pi


        elif self.c == 9:

            self.checkSpin()

            if self.velocity[0]:

                self.velocity = decode_direction_map[0]

                self.setVel(self.velocity)

                self.c = 10

        elif self.c == 10:

            lidar = self.pullLidar(cell_directions(0))

            if lidar[0] is None:

                return

            if lidar[0] < 0.25:

                self.velocity = decode_direction_map[3]

                self.setVel(self.velocity)

                exit(0)

    def sleepProcess(self):

        sleep(2.5)

        self.sub = 0

    """
     - primaryProcess()
        function repeated by the timer callback
        which directs the primary process of the
        robot.
    """
    def primaryProcess(self):
        
        # check if subprocess is in progress
        if self.sub:

            print("calling... " + self.subprocess)

            # jump to currect subprocess
            self.subProcess[attribute_priority.index(self.subprocess)]()

            return

        # determine if the attributes have been checked
        if not self.attribute_check_complete:

            print("doing cell check...")
            
            self.checkCell(self.cell)
             
            # toggle attribute check flag
            self.attribute_check_complete = 1

            return

        # check if bot is spinning
        if self.velocity[1]:

            # set wheel velocity
            self.setVel(self.velocity)

            #print(self.c_yaw)

            # check if bot should still be spinning
            self.checkSpin()

            return

        # compare current NWSE lidar values with neighbour lidar values
        new_cell = self.checkNN(self.cell, nearest_neighbour)

        # if new cell is found, update current cell
        if new_cell:

            print(new_cell)

            # reset attribute flag
            self.attribute_check_complete = 0

            # update current cell
            self.cell = new_cell

            # update the new and old directions
            self.old_dir = self.new_dir
            self.new_dir = course_map[map_layers.index("direction")][self.cell[0]][self.cell[1]] 

            #c = course_map[map_layers.index("direction")][self.cell[0]][self.cell[1]]
            c = self.old_dir
            n = self.new_dir
            #n = course_map[map_layers.index("direction")][new_cell[0]][new_cell[1]]

            d = (n - c + 8) % 8
 
            self.t_yaw = self.c_yaw + odom_angles[d]
            
            self.t_yaw = (self.t_yaw + pi) % (2 * pi) - pi

            print(self.c_yaw, self.t_yaw)
 
            # use directions to determine new speed
            self.velocity = decode_direction_map[direction_map[int(self.new_dir)][int(self.old_dir)]]

            if course_map[map_layers.index("neg")][self.cell[0]][self.cell[1]]:

                self.velocity[0] = self.velocity[0] * -1
                self.velocity[1] = self.velocity[1] * -1

            #if not self.velocity[1]:
            
            #if self.velocity[1]:

                #sleep(1.25)

            if not self.velocity[1]:
                # set wheel velocity
                self.setVel(self.velocity)

               
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

    map_data = [[list(map(float, line.split())) for line in open(map_directory + filename)] for filename in map_layers]

    return map_data, map_layers

"""
 - read_directions()
    function used to load the direction table file
    into a 2D array.
"""
def read_directions():

    with open(current_directory + "/dir_table", "r") as file:

        return [list(map(int, line.split())) for line in file]

"""
 - cell_directions()
    function used to convert NWSE to align with a given
    cell direction.
"""
def cell_directions(d):

    n, w, s, e = (A["NORTH"] + d) % 380, (A["WEST"] + d) % 380, (A["SOUTH"] + d) % 380, (A["EAST"] + d) % 380

    # ensure LIDAR index 0 is not used
    # it is broken
    if n == 0:
        n = n + 1
    if w == 0:
        w = w + 1
    if s == 0:
        s = s + 1
    if e == 0:
        e = e + 1

    return n, w, s, e

def yawVel(t, c):

    if t > c:

        return [0.0, 0.1]

    else:

        return [0.0, -0.1]
"""
 ========== FINAL GLOBAL VARIABLES ==========
"""

spin_velocity = 0.17
forward_velocity = -0.05
start_cell = [9, 7]

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
    -1: 1,
    0: 1,
    1: 47,
    2: 95,
    3: 142,
    4: 190,
    5: 237,
    6: 285,
    7: 332
}

odom_angles = {
    0: 0,
    2: (pi / 2),
    4: pi,
    6: (-pi / 2)
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
#nearest_neighbour = [ [0, -1], [-1, 0], [0, 1], [1, 0] ]
nearest_neighbour = [ [0, -1], [-1, 0], [0, 1], [1, 0], [-1, -1], [1, -1], [1, 1], [-1, 1], [0, -2], [-2, 0], [0, 2], [2, 0] ]

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
    2: [0.0, spin_velocity],
    3: [0.0, 0.0],
    4: [(0.5 * -forward_velocity), 0.0]
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
    "NORTH-EAST": 7
}

"""
 list of possible file names that form the map,
 logged in their respective priority.
"""
attribute_priority = [ "loot", "magnet", "rfid", "qr", "safe", "indianna", "cage", "sleep"]

blacklist = []

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
