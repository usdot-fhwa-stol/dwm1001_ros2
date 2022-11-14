#!/usr/bin/env python3

# Written by the USDOT Volpe National Transportation Systems Center
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

from dwm1001_ros2 import serial_tools
import time, serial, sys

# import rclpy
from rclpy.node import Node

from geometry_msgs.msg  import PoseStamped
from geometry_msgs.msg  import TwistStamped

from dwm1001_ros2.dwm1001_apiCommands import DWM1001_API_COMMANDS

class dwm1001_sensor():

    def __init__(self, node: Node, params):
        
        self.node = node

        (topic_prefix, serial_number, device_name, tag_location, tag_name) = \
            self.node.get_parameters(['topic_prefix','serial_number','device_name','tag_location','tag_name'])

        self.dwm_tag_publisher_=node.create_publisher(PoseStamped,topic_prefix.value + 'pose',10)
        self.dwm_twist_publisher_=node.create_publisher(TwistStamped,topic_prefix.value + 'twist',10)

        self.port = None
        node.get_logger().info('Looking for DWM1001 Port')
        while self.port == None:
            if serial_number.value == 'n/a':
                self.port = serial_tools.get_port(device_name.value)
            else:
                self.port = serial_tools.get_port_from_sn(serial_number.value)
            time.sleep(0.1)

        node.get_logger().info('DWM1001 port is found at %s' %self.port)

        self.last_time = time.time()

        self.previous_p = PoseStamped()

        self.eastings_offset = 0.0
        self.northings_offset = 0.0

    def setup(self):

        self.dwm1001_serial = serial.Serial(
            self.port, # = serial_tools.get_port(self.deviceName_), # dynamically detect port if there are a lot of devices connected
            baudrate= 115200,
            parity = serial.PARITY_ODD,
            stopbits = serial.STOPBITS_TWO,
            bytesize = serial.SEVENBITS
        )

        # CLOSE AND THEN OPEN PORT
        # close port if open:
        try:
            self.dwm1001_serial.close()
        except:
            pass
        # open port
        self.node.get_logger().info("Waiting for DWM1001 port to open")
        while self.dwm1001_serial.isOpen() == False:
            try:
                self.dwm1001_serial.open()
            except:
                pass
            time.sleep(0.1)

        if(self.dwm1001_serial.isOpen()):
            self.node.get_logger().info('Port opened: %s'%(str(self.dwm1001_serial.name)))
            # start sending commands to the board so we can initialize the board
            self.initialize_dwm1001_api()
            # give some time to DWM1001 to wake up
            time.sleep(2)
            # send command lec, so we can get positions in CSV format
            self.dwm1001_serial.write(DWM1001_API_COMMANDS.LEC)
            self.dwm1001_serial.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
        else:
            self.node.get_logger().info("Can't open port: "+ str(self.dwm1001_serial.name))

    def initialize_dwm1001_api(self):
        """
        Initialize dwm1001 api, by sending bytes
        :param:
        :returns: none
        """
        # reset incase previous run didn't close properly
        self.dwm1001_serial.write(DWM1001_API_COMMANDS.RESET)
        # send ENTER two times in order to access api
        self.dwm1001_serial.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
        # sleep for half a second
        time.sleep(0.5)
        self.dwm1001_serial.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
        # sleep for half second
        time.sleep(0.5)
        # send a third one - just in case
        self.dwm1001_serial.write(DWM1001_API_COMMANDS.SINGLE_ENTER)

    def get_data(self):
        self.node.get_logger().info("Reading DWM1001 coordinates")
        try:
            while True:
                # read the serial port
                serialReadLine = self.dwm1001_serial.read_until()
                try:
                    self.publish_positions(serialReadLine)
                except IndexError:
                    self.node.get_logger().info('Found index error in the network array! DO SOMETHING!')
                except KeyboardInterrupt:
                    sys.exit()
        except KeyboardInterrupt:
            sys.exit()


    def publish_positions(self, serialData):
        arrayData = str(serialData).split(',')
        if "DIST" in arrayData[0]:
            for i in range(len(arrayData)):
                if "POS" in arrayData[i]:
                    pass
            # publish position info from the tag that's connected
            if "POS" in arrayData[-5]:
                try:
                    new_time = time.time()
                    p = PoseStamped()
                    p.header.stamp = self.node.get_clock().now().to_msg()
                    p.header.frame_id = "ips"
                    p.pose.position.x = float(arrayData[-4])
                    p.pose.position.y = float(arrayData[-3])
                    p.pose.position.z = float(arrayData[-2])
                    p.pose.orientation.x = 0.0
                    p.pose.orientation.y = 0.0
                    p.pose.orientation.z = 0.0
                    p.pose.orientation.w = 1.0
                    self.dwm_tag_publisher_.publish(p)
                    # derive twist using previous p
                    self.dwm_twist_publisher_.publish(self.publish_derived_twist(p, new_time-self.last_time))
                    self.previous_p = p
                    self.last_time = new_time
                    # self.broadcast_transform()
                except:
                    self.node.get_logger().info('Could not make PoseStamped or TwistStamped message')
    def publish_derived_twist(self, p, time_since_last_pose):
        t = TwistStamped()
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id = "ips"
        
        # calculate linear velocities
        t.twist.linear.x = (p.pose.position.x - self.previous_p.pose.position.x)/time_since_last_pose
        t.twist.linear.y = (p.pose.position.y - self.previous_p.pose.position.y)/time_since_last_pose
        t.twist.linear.z = (p.pose.position.z - self.previous_p.pose.position.z)/time_since_last_pose
        # calculate angular velocities
        t.twist.angular.x = 0.0
        t.twist.angular.y = 0.0
        t.twist.angular.z = 0.0 #(self.p.pose.position.z - self.previous_p.pose.position.z)/time_since_last_pose
        return t
