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

import time, serial, os, sys, random
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration


from dwm1001_ros2.sensor.SensorService import dwm1001_sensor

class dwm1001_node(Node):

    def __init__(self):
        super().__init__('dwm1001')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('serial_number', None),
                ('device_name', None),
                ('tag_name', None),
                ('tag_location', None),
                ('topic_prefix', None)
            ])

    def setup(self):
        self.param = self.get_parameters(['serial_number', 'device_name', 'tag_name','tag_location','topic_prefix'])

        self.sensor = dwm1001_sensor(self, self.param)
        self.sensor.setup()
        self.sensor.initialize_dwm1001_api()
        self.sensor.get_data()


        

def main(args=None):
    rclpy.init(args=args)

    dwm_device = dwm1001_node()
    dwm_device.setup()

    rclpy.spin(dwm_device)

    # # destroy node
    dwm_device.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
