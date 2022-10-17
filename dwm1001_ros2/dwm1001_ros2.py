#!/usr/bin/env python3

import time, serial, os, sys, random
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration


from dwm1001_foxy.sensor.SensorService import dwm1001_sensor

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