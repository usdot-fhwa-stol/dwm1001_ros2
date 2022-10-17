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

# This is a simple node to get corrected heading between two dwm1001 nodes.
# It does not pull their positions from the transform tree - so actual orientation
# relative to the robot frame is unknown. It assumes that one is in the front
# and one is in the rear of the robot. If actual sensor locations on the robot
# are correctly measured and known, the yaw calculations can be rotated based on these
# known locations using the '_rotate_yaw()' function.

import time, transforms3d, math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg  import PoseStamped
from geometry_msgs.msg  import TwistStamped
from geometry_msgs.msg  import Quaternion

class dwm1001_math(Node):

    def __init__(self):
        super().__init__('dwm1001_math')

        topic_prefix = 'dwm1001/corrected/'

        self.front_ips_pose_subscriber = self.create_subscription(PoseStamped,'dwm1001/front/pose',self.front_callback,10)
        self.rear_ips_pose_subscriber = self.create_subscription(PoseStamped,'dwm1001/rear/pose',self.rear_callback,10)
        # self.front_ips_twist_subscriber = self.create_subscription(TwistStamped,'dwm1001_front/twist',self.rear_callback,10)
        # self.rear_ips_twist_subscriber = self.create_subscription(TwistStamped,'dwm1001_rear/twist',self.rear_callback,10)
        self.corrected_pose_publisher = self.create_publisher(PoseStamped,topic_prefix+'pose',10)
        self.corrected_twist_publisher = self.create_publisher(TwistStamped,topic_prefix+'twist',10)


        self.front_x = 0.0
        self.front_y = 0.0
        self.rear_x = 0.0
        self.rear_y = 0.0
        self.previous_p = PoseStamped()

        self.previous_pose_position_x = 0.0
        self.previous_pose_position_y = 0.0
        self.y = 0.0
        self.previous_yaw = 0.0
        self.previous_time = time.time()

        corrected_pose_timer_period = 0.1

        self.timer = self.create_timer(corrected_pose_timer_period, self.timer_callback)

    def timer_callback(self):
        p = PoseStamped()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = 'ips_centroid'
        # pose is based around centroid between two ips anchors
        p.pose.position.x = (self.front_x + self.rear_x) / 2
        p.pose.position.y = (self.front_y + self.rear_y) / 2
        p.pose.position.z = 0.0
        self.y = self._get_yaw_from_points(self.front_x,self.rear_x,self.front_y,self.rear_y)
        self.y = self._rotate_yaw(self.y,0.0)

        q_arr = transforms3d.euler.euler2quat(0.0,0.0,self.y)
        p.pose.orientation.x = q_arr[0]
        p.pose.orientation.y = q_arr[1]
        p.pose.orientation.z = q_arr[2]
        p.pose.orientation.w = q_arr[3]
        # publish corrected pose
        self.corrected_pose_publisher.publish(p)
        # publish corrected twist
        now_time = time.time()
        t = self._derive_twist(p, now_time-self.previous_time)
        self.corrected_twist_publisher.publish(t)

        # store vals for next time
        self.previous_yaw = self.y
        self.previous_p = p
        self.previous_pose_position_x = p.pose.position.x
        self.previous_pose_position_y = p.pose.position.y
        self.previous_time = now_time

    # gets yaw from two points - with north being 0 radian
    def _get_yaw_from_points(self,x1,x2,y1,y2):
        rise = y1-y2
        run = x1-x2

        if run > 0:
            yaw = math.pi/2 - math.atan(rise/run)
        elif run < 0:
            yaw = 3*math.pi/2 - math.atan(rise/run)
        else:
            if rise >= 0:
                yaw = 0.0
            else:
                yaw = math.pi

        return yaw

    # rotate yaw
    # will fail if rotation >= 2*pi in either direction
    def _rotate_yaw(self, yaw, rotation):
        y = yaw + rotation
        if y < 0.0:
            y = 2*math.pi - y
        elif y >= 2*math.pi:
            y = y - 2*math.pi
        return y

    def _derive_twist(self, p, time_since_last_pose):
        t = TwistStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "ips"
        
        # calculate linear velocities
        t.twist.linear.x = (p.pose.position.x - self.previous_p.pose.position.x)/time_since_last_pose
        t.twist.linear.y = (p.pose.position.y - self.previous_p.pose.position.y)/time_since_last_pose
        t.twist.linear.z = (p.pose.position.z - self.previous_p.pose.position.z)/time_since_last_pose
        # calculate angular velocities
        t.twist.angular.x = 0.0
        t.twist.angular.y = 0.0
        t.twist.angular.z = 0.0 #(self.y - self.previous_yaw)/time_since_last_pose
        return t



    def front_callback(self, msg):
        self.front_x = msg.pose.position.x
        self.front_y = msg.pose.position.y

    def rear_callback(self, msg):
        self.rear_x = msg.pose.position.x
        self.rear_y = msg.pose.position.y

def main(args=None):
    rclpy.init(args=args)

    dwm_math_node = dwm1001_math()
    # dwm_math_node.setup()

    rclpy.spin(dwm_math_node)

    # # destroy node
    dwm_math_node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main