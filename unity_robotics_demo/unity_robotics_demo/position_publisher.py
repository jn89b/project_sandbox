#!/usr/bin/env python

import rclpy
import math
from unity_robotics_demo_msgs.msg import PosRot

from rclpy.node import Node

"""
Subscribe to actual position from unity asset

Publish position to Unity Asset 
"""

def to_quaternion(roll = 0.0, pitch = 0.0, yaw = 0.0):
    """
    Convert degrees to quaternions
    """
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]


class PositionPublisher(Node):
    
        def __init__(self):
            super().__init__('position_publisher')
            self.publisher_ = self.create_publisher(PosRot, 'new_position', 10)
            
            timer_period = 0.1  # seconds
   
            self.unity_asset_position_sub  = self.create_subscription(
                PosRot, 'pos_rot', self.unity_asset_position_cb, 10)

            self.current_position = PosRot()

            # self.timer = self.create_timer(timer_period, self.timer_callback)

            self.velocity = 0.05
        
            self.i = 0
            # self.do_publish()
    
        def unity_asset_position_cb(self, msg):
            # self.get_logger().info(f'Unity Asset Position: {msg}')
            self.current_position = msg

        def do_publish(self, current_time, velocity):
            # if self.i == 0:
            pos_rot = PosRot()
            pos_rot.pos_x = self.current_position.pos_x + velocity * current_time
            pos_rot.pos_y = self.current_position.pos_y + velocity * current_time 
            pos_rot.pos_z = self.current_position.pos_z + velocity * current_time
            # pos_rot.rot_x = self.current_position.rot_x
            # pos_rot.rot_y = self.current_position.rot_y
            # pos_rot.rot_z = self.current_position.rot_z
            # pos_rot.rot_w = self.current_position.rot_w
            self.get_logger().info(f'Publishing: {pos_rot}')
            self.publisher_.publish(pos_rot)

def main():
    rclpy.init()
    position_pub = PositionPublisher()

    time_now = position_pub.get_clock().now().nanoseconds / 1e9
    while rclpy.ok():
        
        current_time = position_pub.get_clock().now().nanoseconds / 1e9
        #check if current time is modulu of 5 seconds

        dt = current_time - time_now    
        rclpy.spin_once(position_pub)

        position_pub.do_publish(dt, position_pub.velocity)

    # position_pub.destroy_node()
    # rclpy.shutdown()

if __name__ == '__main__':
    main()



            
