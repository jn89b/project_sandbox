#!/usr/bin/env python

import rclpy
import math
import mavros
import numpy as np
from unity_robotics_demo_msgs.msg import PosRot

from rclpy.node import Node

from mavros.base import SENSOR_QOS

"""
Publish position of UAS from sensor information
"""

def euler_from_quaternion(x:float, y:float, z:float, w:float) -> tuple:
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

class UAVPosition(Node):
    def __init__(self):
        super().__init__('uav_position_publisher')
        self.publisher_ = self.create_publisher(PosRot, 'uav_position', 10)

        self.current_position = PosRot()

        # self.position_sub = self.create_subscription(
        #     mavros.local_position.PoseStamped, 
        #     'mavros/local_position/pose', 
        #     self.position_callback, 
        #     qos_profile=SENSOR_QOS)

    def position_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w

        roll, pitch, yaw = euler_from_quaternion(qx, qy, qz, qw)

        deg_roll = np.rad2deg(roll)
        deg_pitch = np.rad2deg(pitch)
        deg_yaw = np.rad2deg(yaw)
        
        #print roll pitch yaw in one line
        # print("roll is {}, pitch is {}, yaw is {}".format(deg_roll, deg_pitch, deg_yaw))
        # print("yaw is {}".format(deg_yaw))

        self.current_position.pos_x = x
        self.current_position.pos_y = y
        self.current_position.pos_z = z
        self.current_position.rot_x = qx
        self.current_position.rot_y = qy
        self.current_position.rot_z = qz
        self.current_position.rot_w = qw
        self.publisher_.publish(self.current_position)

def main(args=None):
    rclpy.init(args=args)
    uav_position_publisher = UAVPosition()

    while rclpy.ok():
        
        uav_position_publisher.current_position.pos_x = 25.0
        uav_position_publisher.current_position.pos_y = 2.0
        uav_position_publisher.current_position.pos_z = 3.0
        uav_position_publisher.current_position.rot_x = 0.0
        uav_position_publisher.current_position.rot_y = 0.0
        uav_position_publisher.current_position.rot_z = 0.0
        uav_position_publisher.current_position.rot_w = 1.0
        
        uav_position_publisher.publisher_.publish(uav_position_publisher.current_position)
        rclpy.spin_once(uav_position_publisher, timeout_sec=0.01)


    # rclpy.spin(uav_position_publisher)
    # uav_position_publisher.destroy_node()
    # rclpy.shutdown()

if __name__=='__main__':
    main()