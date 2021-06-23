#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import math
from std_msgs.msg import Float64
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
#from sensor_msgs import point_cloud2 as pc2

from geometry_msgs.msg import Twist
import sys
import numpy as np
import random

import struct


class Controller(Node):

    def __init__(self):
        # Create the controller node
        super().__init__('controller')

        # Publish to one topic, the command velocity
        self.cmd_vel_pub        = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscribe to the laser time of flight sensors topic. 
        # This must be set to the qos sensor profile since the data is sent as best effort, and the 
        # default qos is reliable, which produces no data traffic
        self.irtof_sub          = self.create_subscription(PointCloud2, 'sensor/scan', self.scan_callback, 
                                    qos_profile=qos_profile_sensor_data)

        self.command            = Twist()
        self.heading            = random.uniform(0, math.pi)


        self.max_range          = 2.0
        self.min_range          = 0.13
        self.collision_range    = 0.3
        self.speed              = 0.5
        # Start off with forward motion
        self.command.linear.x   = self.speed

        # Start a timer to run the control loop at 10Hz
        self.timer              = self.create_timer(0.1, self.control_loop_callback)

    def scan_callback(self, msg):
        # This function gets called with every new message on the laser scan topic.
        # Extract the data and work out if there is a collision, i.e range is less
        # than some amount, if so, select a new direction.

        # There are 16 sensors, make somewhere to store data from each one. The Gazebo
        # version of the laser scanner doesn't send data for points with no return detected
        # but the hardware does, so fill in the maximum range for each entry to start with.
        data        = np.zeros((16))
        data[:]     = self.max_range
        step        = math.pi / 8.0
        prox        = np.zeros(2)
        collision   = False
        for i in range(msg.width):
            # Points within the pointcloud are actually locations in 3D space in the scanner
            # frame. Each is a float32 taking 12 bytes. 
            # Extract point
            [x, y, z]   = struct.unpack('fff', bytes(msg.data[i * msg.point_step : i * msg.point_step + 12]))
            # Get angle and see which sensor it was
            th          = math.atan2(y, x)
            if th < 0.0:
                th += 2 * math.pi
            idx         = int(round(th / step))
            # Get the distance and save it in the array
            dist        = math.sqrt(x**2 + y**2)
            data[idx]   = dist
            # Check if there is a collision
            if dist < self.collision_range:
                collision = True
            # Calculate a vector pointing towards the nearest obstacle
            nm          = np.array([x, y]) / dist
            nm_inv_len  = 1 - (dist - self.min_range) / (self.max_range - self.min_range)
            nm          = nm * nm_inv_len
            prox        += nm

        if collision:
            # If too close to something, get the unit vector pointing away from the 
            # nearest obstacle and use that to set the velocity, with a bit and random
            # angular velocity added
            unit                    = -prox / np.linalg.norm(prox)
            self.command.linear.x   = unit[0] * self.speed
            self.command.linear.y   = unit[1] * self.speed
            self.command.angular.z  = random.uniform(-math.pi / 2.0, math.pi / 2.0)


    def control_loop_callback(self):
        self.cmd_vel_pub.publish(self.command)
        

def main():
    rclpy.init()
    controller = Controller()
    rclpy.spin(controller)


if __name__ == '__main__':
    main()
    
    



