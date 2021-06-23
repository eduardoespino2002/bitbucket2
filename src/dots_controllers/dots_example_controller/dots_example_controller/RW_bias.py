#!/usr/bin/env python3 
import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import qos_profile_system_default as qos_default

import math

from std_msgs.msg import Float64
from std_msgs.msg import Float32
from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
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

        self.gt_sub             = self.create_subscription(Odometry,'ground_truth',self.ground_truth_callback,qos_default)
        self.lifter_sub         = self.create_subscription(Bool, 'lifter', self.lifter_callback, qos_default)
        self.irtof_sub          = self.create_subscription(PointCloud2, 'sensor/scan', self.scan_callback, 
                                    qos_profile=qos_profile_sensor_data)

        self.lifter_pos_pub = self.create_publisher(Float32, 'lifter_pos', qos_default)
        self.command            = Twist()
        self.heading            = random.uniform(math.pi/2.0, 1.5*math.pi)
        self.rate               = 100.0
        self.dt                 = 1.0 / self.rate

        self.max_range          = 2.0
        self.min_range          = 0.13
        self.collision_range    = 0.3
        self.collision          = False 
        self.speed              = 0.5
        self.prox               = np.zeros(2)
        
        # Start off with forward motion

        self.command.linear.x   = self.speed
        self.L_bias             = False
        self.R_bias             = False
        self.target             = 0.0
        self.kp                 = 0.7
        self.box                = False
        self.inexit             = False
        self.yaw                = 0.0

        self.lifter = True
        self.lifter_min = 0.0
        self.lifter_max = 0.05
        self.lifter_vel = 0.1
        self.lifter_pos = self.lifter_min

        # Start a timer to run the control loop at 10Hz
        self.timer              = self.create_timer(0.1, self.control_loop_callback)

    def ground_truth_callback(self,msg):
        # Here we access the robot's position in space and its orientation, 
        # simulating sensors and signals in the arena. 
        pose_x = msg.pose.pose.position.x       # current x position 
        orientation_q = msg.pose.pose.orientation # current orientation as a quaternion in the form (x, y, z, w)
        # the following uses Euler angles to extract the orientation around the z axis (yaw)
        x = orientation_q.x
        y = orientation_q.y
        z = orientation_q.z
        w = orientation_q.w
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        self.yaw = np.arctan2(siny_cosp, cosy_cosp) # orienation around z axis

        ### The following is the algorithm which rules the robots

        # Robot in exit area should deliver box
        self.inexit = False
        if pose_x > 1.5:
            self.inexit = True

        # Pick up a box if you find one, and you don't already have one
        if pose_x < -1.0:
            self.box = True

        # If have a box but not in the delivery area, 
        # then bias movement to the right (towards delivery area)        
        if self.box == True and self.inexit == False:
            self.R_bias = True
 
        # If have a box and in the delivery area, 
        # stop moving to the right and drop box
        if self.box == True and self.inexit == True:
            self.box = False
            self.R_bias = False
 
        # If in the delivery area, without a box
        # move away from the delivery area (to the left)
        if self.box == False and self.inexit == True:
            self.L_bias = True 
        
        # If out of the delivery area, having been moving left,
        # and not facing towards the delivery area
        # then stop moving left and randomly walk
        if self.L_bias == True and self.inexit == False:
            if (self.target - self.yaw) < math.pi/4: 
                self.L_bias = False
 
        # If biased to move to the right then target heading
        # is towards the right (i.e. 0.0 deg)
        if self.R_bias ==True:
            target = 0.0
            self.target = target*math.pi/180.0 
            self.lifter = True
 
        # If biased to move to the left then target heading
        # is towards the left (i.e. 180 deg)
        if self.L_bias == True:
            target = 180.0
            self.target = target*math.pi/180.0
            self.lifter = False
    
    # Control for the lifter 
    def lifter_callback(self, msg):
        self.lifter = msg.data        

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
        self.prox   = np.zeros(2)
        for i in range(msg.width):
            # Points within the pointcloud are actually locations in 3D space in the scanner
            # frame. Each is a float32 taking 12 bytes. 
            # Extract point
            self.collision = False
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
                self.collision = True
            # Calculate a vector pointing towards the nearest obstacle
            nm          = np.array([x, y]) / dist
            nm_inv_len  = 1 - (dist - self.min_range) / (self.max_range - self.min_range)
            nm          = nm * nm_inv_len
            self.prox        += nm
 
    def control_loop_callback(self):
    # Random walk adds a random perturbation to the heading
        self.command.angular.z  = random.uniform(-math.pi / 2.0, math.pi / 2.0)

    # Collision avoidance 
        if self.collision:
            unit                    = -self.prox / np.linalg.norm(self.prox)
            self.command.linear.x   = unit[0] * self.speed
            self.command.linear.y   = unit[1] * self.speed
            self.command.angular.z  = random.uniform(-math.pi / 2.0, math.pi / 2.0)

    # If not colliding, check for biased movement commands
        if self.collision == False:
            if self.R_bias == True or self.L_bias == True:
                # Command change of heading towards target direction
                # Right = 0 deg, Left = 180 deg
                # With a PID controller, kp
                self.command.angular.z = self.kp*(self.target-self.yaw)
            # Command for x and y directions set using new heading (either rand or target)   
            self.command.linear.x   = math.cos(self.command.angular.z) * self.speed
            self.command.linear.y   = math.sin(self.command.angular.z) * self.speed        

        msg = Float32()
        msg.data = self.lifter_pos

        self.lifter_pos_pub.publish(msg) 
        self.cmd_vel_pub.publish(self.command)
 
def main():

    rclpy.init()
    controller = Controller()
    rclpy.spin(controller)
 
if __name__ == '__main__':

    main()

