#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import time
import xml.etree.ElementTree as ET
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

class Controller(Node):

    def __init__(self):
        super().__init__('urdf_prefix')
        self.prefix = self.declare_parameter('prefix', '')
        self.filename = self.declare_parameter('filename', 'robot')

        self.sub = self.create_subscription(String, 'in_robot_description', self.sub_callback, 1)

        # qos_profile = QoSProfile(depth=10)
        # qos_profile.reliability = QoSReliabilityPolicy.RELIABLE
        # qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL


    def sub_callback(self, msg):

        self.get_logger().info('Got robot description urdf, writing to /tmp/%s.urdf' % self.filename.value)
        tree = ET.ElementTree(ET.fromstring(msg.data))
        root = tree.getroot()

        for i in root.iter():
            #if i.tag in ['link', 'joint', 'sensor', 'camera']:
            if i.tag in ['link', 'joint']:
                i.attrib['name'] = self.prefix.value + i.attrib['name']

            elif i.tag in ['parent', 'child']:
                i.attrib['link'] = self.prefix.value + i.attrib['link']

            elif i.tag == 'gazebo':
                if 'reference' in i.attrib:
                    i.attrib['reference'] = self.prefix.value + i.attrib['reference']

            elif i.tag == 'plugin':
                if i.attrib['filename'] == 'libgazebo_ros_ray_sensor.so':
                    x = i.find('frame_name')
                    x.text = self.prefix.value + x.text

                elif i.attrib['filename'] == 'libgazebo_ros_force_based_move.so':
                    x = i.find('robot_base_frame')
                    x.text = self.prefix.value + x.text

                elif i.attrib['filename'] == 'libgazebo_ros_p3d.so':
                    x = i.find('body_name')
                    x.text = self.prefix.value + x.text

                elif i.attrib['filename'] == 'libgazebo_ros_joint_state_publisher.so':
                    x = i.find('joint_name')
                    x.text = self.prefix.value + x.text

                elif i.attrib['filename'] == 'libgazebo_ros_lifter.so':
                    x = i.find('joint_name')
                    x.text = self.prefix.value + x.text

                elif i.attrib['filename'] == 'libgazebo_ros_camera.so':
                    x = i.find('frame_name')
                    if x != None:
                        x.text = self.prefix.value + x.text


        tree.write('/tmp/%s.urdf' % self.filename.value)
        # msg.data = '<?xml version="1.0" ?>' + ET.tostring(root, encoding='unicode', method='xml')
        # self.pub.publish(msg)

        time.sleep(1)
        self.destroy_node()
        exit(0)
        

def main():

    rclpy.init()

    controller = Controller()
    rclpy.spin(controller)


if __name__ == '__main__':
    main()
    
    



