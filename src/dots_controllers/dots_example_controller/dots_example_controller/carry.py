#!/usr/bin/env python3

import sys
print(sys.path)

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
import math

import tf2_ros
import sensor_msgs.msg
import nav_msgs.msg
import std_msgs.msg
import rosgraph_msgs.msg
#from sensor_msgs import point_cloud2 as pc2
import dots_interfaces.msg
import geometry_msgs.msg
import numpy as np
import random

import struct
import operator
import py_trees
import py_trees_ros



# The behaviour of sequence without memory in the current py-trees is not 
# semantically correct, it should be the inverse of sel without mem. 
# Subclassing and adding inverters makes a fake version with the correct semantics
class FakeSequence(py_trees.decorators.Inverter):
    def __init__(self, **kwargs):
        super(FakeSequence, self).__init__(child=py_trees.composites.Selector(**kwargs))
    def add_child(self, child):
        self.decorated.add_child(py_trees.decorators.Inverter(child))



def rpy_from_quaternion(quaternion):
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

def quaternion_from_rpy(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = sr * cp * cy - cr * sp * sy
    q[1] = cr * sp * cy + sr * cp * sy
    q[2] = cr * cp * sy - sr * sp * cy
    q[3] = cr * cp * cy + sr * sp * sy
    return q



class G:
    # Global values
    max_linear_velocity     = 0.5
    max_angular_velocity    = 3.0

    # P controllers
    Pv                      = 2.0
    Pw                      = 2.0
    carriers                = [100, 101, 102, 103, 104]

    # Perimeter amera angles
    camera_angles           = [0.2967, 2.251, -2.251, -0.2967]


    # Alignment parameters
    max_align_velocity      = 0.1
    align_tolerance         = 0.02
    max_under_velocity      = 0.3
    max_carry_velocity      = 0.3

    # Transform from the ID fiducial to the pre-dock position goal. This is goal in the frame
    # of the observing camera
    fid_to_goal = geometry_msgs.msg.TransformStamped()
    fid_to_goal.transform.translation.x     = 0.0
    fid_to_goal.transform.translation.y     = -0.18
    fid_to_goal.transform.translation.z     = 0.20
    q                                       = quaternion_from_rpy(0.0, np.pi/2, np.pi/2)
    fid_to_goal.transform.rotation.x        = q[0]
    fid_to_goal.transform.rotation.y        = q[1]
    fid_to_goal.transform.rotation.z        = q[2]
    fid_to_goal.transform.rotation.w        = q[3]




class Tag_transform:
    def __init__(self, tag, transform):
        self.tag        = tag
        self.transform  = transform



class Lift(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Lift, self).__init__(name)
    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.lifter_pub = self.node.create_publisher(std_msgs.msg.Bool, 'lifter', 10)
    def initialise(self):
        self.delay = 10
    def update(self):
        msg = std_msgs.msg.Bool()
        msg.data = True
        self.lifter_pub.publish(msg)
        self.delay -= 1
        if self.delay == 0:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

class Lower(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Lower, self).__init__(name)
    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.lifter_pub = self.node.create_publisher(std_msgs.msg.Bool, 'lifter', 10)
    def initialise(self):
        self.delay = 10
    def update(self):
        msg = std_msgs.msg.Bool()
        msg.data = False
        self.lifter_pub.publish(msg)
        self.delay -= 1
        if self.delay == 0:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class Pick_random_direction(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Pick_random_direction, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name)
        self.blackboard.register_key(key='random_cmd_vel', access=py_trees.common.Access.WRITE)
    def update(self):
        cv              = geometry_msgs.msg.Twist()
        angle           = random.uniform(-math.pi, math.pi)
        cv.linear.x     = G.max_linear_velocity * math.cos(angle)
        cv.linear.y     = G.max_linear_velocity * math.sin(angle)
        cv.angular.z    = 0.0
        self.blackboard.random_cmd_vel = cv
        return py_trees.common.Status.SUCCESS


        
        
class Process_irtof(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Process_irtof, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name)
        self.blackboard.register_key(key='irtof', access=py_trees.common.Access.READ)
        self.blackboard.register_key(key='obstacle', access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key='collision', access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key='avoid_cmd_vel', access=py_trees.common.Access.WRITE)

        self.max_range          = 2.0
        self.min_range          = 0.13
        self.collision_range    = 0.3
        self.speed              = 0.5


    def setup(self, **kwargs):
        try:
            self.node = kwargs['node']
        except KeyError as e:
            self.node = None
            print(e)

    def update(self):
        # Return running if the blackboard entry does not yet exist
        try:
            irtof = self.blackboard.irtof
        except KeyError:
            return py_trees.common.Status.RUNNING

        # There are 16 sensors, make somewhere to store data from each one. The Gazebo
        # version of the laser scanner doesn't send data for points with no return detected
        # but the hardware does, so fill in the maximum range for each entry to start with.
        msg = irtof
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

        self.blackboard.obstacle    = prox
        self.blackboard.collision   = collision

        cv = geometry_msgs.msg.Twist()

        coll_vector     = -prox / np.linalg.norm(prox)
        cv.linear.x     = coll_vector[0] * G.max_linear_velocity
        cv.linear.y     = coll_vector[1] * G.max_linear_velocity
        cv.angular.z    = random.uniform(-G.max_angular_velocity / 2.0, G.max_angular_velocity / 2.0)

        self.blackboard.avoid_cmd_vel = cv

        #self.node.get_logger().info('%s' % irtof)
        return py_trees.common.Status.SUCCESS
        
class Process_vision(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Process_vision, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name)
        self.blackboard.register_key(key='cam0_tags', access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key='under_carrier', access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key='centre_cmd_vel', access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key='seen_carrier', access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key='align_cmd_vel', access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key='aligned_carrier', access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key='under_cmd_vel', access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key='centered_carrier', access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key='zero_cmd_vel', access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key='home_cmd_vel', access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key='away_cmd_vel', access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        try:
            self.node = kwargs['node']
        except KeyError as e:
            self.node = None
            print('Should have node! %s' % e)
            sys.exit(1)

        # The robot name is always used for the namespace, get the name by removing the 
        # leading '/'
        self.robot_name     = self.node.get_namespace()[1:]

        # Listen to the lists of tags seen by each camera
        self.cam0_tags_sub  = self.node.create_subscription(dots_interfaces.msg.TagArray, 
                            'cam0_tags', self.cam0_tags_callback, qos_profile_system_default)
        self.cam1_tags_sub  = self.node.create_subscription(dots_interfaces.msg.TagArray, 
                            'cam1_tags', self.cam1_tags_callback, qos_profile_system_default)
        self.cam2_tags_sub  = self.node.create_subscription(dots_interfaces.msg.TagArray, 
                            'cam2_tags', self.cam2_tags_callback, qos_profile_system_default)
        self.cam3_tags_sub  = self.node.create_subscription(dots_interfaces.msg.TagArray, 
                            'cam3_tags', self.cam3_tags_callback, qos_profile_system_default)
        self.cam4_tags_sub  = self.node.create_subscription(dots_interfaces.msg.TagArray, 
                            'cam4_tags', self.cam4_tags_callback, qos_profile_system_default)

        self.compass_sub    = self.node.create_subscription(std_msgs.msg.Float32, 
                            'sensor/compass', self.compass_callback, qos_profile_sensor_data)




        # Transform listener and broadcaster
        self.tfbuffer   = tf2_ros.Buffer()
        self.listener   = tf2_ros.TransformListener(self.tfbuffer, self.node)
        self.br         = tf2_ros.TransformBroadcaster(self.node)


        # Local transform
        self.bc = tf2_ros.BufferCore(rclpy.duration.Duration(seconds=10.0))


        self.blackboard.zero_cmd_vel    = geometry_msgs.msg.Twist()
        self.centre_transform           = None
        self.align_transform            = None
        self.blackboard.seen_carrier    = False
        self.blackboard.aligned_carrier = False
        self.blackboard.under_carrier   = False
        self.blackboard.centered_carrier= False
        self.docking_cam                = None
        self.ticks_since_not_seen       = 0
        self.ticks_centering            = 0

    def get_past(self, delta):
        # Look slightly into the past for the transforms. This seems to fail sometimes
        # with a negative time. Presumably a callback arrives before the node has got
        # a time from /clock or from the system (only seen happen in simulation)
        now = self.node.get_clock().now()
        d = rclpy.duration.Duration(seconds=delta)
        try:
            past = now - d
        except ValueError as e:
            past = now
        return past

    def compass_callback(self, msg):
        self.heading = msg.data
        hcv = geometry_msgs.msg.Twist()
        hcv.linear.x = G.max_carry_velocity * np.cos(-self.heading)
        hcv.linear.y = G.max_carry_velocity * np.sin(-self.heading)
        self.blackboard.home_cmd_vel = hcv

        acv = geometry_msgs.msg.Twist()
        acv.linear.x = G.max_linear_velocity * np.cos(np.pi - self.heading)
        acv.linear.y = G.max_linear_velocity * np.sin(np.pi - self.heading)
        self.blackboard.away_cmd_vel = acv


    def cam_tags_callback(self, msg, cam):
        # We can only dock using a single camera. If we are in the process of docking, ignore
        # all other cameras
        if self.blackboard.seen_carrier and (self.docking_cam != cam):
            return

        #self.blackboard.seen_carrier = False
        past = self.get_past(0.1)
        transforms = []
        cam_prefix = '%s_cam%d_' % (self.robot_name, cam)
        for tag in msg.data:
            if tag.id in G.carriers:
                #self.node.get_logger().info('Tag %d in carriers' % tag.id)
                try:
                    transform = self.tfbuffer.lookup_transform(
                        cam_prefix + 'link', 
                        cam_prefix + 'fid%03d' % tag.id, past)
                except (tf2_ros.LookupException, 
                        tf2_ros.ConnectivityException, 
                        tf2_ros.ExtrapolationException) as e:
                    pass
                    #self.node.get_logger().info('Failed tf lookup %s' % e)
                else:
                    #self.node.get_logger().info('Tr %s' % transform)
                    transforms.append(Tag_transform(tag, transform))

        # Choose the closest tag and set that as the target
        if len(transforms):
            transforms.sort(key=lambda t: np.linalg.norm(np.array(( t.transform.transform.translation.x, 
                                                                    t.transform.transform.translation.y, 
                                                                    t.transform.transform.translation.z))))
            # Apply the goal transform
            t = transforms[0]

            # All this to just apply a transform manually!
            # https://answers.ros.org/question/289323/simple-tf-transform-without-broadcasterlistener/
            t.transform.header.stamp        = rclpy.time.Time(seconds=0).to_msg()
            G.fid_to_goal.header.stamp      = rclpy.time.Time(seconds=0).to_msg()
            G.fid_to_goal.header.frame_id   = cam_prefix + 'fid%03d' % t.tag.id
            G.fid_to_goal.child_frame_id    = cam_prefix + 'target'

            self.bc.set_transform(t.transform, 'default_authority')
            self.bc.set_transform(G.fid_to_goal, 'default_authority')

            nt              = self.bc.lookup_transform_core(cam_prefix + 'link', cam_prefix + 'target', rclpy.time.Time(seconds=0))
            nt.header.stamp = self.node.get_clock().now().to_msg()
            t.transform     = nt

            self.align_transform            = t
            self.br.sendTransform(t.transform)
            self.blackboard.seen_carrier    = True
            self.docking_cam                = cam
            self.ticks_since_not_seen       = 0
            #self.node.get_logger().info('Cam %d seen %s' % (cam, self.blackboard.seen_carrier))



    def cam0_tags_callback(self, msg):
        self.cam_tags_callback(msg, 0)

    def cam1_tags_callback(self, msg):
        self.cam_tags_callback(msg, 1)

    def cam2_tags_callback(self, msg):
        self.cam_tags_callback(msg, 2)

    def cam3_tags_callback(self, msg):
        self.cam_tags_callback(msg, 3)




    def cam4_tags_callback(self, msg):
        #self.node.get_logger().info('Got cam4_tags %s %s' % (msg.data, self.node.get_clock().now()))

        past = self.get_past(0.1)
        #self.blackboard.under_carrier = False
        if len(msg.data) == 1:
            try:
                self.centre_transform = self.tfbuffer.lookup_transform(
                    '%s_base_link' % self.robot_name, 
                    '%s_cam4_fid%03d' % (self.robot_name, msg.data[0].id), past)
            except (tf2_ros.LookupException, 
                    tf2_ros.ConnectivityException, 
                    tf2_ros.ExtrapolationException) as e:
                #self.node.get_logger().info('Failed tf lookup %s' % e)
                pass
            else:
                self.blackboard.under_carrier = True
                self.ticks_since_not_seen       = 0



    def update(self):
        self.ticks_since_not_seen += 1
        if self.ticks_since_not_seen > 10:
            # Stop this behaviour if its been too long since seeing target
            self.blackboard.seen_carrier = False
            self.blackboard.aligned_carrier = False
            self.blackboard.under_carrier = False
            self.blackboard.centered_carrier = False
            self.centre_transform = None
            self.ticks_centering = 0
            return py_trees.common.Status.SUCCESS

        if self.centre_transform != None:
            # We're under a carrier, calculate the velocity to centre
            self.blackboard.aligned_carrier = False
            self.ticks_centering += 1
            if self.ticks_centering > 20:
                self.blackboard.centered_carrier = True

            r,p,y   = rpy_from_quaternion(self.centre_transform.transform.rotation)
            tr      = self.centre_transform.transform.translation

            #self.node.get_logger().info('% 8f % 8f % 8f : % 8f % 8f % 8f' % (tr.x, tr.y, tr.z, r, p, y))
            #self.node.get_logger().info('%s %s %s %d %s' % (self.docking_cam, self.blackboard.seen_carrier, self.ticks_since_not_seen, tag.id, self.blackboard.aligned_carrier))

            # Simple P controller to move to centre of carrier
            cmd_vel = geometry_msgs.msg.Twist()
            cmd_vel.linear.x    = G.Pv * tr.x
            cmd_vel.linear.y    = G.Pv * tr.y
            #cmd_vel.angular.z   = G.Pw * y
            cmd_vel.angular.z   = 0.0
            # Limit velocities
            cmd_vel.linear.x    = np.clip(cmd_vel.linear.x, -G.max_under_velocity, G.max_under_velocity)
            cmd_vel.linear.y    = np.clip(cmd_vel.linear.y, -G.max_under_velocity, G.max_under_velocity)
            cmd_vel.angular.z   = np.clip(cmd_vel.angular.z, -G.max_angular_velocity, G.max_angular_velocity)

            self.blackboard.centre_cmd_vel = cmd_vel

        if self.align_transform != None:
            # We've seen a carrier with an appropriate ID, align in preparation 
            # for docking and centering


            tag     = self.align_transform.tag
            r,p,y   = rpy_from_quaternion(self.align_transform.transform.transform.rotation)
            tr      = self.align_transform.transform.transform.translation
            # Clear the detection flag
            #self.blackboard.seen_carrier = False

            # Two possible approaches:
            # 1) Apply a static transform to the detected tag transform to give a goal pose in 
            #   front of the tag, then use a P controller to servo to that pose
            #
            # 2) Directly operate on the tag data.
            #   a) Ensure tag is horizontally in centre of camera (control rotation)
            #   b) Ensure robot is aligned with tag normal (control lateral to camera movement)
            #   c) Ensure robot is fixed distance from tag
            # Each of these can be done using only the calculated corner positions of the tag.
            # 
            #
            # Approach 1 depends on the stability of the transform calculated by the Aruco tag detection
            # code. Approach 2 is more direct from the sensed data, and is may be more robust.
            # We use approach 1 here - quick hack, would benefit from ekf
            
            # Simple P controller to move to centre of carrier
            # rotate to camera angle
            th = G.camera_angles[self.docking_cam]
            cmd_vel = geometry_msgs.msg.Twist()
            cmd_vel.linear.x    = G.Pv * (np.cos(th) * tr.x - np.sin(th) * tr.y)
            cmd_vel.linear.y    = G.Pv * (np.sin(th) * tr.x + np.cos(th) * tr.y)
            cmd_vel.angular.z   = G.Pw * y
            # Limit velocities
            cmd_vel.linear.x    = np.clip(cmd_vel.linear.x, -G.max_align_velocity, G.max_align_velocity)
            cmd_vel.linear.y    = np.clip(cmd_vel.linear.y, -G.max_align_velocity, G.max_align_velocity)
            cmd_vel.angular.z   = np.clip(cmd_vel.angular.z, -G.max_angular_velocity, G.max_angular_velocity)

            self.blackboard.align_cmd_vel = cmd_vel

            # Determine if we are stable enough to move under by checking if position error
            # small enough - should this wait for several ticks?
            #self.blackboard.aligned_carrier = False
            if self.blackboard.aligned_carrier or math.sqrt(tr.x * tr.x + tr.y * tr.y) < G.align_tolerance:
                self.blackboard.aligned_carrier = True
                cmd_vel.linear.x    = G.max_under_velocity * np.cos(th)
                cmd_vel.linear.y    = G.max_under_velocity * np.sin(th)
                cmd_vel.angular.z   = 0.0

                self.blackboard.under_cmd_vel = cmd_vel

        self.node.get_logger().info('%s %d %s %s %s' % (self.docking_cam, self.ticks_since_not_seen, self.blackboard.seen_carrier, self.blackboard.aligned_carrier, self.blackboard.under_carrier))
            


        return py_trees.common.Status.SUCCESS




def create_root():
    root = py_trees.composites.Parallel(
        name    = 'Simple BT controller',
        policy  = py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False)
    )

    topics2bb   = py_trees.composites.Sequence(
        name    = 'Topics2BB',
        memory  = True
    )

    irtof2bb    = py_trees_ros.subscribers.ToBlackboard(
        name                    = 'irtof2bb',
        topic_name              = 'sensor/scan',
        topic_type              = sensor_msgs.msg.PointCloud2,
        qos_profile             = qos_profile_sensor_data,
        blackboard_variables    = {'irtof' : None}
    )

    odom2bb     = py_trees_ros.subscribers.ToBlackboard(
        name                    = 'odom2bb',
        topic_name              = 'odom',
        topic_type              = nav_msgs.msg.Odometry,
        qos_profile             = qos_profile_system_default,
        blackboard_variables    = {'odom' : None}
    )

    #priorities      = py_trees.composites.Sequence(name='Priorities', memory=False)
    priorities      = FakeSequence(name='FS Priorities', memory=False)



    proc_irtof      = Process_irtof(name='Proc irtof')
    proc_vision     = Process_vision(name='Proc vision')

    find_carrier    = py_trees.composites.Selector(name='Find carrier', memory=False)


    finish_centering = py_trees.composites.Sequence('Finish centering', memory=True)
    check_if_centered   = py_trees.behaviours.CheckBlackboardVariableValue(
        name    = 'Centered under carrier?',
        check   = py_trees.common.ComparisonExpression(
            variable    = 'centered_carrier', 
            value       = True, 
            operator    = operator.eq)
    )
    zero_cmd_vel  = py_trees_ros.publishers.FromBlackboard(
        name                = 'zero cmd_vel',
        topic_name          = 'cmd_vel',
        topic_type          = geometry_msgs.msg.Twist,
        qos_profile         = qos_profile_system_default,
        blackboard_variable = 'zero_cmd_vel'
    )

    centre_if_under = py_trees.composites.Sequence('Centre if under', memory=True)
    centre_if_under_sir = py_trees.decorators.SuccessIsRunning(centre_if_under)

    check_if_under  = py_trees.behaviours.CheckBlackboardVariableValue(
        name    = 'Under carrier?',
        check   = py_trees.common.ComparisonExpression(
            variable    = 'under_carrier', 
            value       = True, 
            operator    = operator.eq)
    )
    centre_cmd_vel  = py_trees_ros.publishers.FromBlackboard(
        name                = 'centre cmd_vel',
        topic_name          = 'cmd_vel',
        topic_type          = geometry_msgs.msg.Twist,
        qos_profile         = qos_profile_system_default,
        blackboard_variable = 'centre_cmd_vel'
    )

    under_if_aligned = py_trees.composites.Sequence('Under if aligned', memory=True)
    under_if_aligned_sir = py_trees.decorators.SuccessIsRunning(under_if_aligned)

    check_if_aligned = py_trees.behaviours.CheckBlackboardVariableValue(
        name    = 'Aligned to carrier?',
        check   = py_trees.common.ComparisonExpression(
            variable    = 'aligned_carrier', 
            value       = True, 
            operator    = operator.eq)
    )
    under_cmd_vel  = py_trees_ros.publishers.FromBlackboard(
        name                = 'under cmd_vel',
        topic_name          = 'cmd_vel',
        topic_type          = geometry_msgs.msg.Twist,
        qos_profile         = qos_profile_system_default,
        blackboard_variable = 'under_cmd_vel'
    )


    align_if_found  = py_trees.composites.Sequence('Align if found', memory=True)
    align_if_found_sir = py_trees.decorators.SuccessIsRunning(align_if_found)

    check_if_found  = py_trees.behaviours.CheckBlackboardVariableValue(
        name    = 'Seen carrier?',
        check   = py_trees.common.ComparisonExpression(
            variable    = 'seen_carrier',
            value       = True,
            operator    = operator.eq)
    )
    align_cmd_vel   = py_trees_ros.publishers.FromBlackboard(
        name                = 'align cmd_vel',
        topic_name          = 'cmd_vel',
        topic_type          = geometry_msgs.msg.Twist,
        qos_profile         = qos_profile_system_default,
        blackboard_variable = 'align_cmd_vel'
    )


    avoid_collision  = py_trees.composites.Sequence('Coll avoid', memory=True)
    avoid_collision_sir = py_trees.decorators.SuccessIsRunning(avoid_collision)

    coll_check  = py_trees.behaviours.CheckBlackboardVariableValue(
        name    = 'Obstacles?',
        check   = py_trees.common.ComparisonExpression(
            variable    = 'collision', 
            value       = True, 
            operator    = operator.eq)
    )
    avoid_cmd_vel = py_trees_ros.publishers.FromBlackboard(
        name                = 'avoid cmd_vel',
        topic_name          = 'cmd_vel',
        topic_type          = geometry_msgs.msg.Twist,
        qos_profile         = qos_profile_system_default,
        blackboard_variable = 'avoid_cmd_vel'
    )

    random_wander   = py_trees.composites.Sequence('Wander', memory=True)
    random_wander_sir = py_trees.decorators.SuccessIsRunning(random_wander)

    pick_direction  = Pick_random_direction('Pick direction')
    random_cmd_vel  = py_trees_ros.publishers.FromBlackboard(
        name                = 'random cmd_vel',
        topic_name          = 'cmd_vel',
        topic_type          = geometry_msgs.msg.Twist,
        qos_profile         = qos_profile_system_default,
        blackboard_variable = 'random_cmd_vel'
    )
    wander_delay    = py_trees.behaviours.TickCounter(name='Wander delay', duration=50)

    fetch           = py_trees.composites.Sequence('Fetch', memory=True)
    lift            = Lift('Raise lifter')
    lower           = Lower('Lower lifter')
    home_cmd_vel    = py_trees_ros.publishers.FromBlackboard(
        name                = 'home cmd_vel',
        topic_name          = 'cmd_vel',
        topic_type          = geometry_msgs.msg.Twist,
        qos_profile         = qos_profile_system_default,
        blackboard_variable = 'home_cmd_vel'
    )

    home_delay      = py_trees.behaviours.TickCounter(name='Home delay', duration=60)
    away_cmd_vel    = py_trees_ros.publishers.FromBlackboard(
        name                = 'away cmd_vel',
        topic_name          = 'cmd_vel',
        topic_type          = geometry_msgs.msg.Twist,
        qos_profile         = qos_profile_system_default,
        blackboard_variable = 'away_cmd_vel'
    )
    away_delay      = py_trees.behaviours.TickCounter(name='Away delay', duration=30)


    find_carrier    = py_trees.composites.Selector(name='Find carrier', memory=False)


    escape_carrier    = py_trees.composites.Selector(name='Escape carrier', memory=False)
    finish_centering2 = py_trees.composites.Sequence('Finish centering2', memory=True)
    check_if_centered2   = py_trees.behaviours.CheckBlackboardVariableValue(
        name    = 'Centered under carrier2?',
        check   = py_trees.common.ComparisonExpression(
            variable    = 'centered_carrier', 
            value       = True, 
            operator    = operator.eq)
    )
    zero_cmd_vel2  = py_trees_ros.publishers.FromBlackboard(
        name                = 'zero cmd_vel2',
        topic_name          = 'cmd_vel',
        topic_type          = geometry_msgs.msg.Twist,
        qos_profile         = qos_profile_system_default,
        blackboard_variable = 'zero_cmd_vel'
    )

    centre_if_under2 = py_trees.composites.Sequence('Centre if under2', memory=True)
    centre_if_under_sir2 = py_trees.decorators.SuccessIsRunning(centre_if_under2)

    check_if_under2  = py_trees.behaviours.CheckBlackboardVariableValue(
        name    = 'Under carrier2?',
        check   = py_trees.common.ComparisonExpression(
            variable    = 'under_carrier', 
            value       = True, 
            operator    = operator.eq)
    )
    centre_cmd_vel2  = py_trees_ros.publishers.FromBlackboard(
        name                = 'centre cmd_vel2',
        topic_name          = 'cmd_vel',
        topic_type          = geometry_msgs.msg.Twist,
        qos_profile         = qos_profile_system_default,
        blackboard_variable = 'centre_cmd_vel'
    )



    idle = py_trees.behaviours.Success(name='Idle')



    root.add_child(topics2bb)
    topics2bb.add_child(irtof2bb)
    topics2bb.add_child(odom2bb)
    root.add_child(priorities)
    priorities.add_child(proc_irtof)
    priorities.add_child(proc_vision)
    priorities.add_child(fetch)
    priorities.add_child(idle)

    fetch.add_child(find_carrier)
    fetch.add_child(lift)
    fetch.add_child(home_cmd_vel)
    fetch.add_child(home_delay)
    fetch.add_child(lower)
    fetch.add_child(escape_carrier)
    fetch.add_child(away_cmd_vel)
    fetch.add_child(away_delay)

    find_carrier.add_child(finish_centering)
    finish_centering.add_child(check_if_centered)
    finish_centering.add_child(zero_cmd_vel)

    find_carrier.add_child(centre_if_under_sir)
    centre_if_under.add_child(check_if_under)
    centre_if_under.add_child(centre_cmd_vel)

    find_carrier.add_child(under_if_aligned_sir)
    under_if_aligned.add_child(check_if_aligned)
    under_if_aligned.add_child(under_cmd_vel)

    find_carrier.add_child(avoid_collision_sir)
    avoid_collision.add_child(coll_check)
    avoid_collision.add_child(avoid_cmd_vel)

    find_carrier.add_child(align_if_found_sir)
    align_if_found.add_child(check_if_found)
    align_if_found.add_child(align_cmd_vel)

    find_carrier.add_child(random_wander_sir)
    random_wander.add_child(pick_direction)
    random_wander.add_child(random_cmd_vel)
    random_wander.add_child(wander_delay)

    escape_carrier.add_child(finish_centering2)
    finish_centering2.add_child(check_if_centered2)
    finish_centering2.add_child(zero_cmd_vel2)

    escape_carrier.add_child(centre_if_under_sir2)
    centre_if_under2.add_child(check_if_under2)
    centre_if_under2.add_child(centre_cmd_vel2)




    return root




        

def main():
    rclpy.init()

    #py_trees.logging.level = py_trees.logging.level.DEBUG
    root = create_root()
    #tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=True)
    tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=False)
    tree.setup()
    tree.tick_tock(period_ms=100.0)
    rclpy.spin(tree.node)



if __name__ == '__main__':
    main()
    
    



