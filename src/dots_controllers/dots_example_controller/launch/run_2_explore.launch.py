
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from scripts import GazeboRosPaths
import sys
import xacro

def generate_launch_description():


    # Starting poses for robots
    robots     = [
                    ('robot_00000001',     (    0,      0,      0   )),
                    ('robot_00000002',     (    1,      -0.7,   0.5 )),
              #      ('robot_00000003',     (  0.5,      -0.7,   1.5 )),
               #     ('robot_00000004',     (    1,       0.7,   2.5 )),
                #    ('robot_00000005',     (  0.5,       0.7,   0.5 )),
                 #   ('robot_00000006',     (    0,      -0.7,   1.5 )),
    ]

    # Package directories
    pkg_share       = get_package_share_directory('dots_sim')
    pkg_controller  = get_package_share_directory('dots_example_controller')


    # Launch config variables
    use_sim_time    = LaunchConfiguration('use_sim_time', default='True')
    use_rviz        = LaunchConfiguration('use_rviz', default='False')

    declare_use_sim_time    = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_use_rviz        = DeclareLaunchArgument('use_rviz', default_value='false')

    # Build the launch description
    ld = LaunchDescription()
    for r in robots:
        print('robot:%s pose:%s' % (r[0],r[1]))
        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_controller, 'launch', 'explore.launch.py')),
            launch_arguments    = { 'robot_name'    : PythonExpression(['"', r[0], '"']),
                                    'robot_pose'    : '%f,%f,%f' % (r[1][0], r[1][1], r[1][2]),
                                    'use_sim_time'  : use_sim_time,
            }.items()
        ))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource( os.path.join(pkg_share, 'launch', 'gazebo_rviz.launch.py')),
        launch_arguments    = {'use_rviz'   : use_rviz}.items()
    ))

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_use_rviz)

    return ld
    

