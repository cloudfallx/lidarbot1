import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description(use_sim_time=True):
    pkg_share = get_package_share_directory('lidarbot1')
    
    # Process Xacro file to URDF
    xacro_file = os.path.join(pkg_share, 'urdf', 'lidarbot1.urdf.xacro')
    robot_desc = Command(['xacro ', xacro_file])

    # Path to Rviz2 config file
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'lidarbot1_rviz.rviz')

    return LaunchDescription([
        # # Robot State Publisher
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     parameters=[{
        #         'robot_description': robot_desc,
        #         'use_sim_time': use_sim_time
        #     }],
        #     output='screen',
        # ),

        # Joint State Publisher GUI
        # Node(
        #     package='joint_state_publisher_gui',
        #     executable='joint_state_publisher_gui',
        #     name='joint_state_publisher_gui',
        #     parameters=[{'use_sim_time': use_sim_time}] 
        # ),

        # Rviz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}] 
        )
    ])
