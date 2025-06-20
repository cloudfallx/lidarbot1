from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description(use_sim_time=True):  # Parameter with default True for Gazebo
    return LaunchDescription([
        
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': Command([
                    'xacro ',
                    PathJoinSubstitution([
                        FindPackageShare('lidarbot1'),
                        'urdf',
                        'lidarbot1.urdf.xacro'
                    ])
                ]),
                'use_sim_time': use_sim_time  # Gazebo time sync
            }],
            output='screen'
        ),

        # Joint State Publisher (critical fix added)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            parameters=[{'use_sim_time': use_sim_time}]  # Added parameter propagation
        )
    ])
