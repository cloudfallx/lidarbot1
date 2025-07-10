from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():  # Parameter with default True for Gazebo
    
    use_sim_time = LaunchConfiguration('use_sim_time')
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

        # # Joint State Publisher
        # Node(
        #     package='joint_state_publisher',
        #     executable='joint_state_publisher',
        #     parameters=[{'use_sim_time': use_sim_time}] 
        # )
    ])
