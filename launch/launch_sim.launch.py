import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'lidarbot1'
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world')

    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value='/home/cloudfall/ros2_ws/src/lidarbot1/urdf/empty.world',
        description='Full path to the world SDF file'
    )

    # Robot State Publisher Launch File
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
            )
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # New Gazebo Sim Launch File (ros_gz_sim)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            )
        ]),
        launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()

    )

    # Spawn Entity Node (use 'create' for new Gazebo)
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',  # 'create' is the new executable for spawning entities
        arguments=[
            '-topic', 'robot_description',
            '-name', 'lidarbot',
            '-z', '1.0',
        ],
        output='screen'
    )

    return LaunchDescription([
        rsp,
        declare_world_arg,
        gazebo,
        spawn_entity,
    ])
