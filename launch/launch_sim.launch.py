import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    package_name='lidarbot1'

    # Robot State Publisher Launch File
    rsp=IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Gazebo Launch File
    gazebo=IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
        )])
    )

    # Spawner Node
    spawn_entity=Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'lidarbot'
        ],
        output='screen'
    )

    diff_drive_spawner=Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"]
    )

    joint_broad_spawner=Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"]
    )

    # Twist Mux
    # twist_mux_params=os.path.join(get_package_share_directory(package_name), 'config', 'twist_mux.yaml')

    # twist_mux=Node(
    #     package="twist_mux",
    #     executable="twist_mux",
    #     parameters=[twist_mux_params, {'use_sim_time': True}],
    #     remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')]
    # )
    
    # Launch the files
    return LaunchDescription([
        rsp,
        gazebo,
        # twist_mux,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner
    ])

