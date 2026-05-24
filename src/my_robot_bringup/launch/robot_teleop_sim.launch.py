import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    udh1_description_dir = get_package_share_directory(
        'udh1_description'
    )

    my_bringup_dir = get_package_share_directory(
        'my_robot_bringup'
    )

    urdf_file = os.path.join(
        udh1_description_dir,
        'urdf',
        'udh1.urdf'
    )

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': False
            }
        ]
    )

    base_driver = Node(
        package='serial_com_py',
        executable='base_driver_sim',
        name='base_driver',
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d',
            os.path.join(
                my_bringup_dir,
                'rviz',
                'nav2_udh1.rviz'
            )
        ]
    )

    safe_stop = Node(
        package='serial_com_py',
        executable='safe_stop',
        name='safe_stop',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        safe_stop,
        base_driver,
        rviz
    ])