import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_udh1_description = get_package_share_directory('udh1_description')
    udh1_mapping_dir = get_package_share_directory('udh1_mapping')

    xacro_file = os.path.join(pkg_udh1_description, 'urdf', 'udh1.urdf')
    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = {'robot_description': robot_description_config.toxml()}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_desc]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': '/home/joao/Navigation/mapas/udh1_mapa.world'
        }.items()
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'udh1', '-z', '0.1'],
        output='screen'
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': '/home/joao/Navigation/mapas/udh1_mapa.yaml',
            'use_sim_time': True
        }]
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            os.path.join(udh1_mapping_dir, 'config', 'nav2_params.yaml'),
            {
                'use_sim_time': True,
                'scan_topic': '/scan'
            }
        ]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
            'use_sim_time': True
        }]
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch', 'navigation_launch.py'
            )
        ),
        launch_arguments={
            'params_file': os.path.join(udh1_mapping_dir, 'config', 'nav2_params.yaml'),
            'use_sim_time': 'true',
        }.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', '/home/joao/Navigation/src/my_robot_bringup/rviz/nav2_udh1.rviz']
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn_robot,
        map_server_node,
        amcl_node,
        lifecycle_manager,
        nav2_bringup,
        rviz_node
    ])
