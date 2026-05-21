import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    udh1_description_dir = get_package_share_directory('udh1_description')
    udh1_gazebo_dir      = get_package_share_directory('udh1_gazebo')
    gazebo_ros_dir       = get_package_share_directory('gazebo_ros')

    # ── Argumentos ──────────────────────────────────────────────────
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(udh1_gazebo_dir, 'worlds', 'udh1_world.world'),
        description='Mundo Gazebo a carregar'
    )
    x_arg = DeclareLaunchArgument('x', default_value='0.0',  description='Posição X inicial')
    y_arg = DeclareLaunchArgument('y', default_value='0.0',  description='Posição Y inicial')
    z_arg = DeclareLaunchArgument('z', default_value='0.15', description='Posição Z inicial')
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Abrir RViz automaticamente (true/false)'
    )

    world  = LaunchConfiguration('world')
    x_pos  = LaunchConfiguration('x')
    y_pos  = LaunchConfiguration('y')
    z_pos  = LaunchConfiguration('z')

    # ── Robot Description ────────────────────────────────────────────
    xacro_file = os.path.join(udh1_description_dir, 'urdf', 'udh1.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # ── Config do RViz ───────────────────────────────────────────────
    rviz_config = os.path.join(udh1_gazebo_dir, 'rviz', 'udh1_gazebo.rviz')

    # ── Nós ─────────────────────────────────────────────────────────

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time': True
        }]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_udh1',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'udh1',
            '-x', x_pos,
            '-y', y_pos,
            '-z', z_pos,
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        world_arg,
        x_arg,
        y_arg,
        z_arg,
        rviz_arg,
        robot_state_publisher,
        joint_state_publisher,
        gazebo,
        spawn_robot,
        rviz_node,
    ])
