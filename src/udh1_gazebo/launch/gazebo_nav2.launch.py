import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    udh1_description_dir = get_package_share_directory('udh1_description')
    udh1_gazebo_dir      = get_package_share_directory('udh1_gazebo')
    udh1_mapping_dir     = get_package_share_directory('udh1_mapping')
    gazebo_ros_dir       = get_package_share_directory('gazebo_ros')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(udh1_gazebo_dir, 'worlds', 'udh1_world.world'),
    )
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            os.path.expanduser('~'), 'athome_ws', 'mapas', 'mapa_udh1.yaml'
        ),
    )
    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(udh1_mapping_dir, 'config', 'nav2_params_sim.yaml'),
    )
    x_arg = DeclareLaunchArgument('x', default_value='0.0')
    y_arg = DeclareLaunchArgument('y', default_value='0.0')
    z_arg = DeclareLaunchArgument('z', default_value='0.15')

    world    = LaunchConfiguration('world')
    map_file = LaunchConfiguration('map')
    params   = LaunchConfiguration('params_file')
    x_pos    = LaunchConfiguration('x')
    y_pos    = LaunchConfiguration('y')
    z_pos    = LaunchConfiguration('z')

    xacro_file = os.path.join(udh1_description_dir, 'urdf', 'udh1.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()
    rviz_config = os.path.join(udh1_gazebo_dir, 'rviz', 'udh1_gazebo.rviz')

    # ── Imediato ─────────────────────────────────────────────────────
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
            '-x', x_pos, '-y', y_pos, '-z', z_pos,
        ]
    )

    # ── Delay 5s — Localização ───────────────────────────────────────
    map_server = TimerAction(period=5.0, actions=[
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'yaml_filename': map_file
            }]
        )
    ])

    amcl = TimerAction(period=5.0, actions=[
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[params, {'use_sim_time': True}]
        )
    ])

    lifecycle_localization = TimerAction(period=8.0, actions=[
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart':    True,
                'node_names':   ['map_server', 'amcl']
            }]
        )
    ])

    # ── Delay 10s — Nav2 nós individuais (sem nav2_bringup) ──────────
    controller_server = TimerAction(period=10.0, actions=[
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params, {'use_sim_time': True}]
        )
    ])

    planner_server = TimerAction(period=10.0, actions=[
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params, {'use_sim_time': True}]
        )
    ])

    behavior_server = TimerAction(period=10.0, actions=[
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params, {'use_sim_time': True}]
        )
    ])

    bt_navigator = TimerAction(period=10.0, actions=[
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params, {'use_sim_time': True}]
        )
    ])

    # ── Delay 13s — Lifecycle do Nav2 ────────────────────────────────
    lifecycle_navigation = TimerAction(period=13.0, actions=[
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart':    True,
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                ]
            }]
        )
    ])

    # ── Delay 15s — RViz ─────────────────────────────────────────────
    rviz_node = TimerAction(period=15.0, actions=[
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}]
        )
    ])

    return LaunchDescription([
        world_arg, map_arg, params_arg,
        x_arg, y_arg, z_arg,
        robot_state_publisher,
        joint_state_publisher,
        gazebo,
        spawn_robot,
        map_server,
        amcl,
        lifecycle_localization,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        lifecycle_navigation,
        rviz_node,
    ])
