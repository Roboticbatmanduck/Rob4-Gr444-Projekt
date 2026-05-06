from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


def generate_launch_description():

    simulation_dir = get_package_share_directory('simulation')
    follow_me_dir = get_package_share_directory('follow_me')

    config_file = os.path.join(
        simulation_dir,
        'config',
        'follow_me_sim.yaml'
    )

    start_followme_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'world': os.path.join(
                simulation_dir,
                'worlds',
                'followme.world'
            ),
            'gui': 'false',
            'extra_gazebo_args': '-s libgazebo_ros_factory.so',
        }.items()
    )

    spawn_robot_delayed = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'turtlebot3_burger_depth',
                    '-file', os.path.join(
                        simulation_dir,
                        'models',
                        'model.sdf'
                    )
                ],
                output='screen'
            )
        ]
    )

    start_rqt_image_view = TimerAction(
        period=20.0,
        actions=[
            Node(
                package='rqt_image_view',
                executable='rqt_image_view',
                output='screen'
            )
        ]
    )

    yolo_node = TimerAction(
        period=22.0,
        actions=[
            Node(
                package='follow_me',
                executable='person_center_pc',
                name='person_center_pc',
                parameters=[config_file],
                output='screen',
            )
        ]
    )

    person_distance_node = TimerAction(
        period=23.0,
        actions=[
            Node(
                package='follow_me',
                executable='person_distance',
                name='person_distance',
                parameters=[config_file],
                output='screen',
            )
        ]
    )

    person_angle_node = TimerAction(
        period=23.0,
        actions=[
            Node(
                package='follow_me',
                executable='person_angle',
                name='person_angle',
                parameters=[config_file],
                output='screen',
            )
        ]
    )

    debug_visualizer_node = TimerAction(
        period=24.0,
        actions=[
            Node(
                package='follow_me',
                executable='debug_visualizer',
                name='debug_visualizer',
                parameters=[config_file],
                output='screen',
            )
        ]
    )

    follower_node = TimerAction(
        period=25.0,
        actions=[
            Node(
                package='follow_me',
                executable='simple_follower',
                name='simple_follower',
                parameters=[config_file],
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=os.path.join(simulation_dir, 'models')
        ),
        start_followme_world,
        spawn_robot_delayed,
        start_rqt_image_view,
        yolo_node,
        person_distance_node,
        person_angle_node,
        debug_visualizer_node,
        follower_node,
    ])
