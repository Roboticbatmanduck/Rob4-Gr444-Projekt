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
            'gui': 'true',
            'extra_gazebo_args': '--verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so',
        }.items()
    )

    spawn_robot = Node(
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
    output='screen',
    prefix="bash -c 'until ros2 service list | grep /spawn_entity; do echo waiting for spawn service...; sleep 1; done; exec'"
)

    yolo_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='follow_me',
                executable='yolo_person_center',
                name='person_center',
                parameters=[config_file],
                output='screen',
            )
        ]
    )

    person_distance_node = TimerAction(
        period=7.0,
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
        period=7.0,
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
        period=8.0,
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

    angle_regulator = TimerAction(
        period=9.0,
        actions=[
            Node(
                package="follow_me",
                executable="angle_regulator",
                name="angle_regulator",
                output="screen",
                parameters=[config_file],
            )
        ],
    )

    distance_regulator = TimerAction(
        period=9.0,
        actions=[
            Node(
                package="follow_me",
                executable="distance_regulator",
                name="distance_regulator",
                output="screen",
                parameters=[config_file],
            )
        ],
    )

    command_sender = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="follow_me",
                executable="command_sender",
                name="command_sender",
                output="screen",
                parameters=[config_file],
            )
        ],
    )

    start_rqt_image_view = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='rqt_image_view',
                executable='rqt_image_view',
                output='screen'
            )
        ]
    )



    return LaunchDescription([
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=os.path.join(simulation_dir, 'models')
        ),
        SetEnvironmentVariable(
        name='FASTRTPS_DEFAULT_PROFILES_FILE',
        value=''
        ),
        SetEnvironmentVariable(
            name='RMW_FASTRTPS_USE_QOS_FROM_XML',
            value='0'
        ),
        SetEnvironmentVariable(
            name='ROS_DISABLE_SHM',
            value='1'
        ),

        start_followme_world,
        spawn_robot,
        start_rqt_image_view,
        yolo_node,
        person_distance_node,
        person_angle_node,
        debug_visualizer_node,
        angle_regulator,
        distance_regulator,
        command_sender
        ,
    ]
    )
