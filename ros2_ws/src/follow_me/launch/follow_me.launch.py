from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")

    config_arg = DeclareLaunchArgument(
        "config_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("follow_me"),
            "config",
            "follow_me.yaml",
        ]),
        description="Path to follow_me parameter YAML file",
    )

    turtlebot3_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("turtlebot3_bringup"),
                "launch",
                "robot.launch.py",
            ])
        ])
    )

    realsense_camera = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare("realsense2_camera"),
                        "launch",
                        "rs_launch.py",
                    ])
                ]),
                launch_arguments={
                    "enable_color": "true",
                    "enable_depth": "true",
                    "align_depth.enable": "true",
                    "enable_sync": "true",
                
                    "rgb_camera.profile": "640x480x30",
                    "depth_module.profile": "640x480x30",
                }.items()
            )
        ],
    )

    yolo_person_center = TimerAction(
        period=7.0,
        actions=[
            Node(
                package="follow_me",
                executable="yolo_person_center",
                name="yolo_person_center",
                output="screen",
                parameters=[config_file],
                arguments=["--ros-args", "--log-level", "info"],
            )
        ],
    )

    person_angle = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="follow_me",
                executable="person_angle",
                name="pixel_to_angle",
                output="screen",
                parameters=[config_file],
                arguments=["--ros-args", "--log-level", "info"],
            )
        ],
    )

    person_distance = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="follow_me",
                executable="person_distance",
                name="person_distance",
                output="screen",
                parameters=[config_file],
                arguments=["--ros-args", "--log-level", "info"],
            )
        ],
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

    return LaunchDescription([
        config_arg,
        turtlebot3_bringup,
        realsense_camera,
        yolo_person_center,
        person_angle,
        person_distance,
        angle_regulator,
        distance_regulator,
        command_sender,
    ])