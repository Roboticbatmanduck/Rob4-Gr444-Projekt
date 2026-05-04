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

    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    simulation_dir = get_package_share_directory('simulation')
    # Start followme-world UDEN robot
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
    }.items()
    )

    # Spawn robot EFTER delay
    spawn_robot_delayed = TimerAction(
        period=5.0,  # ⏱️ Justér evt. til 6–8 sek
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'turtlebot3_burger_depth',
                    '-file',
                    os.path.join(
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
        period=7.0,  # lidt efter robot-spawn
        actions=[
            Node(
                package='rqt_image_view',
                executable='rqt_image_view',
                output='screen'
            )
        ]
    )

# --- YOLO NODE (delayed) ---
    yolo_node = TimerAction(
    	period=8.0,
    	actions=[
            Node(
        	    package='follow_me',
        	    executable='person_center_pc',
        	    name='person_center_pc',
        	    output='screen',
            )
        ]
    )

# --- SIMPLE FOLLOWER NODE (delayed) ---
    follower_node = TimerAction(
    	period=9.0,
    	actions=[
        	Node(
            	package='follow_me',
            	executable='simple_follower',
        		name='simple_follower',
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
        follower_node
    ])
