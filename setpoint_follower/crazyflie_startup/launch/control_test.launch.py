import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('crazyflie_ros2_setpoint_follower'),
        'config',
        'config.yaml'
        )
    
    control = Node(
        package='solve_setpoint',
        executable='control',
        name='control',
        output='screen',
        parameters=[
            {'robot_prefix': '/barrier_service'},
            {'use_sim_time': False}
        ]
    )

    barrier_service = Node(
        package='barrier_builder',
        executable='barrier_server',
        name='barrier_service',
        output='screen',
        parameters=[
            {'robot_prefix': '/barrier_service'},
            {'use_sim_time': False},
            config
        ]
    )

    return LaunchDescription([
        control,
        barrier_service
        ])