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
    # Configure ROS nodes for launch
    config_file = os.path.join(
        get_package_share_directory('crazyflie_ros2_setpoint_follower'),
        'config',
        'crazyflie.rviz'
    )

    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', config_file]
    )

    marker_node = Node(
        package='graph_rviz',
        executable='edge_markers',
        name='edge_markers',
        output='screen',
        parameters=[
            {'robot_prefix': '/marker_node'},
            {'use_sim_time': True}
        ]
    )

    return LaunchDescription([
        rviz_node,
        marker_node
        ])