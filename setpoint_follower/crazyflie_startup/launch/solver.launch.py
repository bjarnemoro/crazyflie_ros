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

    # Setup project paths
    pkg_project_crazyflie_setpoint = get_package_share_directory('crazyflie_ros2_setpoint_follower')

    #Setup to launch a crazyflie gazebo simulation from the ros_gz_crazyflie project
    crazyflie_simulation = IncludeLaunchDescription(
       PythonLaunchDescriptionSource(
           os.path.join(pkg_project_crazyflie_setpoint, 'launch', 'crazyflie_simulation_10.launch.py'))
    )

    config = os.path.join(
       get_package_share_directory('crazyflie_ros2_setpoint_follower'),
       'config',
       'config.yaml'
       )

    manager_node = Node(
        package='solve_setpoint',
        executable='manager',
        name='manager_node',
        output='screen',
        parameters=[
            {'robot_prefix': '/crazyflie'},
            {'use_sim_time': True},
            config
        ]
    )

    agent_node = Node(
            package='solve_setpoint',
            executable='agentMPC',
            output='screen',
            parameters=[
                {'robot_prefix': '/crazyflie'},
                {'use_sim_time': True},
                config
            ]
        )
    
    barrier_service = Node(
        package='barrier_builder',
        executable='barrier_server',
        name='barrier_service',
        output='screen',
        parameters=[
            {'robot_prefix': '/barrier_service'},
            {'use_sim_time': True},
            config
        ]
    )

    return LaunchDescription([
        crazyflie_simulation,
        agent_node,
        barrier_service,
        manager_node
        ])