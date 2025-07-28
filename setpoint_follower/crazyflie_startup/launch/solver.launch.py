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
    pkg_project_crazyflie_gazebo = get_package_share_directory('ros_gz_crazyflie_bringup')

    # Setup to launch a crazyflie gazebo simulation from the ros_gz_crazyflie project
    crazyflie_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_crazyflie_gazebo, 'launch', 'crazyflie_simulation_10.launch.py'))
    )

    manager_node = Node(
        package='solve_setpoint',
        executable='manager',
        name='manager_node',
        output='screen',
        parameters=[
            {'robot_prefix': '/manager_node'},
            {'use_sim_time': True}
        ]
    )

    drones = ['/crazyflie{}'.format(i) for i in range(1,11)]

    # agent_node_list = []
    # for drone in drones:

    #     agent_node = Node(
    #         package='solve_setpoint',
    #         executable='agent',
    #         name='agent_{}'.format(drone),
    #         output='screen',
    #         parameters=[
    #             {'robot_prefix': '/{}'.format(drone)},
    #             {'use_sim_time': True}
    #         ]
    #     )

    #     agent_node_list.append(agent_node)
    agent_node = Node(
            package='solve_setpoint',
            executable='agent',
            output='screen',
            parameters=[
                {'robot_prefix': 'crazyflie'},
                {'use_sim_time': True}
            ]
        )

    return LaunchDescription([
        crazyflie_simulation,
        agent_node,
        manager_node
        ])