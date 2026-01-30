import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments for dynamic configuration
    with_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        choices=['true', 'false'],
        description='Launch RViz (default: true)'
    )

    mission_yaml = DeclareLaunchArgument(
        'mission_yaml',
        default_value='config.yaml',
        description='Full path to the mission yaml file to load'
    )

    # LaunchConfiguration for dynamic values
    rviz = LaunchConfiguration('rviz')
    m_yaml = LaunchConfiguration('mission_yaml')

    # Setup project paths
    pkg_project_crazyflie_setpoint = get_package_share_directory('crazyflie_ros2_setpoint_follower')

    # Resolve path to the mission YAML configuration dynamically
    config = PathJoinSubstitution([
        FindPackageShare('crazyflie_ros2_setpoint_follower'),
        'config',
        m_yaml
    ])

    # Setup to launch a crazyflie gazebo simulation from the ros_gz_crazyflie project
    crazyflie_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_crazyflie_setpoint, 'launch', 'crazyflie_simulation_10.launch.py'))
    )

    # Define the manager node
    manager_node = Node(
        package='solve_setpoint',
        executable='manager',
        name='manager_node',
        output='screen',
        parameters=[config]  # Dynamic parameter loading from YAML
    )

    # Define the agent node
    agent_node = Node(
        package='solve_setpoint',
        executable='agentMPC',
        output='screen',
        parameters=[config]  # Dynamic parameter loading from YAML
    )

    # Define the barrier service node
    barrier_service = Node(
        package='barrier_builder',
        executable='barrier_server',
        name='barrier_service',
        output='screen',
        parameters=[config]  # Dynamic parameter loading from YAML
    )

    # Include RViz launch file conditionally
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('crazyflie_ros2_setpoint_follower'),
                'launch',
                'crazyflie_rviz.launch.py'
            )
        ),
        condition=IfCondition(rviz)
    )

    # Return the full launch description
    return LaunchDescription([
        with_rviz,
        mission_yaml,
        crazyflie_simulation,
        agent_node,
        barrier_service,
        manager_node,
        rviz_launch,
    ])
