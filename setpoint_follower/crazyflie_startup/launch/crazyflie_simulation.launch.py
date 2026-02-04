import os
import yaml
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.utilities import perform_substitutions

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


###############################################################################################################
# Helper functions
###############################################################################################################


def create_new_sdf(name):
    pkg_world = get_package_share_directory('crazyflie_ros2_setpoint_follower')
    sdf_template = os.path.join(pkg_world, 'models', 'crazyflie', 'model.sdf')

    with open(sdf_template, 'r') as f:
        sdf = f.read()

    sdf = sdf.replace(
        "<robotNamespace>crazyflie</robotNamespace>",
        f"<robotNamespace>{name}</robotNamespace>"
    )
    
    sdf = sdf.replace(
        "<robotNamespace>crazyflie</robotNamespace>",
        f"<robotNamespace>{name}</robotNamespace>"
    )

    filepath = os.path.join(
        pkg_world,
        'models',
        'crazyflie',
        f'{name}.sdf'
    )

    with open(filepath, 'w') as f:
        f.write(sdf)

    return filepath   


def get_drones_from_config(config_path):
    with open(config_path, "r") as f:
        cfg = yaml.safe_load(f)

    params = cfg.get("/**", {}).get("ros__parameters", {})
    if not params:
        raise RuntimeError("Missing ros__parameters in mission file")

    agents = {}
    for key, val in params.items():
        if key.startswith("agent_"):
            agent_id = int(key.split("_")[1])
            agents[agent_id] = val

    if not agents:
        raise RuntimeError("No agents found in mission file")

    return agents


def generate_bridge(drone_ids):
    nodes = []

    # Clock bridge: only one instance needed
    nodes.append(
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="clock_bridge",
            arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
            output="screen",
        )
    )

    for drone_id in drone_ids:
        drone = f"crazyflie{drone_id}"

        # Cmd_vel bridge (ROS -> GZ)
        nodes.append(
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name= "{}_cmd_bridge".format(drone),
                arguments=[
                    "/{}/gazebo/command/twist@geometry_msgs/msg/Twist]gz.msgs.Twist".format(drone)
                ],
                remappings=[
                    ( "/{}/gazebo/command/twist".format(drone), "/{}/cmd_vel".format(drone))
                ],
                output="screen",
            )
        )

        # Odometry bridge (GZ -> ROS)
        nodes.append(
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="{}_odom_bridge".format(drone),
                arguments=[
                    "/model/{}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry".format(drone)
                ],
                remappings=[
                    ("/model/{}/odometry".format(drone), "/{}/odom".format(drone))
                ],
                output="screen",
            )
        )

        # Static TF for each drone
        nodes.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    '0', '0', '0', '0', '0', '0',
                    'world', f'{drone}/odom'
                ],
                output='screen'
            )
        )

    return nodes


def spawn_drones(drones):
    nodes = []

    for drone_id, spec in drones.items():
        drone = f"crazyflie{drone_id}"
        pos = spec.get("pos")

        if not pos or len(pos) != 3:
            raise RuntimeError(f"{drone} must have pos: [x,y,z]")

        sdf_path = create_new_sdf(drone)

        print("Spawning:", drone_id)   # <-- DEBUG

        nodes.append(
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', drone,
                    '-file', sdf_path,
                    '-x', str(pos[0]),
                    '-y', str(pos[1]),
                    '-z', str(pos[2]),
                ],
                output='screen',
            )
        )

    return nodes


###############################################################################################################
# Runtime setup
###############################################################################################################

def launch_setup(context, *args, **kwargs):

    config_path = perform_substitutions(context, [MISSION_CONFIG])
    drones = get_drones_from_config(config_path)

    drone_ids = sorted(drones.keys())

    nodes = []
    nodes += spawn_drones(drones)
    nodes += generate_bridge(drone_ids)

    return nodes


###############################################################################################################
# Main launch description
###############################################################################################################

def generate_launch_description():

    gazebo_arg = DeclareLaunchArgument(
        'gazebo_launch',
        default_value='true',
        choices=['true', 'false'],
    )

    mission_yaml = DeclareLaunchArgument(
        'mission_yaml',
        default_value='config.yaml',
    )

    global MISSION_CONFIG
    MISSION_CONFIG = PathJoinSubstitution([
        FindPackageShare('crazyflie_ros2_setpoint_follower'),
        'config',
        'missions',
        LaunchConfiguration('mission_yaml'),
    ])

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_world = get_package_share_directory('crazyflie_ros2_setpoint_follower')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('gazebo_launch')),
        launch_arguments={
            'gz_args': os.path.join(pkg_world, 'config', 'crazyflie_world.sdf') + ' -r'
        }.items(),
    )

    return LaunchDescription([
        gazebo_arg,
        mission_yaml,
        gz_sim,
        OpaqueFunction(function=launch_setup),
    ])
