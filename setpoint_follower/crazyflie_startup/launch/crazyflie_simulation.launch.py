# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import tempfile
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
 

def create_new_sdf(name):
    pkg_world = get_package_share_directory('crazyflie_ros2_setpoint_follower')
    SDF_PATH = os.path.join(pkg_world, 'models/crazyflie/model.sdf')

    with open(SDF_PATH) as f:
        sdf = f.read()

    sdf = sdf.replace("<robotNamespace>crazyflie</robotNamespace>", "<robotNamespace>{}</robotNamespace>".format(name))

    sdf = sdf.replace("<robotNamespace>crazyflie</robotNamespace>", "<robotNamespace>{}</robotNamespace>".format(name))

    filepath = os.path.join(pkg_world, "models/crazyflie/{}.sdf".format(name))
    file = open(filepath, "w")
    file.write(sdf)
    
    return file

def generate_bridge(drone_indices):
    bridge_nodes = []
    for drone_id in drone_indices:
        drone   = 'crazyflie{}'.format(drone_id)
        cmd_arg = ("/{}/gazebo/command/twist@geometry_msgs/msg/Twist]gz.msgs.Twist".format(drone), 
        "/{}/cmd_vel".format(drone), 
        "/{}/gazebo/command/twist".format(drone), 
        "{}_cmd_bridge".format(drone))

        odom_arg = ("/model/{}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry".format(drone), 
        "/{}/odom".format(drone), 
        "/model/{}/odometry".format(drone),
        "{}_odom_bridge".format(drone))

        for args in (cmd_arg, odom_arg):
            bridge_nodes.append(
                Node(
                    package="ros_gz_bridge",
                    executable="parameter_bridge",
                    name=args[3],
                    arguments=[args[0]],
                    remappings=[(args[2], args[1])],
                    output="screen"
                )
            )

    return bridge_nodes

def drone_positions_from_config(config_path):
    """
    Load drone positions from a ROS 2 parameter YAML file.

    Expected structure:
      /**:
        ros__parameters:
          NUM_AGENTS: N
          POSITIONS:
            POS_1: [x, y, z]
            ...

    Args:
      config_path (str): Path to YAML config file

    Returns:
      dict[int, list[float]]: { drone_index: [x, y, z] }
    """
    with open(config_path, "r") as f:
        cfg = yaml.safe_load(f)

    

    if "/**" not in cfg:
        raise RuntimeError("Missing '/**' root key in config")

    params = cfg["/**"].get("ros__parameters", {})
    
    if params is None :
        raise  RuntimeError("Current parameter file seems to be broken. Make sure that the keys are nested under ros__parameters")

    if "NUM_AGENTS" not in params:
        raise RuntimeError("NUM_AGENTS not defined in config")

    if "POSITIONS" not in params:
        raise RuntimeError("POSITIONS not defined in config")

    num_agents = params["NUM_AGENTS"]
    positions_cfg = params["POSITIONS"]

    positions = {}

    for i in range(1, num_agents + 1):
        key = f"POS_{i}"

        if key not in positions_cfg:
            raise RuntimeError(f"Given number of agents is {num_agents}. Missing position for drone {i} ({key}). Count starts from 1")

        value = positions_cfg[key]

        if not isinstance(value, list) or len(value) != 3:
            raise ValueError(
                f"{key} must be a list of 3 numbers [x, y, z]"
            )

        positions[i] = [float(v) for v in value]

    return positions


def generate_launch_description():
    # Configure ROS nodes for launch

    gz_ln_arg = DeclareLaunchArgument(
        'gazebo_launch',
        default_value='True'
    )
    # Setup project paths
    pkg_project_bringup = get_package_share_directory('ros_gz_crazyflie_bringup')
    pkg_project_gazebo = get_package_share_directory('ros_gz_crazyflie_gazebo')
    pkg_world = get_package_share_directory('crazyflie_ros2_setpoint_follower')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            condition=IfCondition(LaunchConfiguration('gazebo_launch')),
        launch_arguments={'gz_args': os.path.join(pkg_world, 'config', 
        'crazyflie_world.sdf') + ' -r'}.items(),
    )

    #bridge_nodes = generate_bridge_nodes(drones)
    config_path = os.path.join(
        pkg_world,
        'config',
        'config.yaml'
    )
    
    drones_position = drone_positions_from_config(config_path)
    
    drone_spawns    = []
    for drone_id, pos in drones_position.items():
        
        drone     = 'crazyflie{}'.format(drone_id)
        temp_file = create_new_sdf(drone)

        start_gazebo_ros_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', drone,
            '-file', temp_file.name,
            '-x', str(pos[0]),
            '-y', str(pos[1]),
            '-z', str(pos[2])
        ],
        output='screen')
        drone_spawns.append(start_gazebo_ros_spawner_cmd)

        #temp_file.close()

    gen_nodes = generate_bridge(drones_position.keys())

    arg = "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
    clock_bridge = Node(
        package = 'ros_gz_bridge',
        executable = 'parameter_bridge',
        name = 'clock_bridge',
        arguments=[arg],
        output="screen"
    )

    cmd_bridges = []
    for drone_id in drones_position.keys():
        cmd_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name=f"{drone}_cmd_bridge",
            arguments=["/crazyflie{}/gazebo/command/twist@geometry_msgs/msg/Twist]gz.msgs.Twist".format(drone_id)],
            remappings=[("/{}/gazebo/command/twist".format(drone), "/{}/cmd_vel".format(drone))],
            output="screen"
        )
        cmd_bridges.append(cmd_bridge)

    tf_bridges = []
    for drone_id in drones_position.keys():
        tf_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'crazyflie{}/odom'.format(drone_id)],
            output='screen'
        )
        tf_bridges.append(tf_node)

    return LaunchDescription([
        gz_ln_arg,
        gz_sim,
        clock_bridge,
        *gen_nodes,
        *drone_spawns,
        *tf_bridges      
        ])