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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

SDF_PATH = '/home/bmoro/Documents/crazyflies/ros2_ws/src/ros_gz_crazyflie/ros_gz_crazyflie_gazebo/models/crazyflie/model.sdf'
 

def create_new_sdf(name):
    with open(SDF_PATH) as f:
        sdf = f.read()

    sdf = sdf.replace("<robotNamespace>crazyflie</robotNamespace>", "<robotNamespace>{}</robotNamespace>".format(name))

    sdf = sdf.replace("<robotNamespace>crazyflie</robotNamespace>", "<robotNamespace>{}</robotNamespace>".format(name))

    file = open("/home/bmoro/Documents/crazyflies/ros2_ws/src/ros_gz_crazyflie/ros_gz_crazyflie_gazebo/models/crazyflie/{}.sdf".format(name), "w")
    file.write(sdf)

    # temp_sdf = tempfile.NamedTemporaryFile(mode='w+', dir='/home/bmoro/Documents/crazyflies/ros2_ws/src/ros_gz_crazyflie/ros_gz_crazyflie_gazebo/models/crazyflie'
    #     , delete=False, suffix='.sdf')
    # temp_sdf.write(sdf)
    
    return file

def generate_bridge_nodes(drone_names):
    bridge_nodes = []

    bridges_per_drone = [
        {
            "ros_suffix": "/cmd_vel",
            "gz_suffix": "/gazebo/command/twist",
            "ros_type": "geometry_msgs/msg/Twist",
            "gz_type": "gz.msgs.Twist",
            "dir": "ROS_TO_GZ"
        },
        {
            "ros_suffix": "/odom",
            "gz_suffix": "/odometry",
            "ros_type": "nav_msgs/msg/Odometry",
            "gz_type": "gz.msgs.Odometry",
            "dir": "GZ_TO_ROS"
        },
        {
            "ros_suffix": "/scan",
            "gz_suffix": "/lidar",
            "ros_type": "sensor_msgs/msg/LaserScan",
            "gz_type": "gz.msgs.LaserScan",  # <- USE gz.msgs.LaserScan
            "dir": "GZ_TO_ROS"
        },
        # This bridge is not supported; remove it if using Pose_V/TFMessage
        # {
        #     "ros_suffix": "/tf",
        #     "gz_suffix": "/pose",
        #     "ros_type": "tf2_msgs/msg/TFMessage",
        #     "gz_type": "gz.msgs.Pose_V",
        #     "dir": "GZ_TO_ROS"
        # },
    ]

    for drone in drone_names:
        for bridge in bridges_per_drone:
            ros_topic = f"/{drone}{bridge['ros_suffix']}"
            if "model" in bridge["gz_suffix"]:
                gz_topic = bridge["gz_suffix"].replace("crazyflie", drone)
            else:
                gz_topic = f"/model/{drone}{bridge['gz_suffix']}"

            if bridge["dir"] == "GZ_TO_ROS":
                arg = f"{gz_topic}@{bridge['gz_type']}[{bridge['ros_type']}]"
            else:
                arg = f"{ros_topic}@{bridge['ros_type']}[{bridge['gz_type']}]"

            bridge_nodes.append(
                Node(
                    package="ros_gz_bridge",
                    executable="parameter_bridge",
                    name=f"{drone}_bridge_{bridge['ros_suffix'].strip('/')}",
                    arguments=[arg],
                    remappings=[(gz_topic, ros_topic)],
                    output="screen"
                )
            )

    return bridge_nodes


def generate_bridge(drone_names):
    bridge_nodes = []
    for drone in drone_names:
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

def generate_launch_description():
    # Configure ROS nodes for launch

    gz_ln_arg = DeclareLaunchArgument(
        'gazebo_launch',
        default_value='True'
    )
    # Setup project paths
    pkg_project_bringup = get_package_share_directory('ros_gz_crazyflie_bringup')
    pkg_project_gazebo = get_package_share_directory('ros_gz_crazyflie_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Load the SDF file from "description" package
    if LaunchConfiguration('gazebo_launch') == 'True':
        gz_model_path = os.getenv('GZ_SIM_RESOURCE_PATH')
        sdf_file  =  os.path.join(gz_model_path, 'crazyflie', 'model.sdf')
        with open(sdf_file, 'r') as infp:
            robot_desc = infp.read()

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            condition=IfCondition(LaunchConfiguration('gazebo_launch')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_project_gazebo,
            'worlds',
            'crazyflie_world_10.sdf -r'
        ])}.items(),
    )

    #bridge_nodes = generate_bridge_nodes(drones)
    drones = ["crazyflie1", "crazyflie2", "crazyflie3", 
    "crazyflie4", "crazyflie5", "crazyflie6",
    "crazyflie7", "crazyflie8", "crazyflie9", "crazyflie10"]
    pos2 = [
        ('1.0', '0.5', '0'),
        ('1.0', '-0.5', '0'),
        ('0.33', '1.0', '0'),
        ('0.33', '0.0', '0'),
        ('0.33', '-1.0', '0'),
        ('-0.33', '1.0', '0'),
        ('-0.33', '0.0', '0'),
        ('-0.33', '-1.0', '0'),
        ('-1.0', '0.5', '0'),
        ('-1.0', '-0.5', '0')]
    drone_spawns = []
    for drone, pos in zip(drones, pos2):
        temp_file = create_new_sdf(drone)

        start_gazebo_ros_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', drone,
            '-file', temp_file.name,
            '-x', pos[0],
            '-y', pos[1],
            '-z', pos[2]
        ],
        output='screen')
        drone_spawns.append(start_gazebo_ros_spawner_cmd)

        #temp_file.close()

    gen_nodes = generate_bridge(drones)

    arg = "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
    clock_bridge = Node(
        package = 'ros_gz_bridge',
        executable = 'parameter_bridge',
        name = 'clock_bridge',
        arguments=[arg],
        output="screen"
    )

    cmd_bridges = []
    for drone in drones:
        cmd_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name=f"{drone}_cmd_bridge",
            arguments=["/{}/gazebo/command/twist@geometry_msgs/msg/Twist]gz.msgs.Twist".format(drone)],
            remappings=[("/{}/gazebo/command/twist".format(drone), "/{}/cmd_vel".format(drone))],
            output="screen"
        )
        cmd_bridges.append(cmd_bridge)

    control_nodes = []
    for drone in drones:
        control = Node(
            package='ros_gz_crazyflie_control',
            executable='control_services',
            output='screen',
            parameters=[
                {'hover_height': 0.5},
                {'robot_prefix': '/{}'.format(drone)},
                {'incoming_twist_topic': '/cmd_vel'},
                {'max_ang_z_rate': 0.4},
            ]
        )
        control_nodes.append(control)

    return LaunchDescription([
        gz_ln_arg,
        gz_sim,
        clock_bridge,
        *gen_nodes,
        *drone_spawns,
        #*cmd_bridges,
        #*bridge_nodes,
        *control_nodes        
        ])