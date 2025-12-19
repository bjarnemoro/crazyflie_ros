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
 

def create_new_sdf(name):
    pkg_project_gazebo = get_package_share_directory('ros_gz_crazyflie_gazebo')
    SDF_PATH = os.path.join(pkg_project_gazebo, 'models/crazyflie/model.sdf')

    with open(SDF_PATH) as f:
        sdf = f.read()

    sdf = sdf.replace("<robotNamespace>crazyflie</robotNamespace>", "<robotNamespace>{}</robotNamespace>".format(name))

    sdf = sdf.replace("<robotNamespace>crazyflie</robotNamespace>", "<robotNamespace>{}</robotNamespace>".format(name))

    filepath = os.path.join(pkg_project_gazebo, "models/crazyflie/{}.sdf".format(name))
    file = open(filepath, "w")
    file.write(sdf)
    
    return file

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
    pkg_10_world = get_package_share_directory('crazyflie_ros2_setpoint_follower')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            condition=IfCondition(LaunchConfiguration('gazebo_launch')),
        launch_arguments={'gz_args': os.path.join(pkg_10_world, 'config', 
        'crazyflie_world_10.sdf') + ' -r'}.items(),
    )

    #bridge_nodes = generate_bridge_nodes(drones)
    drones = ['crazyflie{}'.format(i) for i in range(1,11)]
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

    tf_bridges = []
    for drone in drones:
        tf_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', '{}/odom'.format(drone)],
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