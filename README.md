# crazyflie_ros
### setup
ros dependencies
```
ros2: humble/jazzy
gazebo: Gazebo Sim, version 8.10.0
```

python environment
```
ament_copyright==0.17.3
ament_flake8==0.17.3
ament_index_python==1.8.1
ament_pep257==0.17.3
cdd==0.1.12
cvxpy==1.6.6
geometry_msgs==5.3.6
launch_ros==0.26.10
matplotlib==3.10.8
nav_msgs==5.3.6
numpy==2.3.5
pytest==8.4.1
pyx==0.17
rclpy==7.1.6
scipy==1.16.3
setuptools==70.0.0
std_msgs==5.3.6
tf_transformations==1.1.0
visualization_msgs==5.3.6
```

### installation

Create a new ROS 2 workspace:

``` bash
mkdir -p ros_ws/src
```

Install this repository and the crazyswarm2 library:

```bash
cd ros_ws/src
git clone --recursive https://github.com/bjarnemoro/crazyflie_ros.git
git clone --recursive https://github.com/IMRCLab/crazyswarm2.git
```

Install the motion capture tracking dependencies:

```bash
sudo apt-get install ros-${ROS_DISTRO}-motion-capture-tracking
sudo apt-get install ros-${ROS_DISTRO}-tf-transformations
```
Install the cflib Python dependencies:

```bash
pip3 install cflib transforms3d
```

If you encounter errors related to the crazyswarm2 library, please refer to the crazyswarm2 [installation guide](https://imrclab.github.io/crazyswarm2/installation.html). If everything looks good, build the workspace:

```bash
cd ..
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Running Simulation
you can run the main simulation in gazebo with rviz

1. Without Human in the loop (tasks are read from the configuration file inside `config/mission/simple.yaml`)
```
ros2 launch crazyflie_ros2_setpoint_follower solver.launch.py mission_yaml:=simple.yaml backend:=sim hil:=false

```

2. with human in the loop

```
ros2 launch crazyflie_ros2_setpoint_follower solver.launch.py mission_yaml:=simple.yaml backend:=sim hil:=true

```

you can store you mission in a config file like the ones stored in ´setpoint_follower/crazyflie_startup/config/missions´ folder. Then you can start your personalized mission as 


### Running Hardware Experiment

To run the real experiments you need to set up a few things 

1. Pull a number of n of crazyflies. These are numbered and their address is the hexadeciaml version of their number. The address of each drone will be defined by this funciton 

```
f"radio://{radio}/{channel}/2M/E7E7E7E7{agent_id:02X}"
```

**radio**: defines which radio to use if you use multiple. The number starts from 0 if you have one radio.


**channel**:defined which channel the crazyflie is currently on. This should be setup by the `cfclient` connecting the drone to your computer via usb cable

**address**. The address of the crazyflie should also be set form the `cfclient`. It is typical to use the convension of hexadeciaml number. So crazyflie 1 will have the address E7E7E7E701, while crazyflie 10 will have the address E7E7E7E70A and so on. If you are unsure you can use this [tool](https://www.binaryhexconverter.com/decimal-to-hex-converter) to help you, but you will not have to do this conversion yourself as it is done already inside the package.


2. Make sure that the mocap system is active and is delivering the point cloud of the markers.

3. Create a configuation file inside `config/missions/` in which you set up the required agents and you define the initial position of the agents as

```
/**: 
  ros__parameters:
    
    ###########################################
    # General node parameters 
    ###########################################
    DIM: 2               # Dimension of the space
    MANAGER_TIMER: 0.8   # Time at which the manager updates (seconds)
    AGENT_TIMER: 0.1     # frequency of the MPC (seconds)
    SPEED: 0.4           # Nominal speed of the agents
    BOX_WEIGHT: 10       # Weight of the boxes in the cost function
    COMM_DISTANCE: 2.    # Communication distance between agents
    HOOVERING_HEIGHT: 1. # Alitude at which the drones move nominally

    ###########################################################################################################
    # Specific of each agent. The agent number corresponds to the crazyflie number (check the tag on the drone).
    # N.B. Make sure that a drone with such number exists in the real fleet or in the simulation.
    ###########################################################################################################

    agent_1:
      pos:  [ -1.08,   -0.99,  0.3 ]
      radio: 0
      channel: 40

    agent_4:
      pos:  [ -1.1,  0.68,  0.3 ]
      radio: 0
      channel: 40

    agent_5:
      pos:  [ -0.52,  -0.18,  0.3 ]
      radio: 0
      channel: 40
```

make sure that the initial position is roughly accurate.

2. Once your crazyflies are ready you should **first** run the launch file for your mission

```
ros2 launch crazyflie_ros2_setpoint_follower solver.launch.py mission_yaml:=simple.yaml backend:=hardware hil:=true

```

it is important that launch this first as it will automatically generate a custom `crazyflie.yalm` which can be used by the crazyswamr library to connect to each crazyflie

3. After you have launch your mission you can launch your drones using crazyswamr via the custom launch file 

```
ros2 launch crazyflie_ros2_setpoint_follower launch_cf_hardware.launch.py 

```

this is just an helper node that launches crazyswarm with some specific parameters that will be useful for this simulation. 


## Undesratnding the misison file

The whole mission is defined by the mission file that you have created in your `missions/` folder. A simple example could be given by this file 


```

/**: 
  ros__parameters:
    
    ###########################################
    # General node parameters 
    ###########################################
    DIM: 2               # Dimension of the space
    MANAGER_TIMER: 0.8   # Time at which the manager updates (seconds)
    AGENT_TIMER: 0.1     # frequency of the MPC (seconds)
    SPEED: 0.4           # Nominal speed of the agents
    BOX_WEIGHT: 10       # Weight of the boxes in the cost function
    COMM_DISTANCE: 2.    # Communication distance between agents
    HOOVERING_HEIGHT: 1. # Alitude at which the drones move nominally

    ###########################################################################################################
    # Specific of each agent. The agent number corresponds to the crazyflie number (check the tag on the drone).
    # N.B. Make sure that a drone with such number exists in the real fleet or in the simulation.
    ###########################################################################################################

    agent_1:
      pos:  [ -1.08,   -0.99,  0.3 ]
      radio: 0
      channel: 40

    agent_4:
      pos:  [ -1.1,  0.68,  0.3 ]
      radio: 0
      channel: 40

    agent_5:
      pos:  [ -0.52,  -0.18,  0.3 ]
      radio: 0
      channel: 40


    ###########################################################################################################
    # Tasks specifications
    ###########################################################################################################

    # Tasks grouping by period
    PERIODS:
      period_0: [20.0, 25.0]
      period_1: [50.0, 65.0]
      period_2: [85.0, 90.0]

    TASKS:
      # PERIOD 0 — LETTER K
      # =======================
      # Origin (0,0) is the center of the K.
      task_1: 
        period_num: 0             # period at which the task is accomplished 
        edges: [1, 1]             # edge that defines the relative position (nb: make sure that an agent with the given number exists)
        rel_position: [0., 0.0]   # relative position with respect to the edge defined by the two agents
        size: 0.2                 # size of the task (radius of the area to be covered)
        type: "always"
      task_2: # Vertical Bottom
        period_num: 0
        edges: [1, 4]
        rel_position: [0., -.9]
        size: 0.2
        type: "always"
      task_3: # Midpoint/Joint
        period_num: 0
        edges: [1, 5]
        rel_position: [0., .9]
        size: 0.2
        type: "always"

```

when launching the mission without human in the loop, the tasks will be taken from this file and given to your manager node.
When instead the mission is launched with human in the loop, then the tasks are given directly via text as high level commands from a human operator. In this case an LLM will parse the input for you and derive the required tasks online.

The commands to the llm are given simply by publishing a sting command as follows 


```
ros2 topic pub --once /operator_command std_msgs/String "{data: \"Agents 5,4 and 1 should form a triangle formation between 30 and 40. Then agents 5,4 and 1 should form a straigh line and Agent 1 is at position [0,0]\"}"
```



