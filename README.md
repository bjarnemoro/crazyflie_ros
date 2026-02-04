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
ros2 launch crazyflie_ros2_setpoint_follower solver.launch.py mission_yaml:=simple.yaml backend:=hardware hil:=false

```

2. with human in the loop

```
ros2 launch crazyflie_ros2_setpoint_follower solver.launch.py mission_yaml:=simple.yaml backend:=hardware hil:=true

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


2. Once your crazyflies are ready you should **first** run the launch file for your mission

```
ros2 launch crazyflie_ros2_setpoint_follower solver.launch.py mission_yaml:=simple.yaml backend:=hardware hil:=true

```










```
ros2 launch crazyflie launch.py backend:=cflib topics.poses.qos.mode:=sensor
ros2 launch crazyflie_ros2_setpoint_follower solver_real.launch.py
```

### changing tasks
the tasks are loaded in the setpoint_follower/solve_setpoint/solve_setpoint/managers.py
where the TaskManager is initialized via
```
self.task_manager = TaskManager(self.DIM, self.tasks)
```
the task manager allows for loading via a list of tasks or a json file
```
class TaskManager():
    def __init__(self, dim, tasks=None, task_path=None):
        """load the the tasks either via a json file or an array of tasks with the following structure:
        [([],[],[]), ..., ([],[],[])] with each tuple having time period, agent idx, relative pos
        i.e [0,3], [0, 4], [10, 20] so from time 0 to 3 sec agent 0 has a postion of [10, 20] compared to agent 4"""
```
when loading via the list of tasks simply append them like this when initialzing the manager, with the first list time period, second the index of the agents, and the last the relative position between them.

```
self.tasks = [
    Task([4,9], [0,9], [-2,-1]),
    Task([4,9], [4, 5], [0, 1.6]),
    Task([10,13], [1,8], [-2, 1]),
    Task([10,13], [7,2], [0.2, 1.6])]

```
