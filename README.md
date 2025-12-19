# crazyflie_ros
### setup

### installation
clone the repository, note that the submodules are not on the main branch
```
git clone --recursive https://github.com/bjarnemoro/crazyflie_ros.git
```
build the crazyflie_ros directory
```
colcon build --symlink-install
```

### running
you can run the main simulation in gazebo
```
ros2 launch crazyflie_ros2_setpoint_follower solver.launch.py 
```
for additional visualization of the graphs and tasks in rviz rn
```
ros2 launch crazyflie_ros2_setpoint_follower crazyflie_rviz.launch.py 
```

The real world file is currently missing however in order to run that you connect to the crazyflie using the crazyswarm library
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
