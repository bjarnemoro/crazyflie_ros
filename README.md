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
