import rclpy
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict
from rclpy.node import Node
from copy import deepcopy
from enum import Enum

from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from solve_setpoint.solvers.shortest_path import get_shortest_distance
from solve_setpoint.solvers.radius_solver import solve_radius
from solve_setpoint.solvers.task_solver import solve_task

from solve_setpoint.task_manager import TaskManager, Task
from solve_setpoint.graph_manager import GraphManager

NUM_DRONES = 10
MODE = "2d"#"2d" or "3d"

class Manager(Node):
    """Provides the interface to the outside
    by means of a task file which gives tasks over time 
    """
    def __init__(self):
        super().__init__('manager')
        self.declare_parameter('robot_prefix', '/manager')
        self.robot_prefix = self.get_parameter('robot_prefix').value

        #setup the graph manager and task manager
        self.graph_manager = GraphManager()

        self.edges = [[0,1], [0,2], [0,3], [1,3], [1,4], [2,3], [3,4], [2,5], [3,6], [4,7], [8,9], [5,8], [6,8] ,[6,9], [7,9], [5,6], [6,7]]
        self.graph_manager.set_edges(self.edges)

        msg = Int32MultiArray()
        msg.data = [i for pair in self.edges for i in pair]
        self.edge_publisher = self.create_publisher(Int32MultiArray, "/graph_edges", 10)
        self.edge_publisher.publish(msg)


        if MODE == "2d":
            self.tasks = [
                Task([5,8], [0,9], [-2,-1]),
                Task([5,8], [4, 5], [0, 1.6]),
                Task([10,13], [1,8], [-2, 1]),
                Task([10,13], [7,2], [0.2, 1.6])]
        elif MODE == "3d":
            self.tasks = [
                Task([5,8], [0,9], [-2,-1, 1]),
                Task([5,8], [4,5], [0, 1.6, 0.5]),
                Task([10,13], [1,8], [-2, 1, 0.2]),
                Task([10,13], [7,2], [0.2, 1.6, -1])]

        self.task_manager = TaskManager(self.tasks)
        self.recalc_times = self.task_manager.recalculate_at()

        self.setpoints = np.zeros((NUM_DRONES, 3))

        #connect the graph manager to the realtime position of the drones
        self.drones = ['/crazyflie{}'.format(i) for i in range(1,11)]
        self.odom_subscribers = []
        self.setpoint_publishers = []
        for i, drone in enumerate(self.drones):
            callback = lambda msg, idx=i: self.graph_manager.set_pos(msg, idx)
            self.odom_subscribers.append(self.create_subscription(
                Odometry, drone + '/odom', callback, 10))
            self.setpoint_publishers.append(self.create_publisher(Odometry, drone + '/set', 10))

        self.get_logger().info("\n\nStarting manager!!\n\n")

        self.succesful_start = False
        self.in_position = False

        self.timer = self.create_timer(0.1, self.mainloop)
        self.setpoint_timer = self.create_timer(0.1, self.setpoint_update)
        self.once = True
            

        self.state = 0

    def mainloop(self):
        if self.state == 0:
            if np.all(self.graph_manager.online_status):
                self.get_logger().info("Received odom message from all agents, changing state")
                self.state += 1
                self.graph_manager.set_setpoints() #TEMP!!

                self.start_time = self.get_clock().now().nanoseconds/1e9
                self.prev_time = self.start_time

        elif self.state == 1:
            self.current_time = self.get_clock().now().nanoseconds/1e9
            self.delta_time = self.current_time - self.prev_time
            self.total_elapsed = self.current_time - self.start_time
            self.prev_time = self.current_time

            msg = Int32MultiArray()
            msg.data = [i for pair in self.edges for i in pair]
            self.edge_publisher = self.create_publisher(Int32MultiArray, "/graph_edges", 10)
            self.edge_publisher.publish(msg)

            for t in self.recalc_times:
                if self.total_elapsed-self.delta_time < t and self.total_elapsed >= t:
                    self.get_logger().info("recaculating at: {}".format(self.total_elapsed))

                    comm_graph = self.graph_manager.get_comm_graph()
                    
                    agents = self.graph_manager.get_pos()
                    task_paths, task_pos, task_rad = self.task_manager.obtain_current_tasks(self.total_elapsed, comm_graph)

                    if task_paths: #only perform if not empty, so there must be tasks
                        return_mode = "relative"

                        if MODE == "2d":
                            agents = agents[:,:2]

                        pos_result = solve_task(agents, task_paths, task_pos, return_mode)
                        rad_result = solve_radius(agents, task_paths, task_rad, return_mode)

                        self.graph_manager.compute_setpoint(pos_result)

                        #plot results:
                        pos_result = solve_task(agents, task_paths, task_pos, "absolute")
                        results = self.graph_manager.get_pos()
                        for idx, val in pos_result:
                            results[idx][:2] = val[:2]
                        self.graph_manager.show_graph(True, results) 

    def setpoint_update(self):
        if self.state == 1:
            time = self.get_clock().now().nanoseconds
            time_sec = time / 1e9

            setpoints = self.graph_manager.get_setpoints()
            msg = Odometry()

            for setpoint, pub in zip(setpoints, self.setpoint_publishers):
                
                msg.pose.pose.position.x = setpoint[0]
                msg.pose.pose.position.y = setpoint[1]
                if MODE == "2d":
                    msg.pose.pose.position.z = 1.#1+np.cos(time_sec)*0.5#setpoint[2]
                elif MODE == "3d":
                    msg.pose.pose.position.z = setpoint[2]

                pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    manager = Manager()
    rclpy.spin(manager)
    rclpy.shutdown()

if __name__ == "__main__":
    main()