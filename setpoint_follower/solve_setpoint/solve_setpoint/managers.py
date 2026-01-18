import sys
import rclpy
import signal
import numpy as np

from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32MultiArray
from rclpy.executors import MultiThreadedExecutor
from msg_interface.msg import TaskEdge, TaskEdgeList
from barrier_msg.srv import BCompSrv
from barrier_msg.msg import BMsg, TMsg, BMsglist

from solve_setpoint.bMsg import bMsg, HyperCubeHandler
from solve_setpoint.graph_manager import GraphManager
from solve_setpoint.task_manager import TaskManager, Task
from solve_setpoint.solvers.combined_solve import solve_combined

class State:
    START_DRONE = 0
    MAINLOOP = 1

class Mode:
    SIM = 0
    REAL = 1


class Manager(Node):
    """
    The manager handles the task graph and agent graph
    its main functions are setting the setpoint for the agents
    these setpoints are calculated using optimization methods on given tasks
    """
    def __init__(self):
        super().__init__('manager')
        self.declare_parameter('robot_prefix', '/crazyflie')
        self.robot_prefix = self.get_parameter('robot_prefix').value

        self.declare_parameter("SYSTEM", 2)
        self.declare_parameter("DIM", 2)
        self.declare_parameter("NUM_AGENTS", 10)
        self.declare_parameter("MAIN_TIMER", 0.1)
        self.declare_parameter("SETPOINT_TIMER", 0.1)
        self.declare_parameter("COMM_DISTANCE", 1.1)
        self.declare_parameter("BOX_WEIGHT", 10)

        self.SYSTEM = self.get_parameter("SYSTEM").value
        self.DIM = self.get_parameter("DIM").value
        self.NUM_AGENTS = self.get_parameter("NUM_AGENTS").value
        self.MAIN_TIMER = self.get_parameter("MAIN_TIMER").value
        self.SETPOINT_TIMER = self.get_parameter("SETPOINT_TIMER").value
        self.COMM_DISTANCE = self.get_parameter("COMM_DISTANCE").value
        self.BOX_WEIGHT = self.get_parameter("BOX_WEIGHT").value

        #setup the tasks that are to be done
        if self.DIM  == 2:
            self.tasks = [
                Task([3,9], [0,1], [0.0, -0.8]),
                Task([3,9], [1,2], [-0.8, 0.0]),
                Task([10,15], [0,1], [-0.8, 0.0]),
                Task([10,15], [1,2], [0.0, -0.8]),
            ]
            self.tasks = [
                Task([3,9], [0,1], [0.0, -0.6]),
                Task([3,9], [2,0], [0.0, -0.6]),
                Task([3,9], [1,3], [0.0, -0.6]),
                Task([10,15], [0,1], [0.0, -0.8]),
                Task([10,15], [2,0], [-0.8, 0.0]),
                Task([10,15], [1,3], [-0.8, 0.0]),
            ]
        elif self.DIM  == 3:
            self.tasks = [
                Task([4,9], [0,9], [-2,-1, 1]),
                Task([4,9], [4,5], [0, 1.6, 0.5]),
                Task([10,13], [1,8], [-2, 1, 0.2]),
                Task([10,13], [7,2], [0.2, 1.6, -1])]

        #setup the graph manager and task manager
        self.graph_manager = GraphManager(self.NUM_AGENTS, self.COMM_DISTANCE)
        self.task_manager = TaskManager(self.DIM, self.COMM_DISTANCE, self.tasks)

        self.recalc_times = self.task_manager.recalculate_at()
        self.triggered_times = set()

        #setup all the subscribers and publishers
        if self.SYSTEM == Mode.SIM:
            self.drones = ['{}{}'.format(self.robot_prefix, i) for i in range(1,self.NUM_AGENTS+1)]
        elif self.SYSTEM == Mode.REAL:
            self.drones = []

            for srv_name, srv_types in self.get_service_names_and_types():
                if 'crazyflie_interfaces/srv/StartTrajectory' in srv_types:
                    # remove '/' and '/start_trajectory'
                    cfname = srv_name[1:-17]
                    if cfname != 'all':
                        self.drones.append(cfname)
        self.odom_subscribers = []

        self.edge_publisher = self.create_publisher(Int32MultiArray, "/graph_edges", 10)
        self.task_publisher = self.create_publisher(TaskEdgeList, "/task_edges", 10)

        self.barrier_client = self.create_client(BCompSrv, '/compute_barriers')
        while not self.barrier_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.barrier_publisher = self.create_publisher(BMsglist, "/barriers", 10)
        
        odom_name = {Mode.SIM: "/odom", Mode.REAL: "/pose"}
        odom_type = {Mode.SIM: Odometry, Mode.REAL: PoseStamped}

        for i, drone in enumerate(self.drones):
            callback = lambda msg, idx=i: self.graph_manager.set_pos_callback(msg, idx, self.get_logger().info)
            self.odom_subscribers.append(self.create_subscription(
                odom_type[self.SYSTEM], drone + odom_name[self.SYSTEM], callback, 10))

        #start the main loops of the system with a timer method
        self.timer = self.create_timer(self.MAIN_TIMER, self.mainloop)
        self.once = True
        self.counter = 0
            
        #set the starting state
        self.state = State.START_DRONE
        modes = {0: "simulation", 1: "real"}
        self.get_logger().info(f"\n\nStarting the manager in {modes[self.SYSTEM]} mode!!\n\n")

    def send_barrier(self, future):
        try:
            self.msgs = []
            response = future.result()

            msg = BMsglist()
            msg.messages.extend(response.messages)
            self.barrier_publisher.publish(msg)

        except Exception as e:
            self.get_logger().info(f"Service call failed: {e}", )

    def graph_rviz(self, time_recalc):
        edge_recalc = self.graph_manager.calc_edges() 

        #publish edge data for the graph_rviz script if active
        if edge_recalc and self.edge_publisher.get_subscription_count() > 0:
            msg = Int32MultiArray()
            msg.data = [i for pair in self.graph_manager.get_edge() for i in pair]
            self.edge_publisher.publish(msg)

        #publish task data for the graph_rviz script if active
        if time_recalc and self.task_publisher.get_subscription_count() > 0:
            tasks = self.task_manager.get_tasks(self.total_elapsed)

            msg = TaskEdgeList()
            for task in tasks:
                task_edge = TaskEdge()
                task_edge.start_idx = task[0][0]
                task_edge.end_idx = task[0][1]
                task_edge.agent_x = float(task[1][0])
                task_edge.agent_y = float(task[1][1])
                if self.DIM==2:
                    task_edge.agent_z = 0.
                else:
                    task_edge.agent_z = float(task[1][2])

                msg.task_list.append(task_edge)

            self.task_publisher.publish(msg)


    def mainloop(self):
        if self.state == State.START_DRONE:
            #check whether all drones have published their odometry
            if np.all(self.graph_manager.online_status):
                #change state
                self.get_logger().info("Received odom message from all agents, changing state")
                self.state = State.MAINLOOP

                #process the edges now that every agent is online
                self.graph_manager.calc_edges()
                if self.edge_publisher.get_subscription_count() > 0:
                    msg = Int32MultiArray()
                    msg.data = [i for pair in self.graph_manager.get_edge() for i in pair]
                    self.edge_publisher.publish(msg)

                #set the starting time
                self.start_time = self.get_clock().now().nanoseconds/1e9
                self.prev_time = self.start_time

            else:
                self.counter += 1
                if self.counter == 50:
                    self.get_logger().info("Not all agents online yet")
                    self.counter = 0

        elif self.state == State.MAINLOOP:
            self.current_time = self.get_clock().now().nanoseconds/1e9
            self.delta_time = self.current_time - self.prev_time
            self.total_elapsed = self.current_time - self.start_time
            self.prev_time = self.current_time

            time_recalc = False
            for t in self.recalc_times:
                if t not in self.triggered_times and self.total_elapsed >= t:
                    self.triggered_times.add(t)
                    time_recalc = True

            #returns whether the edges have changed
            self.graph_rviz(time_recalc)

            #-------------------------------
            #    Main optimization logic
            #-------------------------------
            if time_recalc:# or edge_recalc:
                if time_recalc:
                    self.get_logger().info("recaculating at: {}".format(self.total_elapsed))

                #calculate the current relative tasks based on the current weighted communication graph
                comm_graph = self.graph_manager.get_comm_graph()
                weights = self.graph_manager.get_weights()
                
                #task_paths, task_pos, task_box = self.task_manager.obtain_current_tasks(
                #    self.total_elapsed, comm_graph, weights)
                edges = self.graph_manager.get_edge()

                
                task_paths, task_pos, task_box = self.task_manager.obtain_current_tasks2(
                    self.total_elapsed, edges, self.get_logger().info)

                if task_paths and "impossible" not in task_paths: #only perform if not empty, and valid paths
                    agents = self.graph_manager.get_pos()
                    return_mode = "relative"

                    if self.DIM == 2:
                        agents = agents[:,:2]

                    self.get_logger().info(f"{task_paths}")

                    succes, pos_result, box_result = solve_combined(
                        task_paths, task_pos, task_box, return_mode, self.BOX_WEIGHT, self.COMM_DISTANCE)

                    if succes: 
                        #compute the barrier function
                        self.req = BCompSrv.Request()
                        tmsg_list = self.task_manager.get_tmsg(self.total_elapsed, pos_result, box_result)
                        self.req.messages.extend(tmsg_list)
                        future = self.barrier_client.call_async(self.req)
                        future.add_done_callback(self.send_barrier)
                        self.get_logger().info(f"{future}")
                    else:
                        self.get_logger().info("The optimization failed")
                else:
                    if "impossible" in task_paths:
                        self.get_logger().info("No valid communication graph at this time")


def signal_handler(sig, frame):
    print("Shutdown signal received, cleaning up...")
    rclpy.shutdown()
    sys.exit(0)


def main(args=None):
    rclpy.init(args=args)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    manager = Manager()
    #executor so subcriptions and publishers can use mutliple threads
    executor = MultiThreadedExecutor()
    executor.add_node(manager)
    executor.spin()
    rclpy.shutdown()

if __name__ == "__main__":
    main()