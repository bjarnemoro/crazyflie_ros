import sys
import rclpy
import signal
import numpy as np

from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32MultiArray
from rclpy.executors import MultiThreadedExecutor
from msg_interface.msg import TaskEdge, TaskEdgeList
from barrier_msg.srv import BCompSrv
from barrier_msg.msg import BMsg, TMsg
from barrier_msg.msg import Config
from incorporate_barrier.MPC_barrier import optimize_path
#from solve_setpoint.config import Config

from solve_setpoint.bMsg import bMsg, HyperCubeHandler
from solve_setpoint.graph_manager import GraphManager
from solve_setpoint.task_manager import TaskManager, Task
from solve_setpoint.solvers.combined_solve import solve_combined

class State:
    START_DRONE = 0
    MAINLOOP = 1


class Manager(Node):
    """
    The manager handles the task graph and agent graph
    its main functions are setting the setpoint for the agents
    these setpoints are calculated using optimization methods on given tasks
    """
    def __init__(self):
        super().__init__('manager')
        self.declare_parameter('robot_prefix', '/manager')
        self.robot_prefix = self.get_parameter('robot_prefix').value

        self.get_logger().info(f"{Config.DIM}")

        #setup the tasks that are to be done
        if Config.DIM == 2:
            self.tasks = [
                Task([4,9], [0,9], [-2,-1]),
                Task([4,9], [4, 5], [0, 1.6]),
                Task([10,13], [1,8], [-2, 1]),
                Task([10,13], [7,2], [0.2, 1.6])]
            self.tasks = [
                Task([4,9], [5,8], [0,-1]),
                Task([4,9], [8,6], [0,-1]),
                Task([4,9], [6,9], [0,-1]),
                Task([4,9], [9,7], [0,-1]),
                #Task([4,9], [6,3], [1,1]),
                #Task([4,9], [6,1], [1,-1]),
                #Task([4,9], [6, 0], [-1, 1]),
                #Task([4,9], [6, 1], [1, 1]),
                Task([10,13], [1,8], [-2, 1]),
                Task([10,13], [7,2], [0.2, 1.6])]
        elif Config.DIM == 3:
            self.tasks = [
                Task([4,9], [0,9], [-2,-1, 1]),
                Task([4,9], [4,5], [0, 1.6, 0.5]),
                Task([10,13], [1,8], [-2, 1, 0.2]),
                Task([10,13], [7,2], [0.2, 1.6, -1])]

        #setup the graph manager and task manager
        self.graph_manager = GraphManager()
        self.task_manager = TaskManager(self.tasks)

        self.recalc_times = self.task_manager.recalculate_at()
        self.triggered_times = set()
        self.setpoints = np.zeros((Config.NUM_AGENTS, 3))

        #setup all the subscribers and publishers
        self.drones = ['/crazyflie{}'.format(i) for i in range(1,Config.NUM_AGENTS+1)]
        self.odom_subscribers = []
        self.setpoint_publishers = []

        self.edge_publisher = self.create_publisher(Int32MultiArray, "/graph_edges", 10)
        self.task_publisher = self.create_publisher(TaskEdgeList, "/task_edges", 10)

        self.barrier_client = self.create_client(BCompSrv, '/compute_barriers')
        while not self.barrier_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        for i, drone in enumerate(self.drones):
            callback = lambda msg, idx=i: self.graph_manager.set_pos_callback(msg, idx)
            self.odom_subscribers.append(self.create_subscription(
                Odometry, drone + '/odom', callback, 10))
            self.setpoint_publishers.append(self.create_publisher(Odometry, drone + '/set', 10)) 

        #start the main loops of the system with a timer method
        self.timer = self.create_timer(Config.MAIN_TIMER, self.mainloop)
        self.setpoint_timer = self.create_timer(Config.SETPOINT_TIMER, self.setpoint_update)
        self.once = True

        self.x = None
            
        #set the starting state
        self.state = State.START_DRONE
        self.get_logger().info("\n\nStarting manager!!\n\n")
        #self.get_logger().info(f"{self.barrier_client}")
        self.test_client()

    def test_client(self):
        pass
        #self.req = BCompSrv.Request()
        #tmsg = TMsg()
        #self.req.messages.append(tmsg)
        #self.future = self.barrier_client.call_async(self.req)
        #self.future.add_done_callback(self.my_callback)

    def my_callback(self, future):
        try:
            self.msgs = []
            response = future.result()

            for bmsg in response.messages:
                my_msg = HyperCubeHandler(
                    bmsg.slopes, 
                    bmsg.gamma0, 
                    bmsg.r, 
                    bmsg.slack, 
                    bmsg.b_vector, 
                    bmsg.time_grid, 
                    bmsg.task_id, 
                    bmsg.edge_i, 
                    bmsg.edge_j)
                
                self.msgs.append(my_msg)

            for bmsg in self.msgs:
                candidates = [bmsg2 for bmsg2 in self.msgs if bmsg2.edge_j == bmsg.edge_i]
                if candidates:
                    bmsg.add_neighbour(candidates[0])

        except Exception as e:
            self.get_logger().info(f"Service call failed: {e}", )

        HORIZON = 100
        DT = 0.1

        self.x = optimize_path(HORIZON, self.msgs, [i for i in range(10)], DT)

    def mainloop(self):
        if self.state == State.START_DRONE:
            #check whether all drones have published their odometry
            if np.all(self.graph_manager.online_status):
                #change state
                self.get_logger().info("Received odom message from all agents, changing state")
                self.state = State.MAINLOOP

                #make sure the drones have a z setpoint of 1
                self.graph_manager.init_setpoints()

                #process the edges now that every agent is online
                self.graph_manager.calc_edges()
                if self.edge_publisher.get_subscription_count() > 0:
                    msg = Int32MultiArray()
                    msg.data = [i for pair in self.graph_manager.get_edge() for i in pair]
                    self.edge_publisher.publish(msg)

                #set the starting time
                self.start_time = self.get_clock().now().nanoseconds/1e9
                self.prev_time = self.start_time

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
                    if Config.DIM==2:
                        task_edge.agent_z = 0.
                    else:
                        task_edge.agent_z = float(task[1][2])

                    msg.task_list.append(task_edge)

                self.task_publisher.publish(msg)

            #-------------------------------
            #    Main optimization logic
            #-------------------------------
            if time_recalc or edge_recalc:
                if time_recalc:
                    self.get_logger().info("recaculating at: {}".format(self.total_elapsed))

                #calculate the current relative tasks based on the current weighted communication graph
                comm_graph = self.graph_manager.get_comm_graph()
                weights = self.graph_manager.get_weights()
                
                task_paths, task_pos, task_box = self.task_manager.obtain_current_tasks(
                    self.total_elapsed, comm_graph, weights)

                if task_paths and "impossible" not in task_paths: #only perform if not empty, and valid paths
                    agents = self.graph_manager.get_pos()
                    return_mode = "relative"

                    if Config.DIM == 2:
                        agents = agents[:,:2]

                    #pos_result = solve_task(agents, task_paths, task_pos, return_mode)
                    #rad_result = solve_radius(agents, task_paths, task_rad, return_mode)

                    succes, pos_result, box_result = solve_combined(task_paths, task_pos, task_box, return_mode)

                    if succes:
                        self.graph_manager.compute_setpoint(pos_result)
                        
                        #compute the barrier function
                        self.req = BCompSrv.Request()
                        tmsg_list = self.task_manager.get_tmsg(self.total_elapsed, pos_result, box_result)
                        self.req.messages.extend(tmsg_list)
                        future = self.barrier_client.call_async(self.req)
                        future.add_done_callback(self.my_callback)
                        self.get_logger().info(f"{future}")
                    else:
                        self.get_logger().info("The optimization failed")
                else:
                    if "impossible" in task_paths:
                        self.get_logger().info("No valid communication graph at this time")

    def setpoint_update(self):
        if self.state == State.MAINLOOP:
            time = self.get_clock().now().nanoseconds
            time_sec = time / 1e9

            setpoints = self.graph_manager.get_setpoints()

            for setpoint, pub in zip(setpoints, self.setpoint_publishers):
                msg = Odometry()
                msg.pose.pose.position.x = setpoint[0]
                msg.pose.pose.position.y = setpoint[1]
                if Config.DIM == 2:
                    msg.pose.pose.position.z = 1.#1+np.cos(time_sec)*0.5#setpoint[2]
                elif Config.DIM == 3:
                    msg.pose.pose.position.z = setpoint[2]

                pub.publish(msg)

    def setpoint_update_MPC(self):
        if self.state == State.MAINLOOP:
            time = self.get_clock().now().nanoseconds
            time_sec = time / 1e9

            if self.x is not None:
                idx = int(time_sec * 10)

                if idx < 100:
                    for i, pub in enumerate(self.setpoint_publishers):
                        msg = Odometry()
                        msg.pose.pose.position.x = self.x[idx, i*2]
                        msg.pose.pose.position.y = self.x[idx, i*2+1]
                        if Config.DIM == 2:
                            msg.pose.pose.position.z = 1.#1+np.cos(time_sec)*0.5#setpoint[2]
                        elif Config.DIM == 3:
                            msg.pose.pose.position.z = self.x[idx, i*2+2]

                        pub.publish(msg)



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