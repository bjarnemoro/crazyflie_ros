import sys
import rclpy
import signal
import numpy as np

from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int32
from rclpy.executors import MultiThreadedExecutor
from msg_interface.msg import TaskEdge, TaskEdgeList
from barrier_msg.srv import BCompSrv
from barrier_msg.msg import BMsg, TMsg, BMsglist

from solve_setpoint.bMsg import bMsg, HyperCubeHandler
from solve_setpoint.graph_manager import GraphManager
from solve_setpoint.task_manager import TaskManager, Task
from solve_setpoint.solvers.combined_solve import solve_combined
from solve_setpoint.utils import WorkingMode, AgentState, ManagerState


class Manager(Node):
    """
    The manager handles the task graph and agent graph
    its main functions are setting the setpoint for the agents
    these setpoints are calculated using optimization methods on given tasks
    """
    def __init__(self):
        super().__init__('manager',allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides = True)
        
        # Parameters are declared from indie the yaml file. Since we have set 
        # automatically_declare_parameters_from_overrides = true, then all the parameters inside
        # the yaml file will be already automatically declared.

        # self.declare_parameter('robot_prefix', '/crazyflie')
        # self.declare_parameter("SYSTEM", 2)
        # self.declare_parameter("DIM", 2)
        # self.declare_parameter("NUM_AGENTS", 10)
        # self.declare_parameter("MAIN_TIMER", 0.1)
        # self.declare_parameter("SETPOINT_TIMER", 0.1)
        # self.declare_parameter("COMM_DISTANCE", 1.1)
        # self.declare_parameter("BOX_WEIGHT", 10)
        
        self.robot_prefix = self.get_parameter('robot_prefix').value
        self.SYSTEM = self.get_parameter("SYSTEM").value
        self.DIM = self.get_parameter("DIM").value
        self.NUM_AGENTS = self.get_parameter("NUM_AGENTS").value
        self.MAIN_TIMER = self.get_parameter("MAIN_TIMER").value
        self.SETPOINT_TIMER = self.get_parameter("SETPOINT_TIMER").value
        self.COMM_DISTANCE = self.get_parameter("COMM_DISTANCE").value
        self.BOX_WEIGHT = self.get_parameter("BOX_WEIGHT").value



        #setup the tasks that are to be done
        self.tasks = self._load_tasks()
        self.get_logger().info(f"Loaded {len(self.tasks)} tasks")

        

        #setup the graph manager and task manager
        self.graph_manager = GraphManager(self.NUM_AGENTS, self.COMM_DISTANCE)
        self.task_manager = TaskManager(self.DIM, self.COMM_DISTANCE, self.tasks)

        self.recalc_times = self.task_manager.recalculate_at()
        self.triggered_times = set()

        #setup all the subscribers and publishers
        if self.SYSTEM == WorkingMode.SIM:
            self.SYSTEM = WorkingMode.SIM
            self.drones = ['{}{}'.format(self.robot_prefix, i) for i in range(1,self.NUM_AGENTS+1)]
        
        elif self.SYSTEM == WorkingMode.REAL:
            self.SYSTEM = WorkingMode.REAL
            self.drones = []

            for srv_name, srv_types in self.get_service_names_and_types():
                if 'crazyflie_interfaces/srv/StartTrajectory' in srv_types:
                    # remove '/' and '/start_trajectory'
                    cfname = srv_name[1:-17]
                    if cfname != 'all':
                        self.drones.append(cfname)
        else :
            raise ValueError(f"The specified working mode {self.SYSTEM} is not valid. Choose 1 for real robots and 0 for simulation.")
        
        
        self.odom_subscribers = []
        self.edge_publisher   = self.create_publisher(Int32MultiArray, "/graph_edges", 10)
        self.task_publisher   = self.create_publisher(TaskEdgeList, "/task_edges", 10)
        self.agent_state_sub  = self.create_subscription(Int32, "/agent_state", self.on_agent_message, 10)

        self.barrier_client = self.create_client(BCompSrv, '/compute_barriers')
        while not self.barrier_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.barrier_publisher = self.create_publisher(BMsglist, "/barriers", 10)
        
        odom_name = {WorkingMode.SIM: "/odom", WorkingMode.REAL: "/pose"}
        odom_type = {WorkingMode.SIM: Odometry, WorkingMode.REAL: PoseStamped}

        for i, drone in enumerate(self.drones):
            callback = lambda msg, idx=i: self.graph_manager.set_pos_callback(msg, idx, self.get_logger().info)
            self.odom_subscribers.append(self.create_subscription(
                odom_type[self.SYSTEM], drone + odom_name[self.SYSTEM], callback, 10))

        #start the main loops of the system with a timer method
        self.timer = self.create_timer(self.MAIN_TIMER, self.mainloop)
        self.once = True
        self.counter = 0
            
        #set the starting state
        self.manager_state = ManagerState.WAITING_FOR_ODOMETRY
        self.agent_state   = None
        modes      = {0: "simulation", 1: "real"}
        self.get_logger().info(f"Working in {self.SYSTEM.name} mode. Manager state {self.manager_state.name} !!")

    def send_barrier(self, future):
        try:
            self.msgs = []
            response = future.result()

            msg = BMsglist()
            msg.messages.extend(response.messages)
            self.barrier_publisher.publish(msg)

        except Exception as e:
            self.get_logger().info(f"Service call failed: {e}", )

    def on_agent_message(self,msg):
        if msg.data == AgentState.TAKEOFF:
            # Logic for ascending to target altitude
            self.agent_state = AgentState.TAKEOFF
            
        elif msg.data == AgentState.READY:
            # Logic for mission execution or hovering
            self.agent_state = AgentState.READY
            
        elif msg.data == AgentState.LANDING:
            # Logic for descending and disarming
            self.agent_state = AgentState.LANDING
            
        else:
            self.get_logger().info("Error: Unknown Agent State.")

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
        if self.manager_state == ManagerState.WAITING_FOR_ODOMETRY:
            #check whether all drones have published their odometry
            if np.all(self.graph_manager.online_status):
                #change state
                self.get_logger().info(f"Received odom message from all agents, changing manager state to: {ManagerState.READY.name}")
                self.manager_state = ManagerState.READY

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

        elif self.manager_state == ManagerState.READY and self.agent_state == AgentState.READY:
            
            self.current_time  = self.get_clock().now().nanoseconds/1e9
            self.delta_time    = self.current_time - self.prev_time
            self.total_elapsed = self.current_time - self.start_time
            self.prev_time     = self.current_time

            time_recalc = False
            for t in self.recalc_times:
                if t not in self.triggered_times and self.total_elapsed >= t:
                    self.triggered_times.add(t)
                    time_recalc = True

            #returns whether the edges have changed
            # self.graph_rviz(time_recalc)

            #-------------------------------
            #    Main optimization logic
            #-------------------------------
            if time_recalc:# or edge_recalc:
                self.get_logger().info("Recaculating at: {}".format(self.total_elapsed))

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

    def _load_tasks(self):
        """
        Load TASKS from ROS 2 parameters using prefix parsing.
        taken from https://gist.github.com/agrueneberg/d76fff493753fa531f1b5d33be0f07ed
        """

        params = self.get_parameters_by_prefix('TASKS')

        if not params:
            self.get_logger().warn("No TASKS parameters found")
            return []

        tasks_by_name = {}

        for full_key, param in params.items():
            # Example: "task_3.timespan"
            task_name, field = full_key.split(".", 1)

            tasks_by_name.setdefault(task_name, {})[field] = param.value

            self.get_logger().info(
                f"Loaded param: {task_name}.{field} = {param.value}"
            )

        tasks = []
        for name, data in tasks_by_name.items():
            try:
                tasks.append(Task(**data))
            except Exception as e:
                self.get_logger().error(
                    f"Failed to create Task '{name}': {e}"
                )

        return tasks



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