import sys
import rclpy
import signal
import numpy as np

from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int32, Bool
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from barrier_msg.srv import BCompSrv
from barrier_msg.msg import BMsg, TMsg, BMsglist, TMsglist

from solve_setpoint.bMsg import bMsg, HyperCubeHandler
from solve_setpoint.graph_manager import GraphManager
from solve_setpoint.task_manager import TaskManager, Task
from solve_setpoint.solvers.combined_solve import solve_task_decomposition
from solve_setpoint.utils import WorkingMode, AgentState, ManagerState, AnsiColor


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


        ##
        self.start_time = None
        self.ready_count = 0
        self.current_tasks = []

        #setup the tasks that are to be done
        self.tasks, self.periods = self._load_tasks()
        self.get_logger().info(f"Loaded {len(self.tasks)} tasks")

        #setup the graph manager and task manager
        self.graph_manager = GraphManager(self.NUM_AGENTS, self.COMM_DISTANCE)
        self.task_manager = TaskManager(self.DIM, self.COMM_DISTANCE, self.periods, self.tasks)

        self.recalc_times    = self.task_manager.recalculate_at()
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
        
        
        self.odom_subscribers     = []
        self.edge_pub                = self.create_publisher(Int32MultiArray, "/graph_edges", 10)
        self.active_agents_pub       = self.create_publisher(Int32MultiArray, "/active_agents", 10)
        self.task_pub                = self.create_publisher(TMsglist, "/task_edges", 10)

        self.landing_command_pub  = self.create_publisher(Bool, "/landing_command", 10)
        self.gather_command_pub   = self.create_publisher(Bool, "/gather_command", 10)
        self.agent_state_sub      = self.create_subscription(Int32, "/agent_state", self.on_agent_message, 10)

        self.barrier_client       = self.create_client(BCompSrv, '/compute_barriers')
        while not self.barrier_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.barrier_publisher = self.create_publisher(BMsglist, "/barriers", 10)
        
        odom_name = {WorkingMode.SIM: "/odom", WorkingMode.REAL: "/pose"}
        odom_type = {WorkingMode.SIM: Odometry, WorkingMode.REAL: PoseStamped}

        for i,drone in enumerate(self.drones, start=1):
            callback = lambda msg, idx=i: self.graph_manager.set_pos_callback(msg, idx, self.get_logger().info)
            self.odom_subscribers.append(self.create_subscription(
                odom_type[self.SYSTEM], drone + odom_name[self.SYSTEM], callback, 10))

        #start the main loops of the system with a timer method
        self.timer = self.create_timer(self.MAIN_TIMER, self.mainloop)
            
        #set the starting state
        self.manager_state = ManagerState.WAITING_FOR_ODOMETRY
        self.agent_state   = None
        modes      = {0: "simulation", 1: "real"}
        self.get_logger().info(f"Working in {self.SYSTEM.name} mode. Manager state {self.manager_state.name} !!")

    def on_new_barrier(self, future):
        try:
            response = future.result()
            msg = BMsglist()
            msg.messages.extend(response.messages)
            self.barrier_publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

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

        elif msg.data == AgentState.GATHERING:
            self.agent_state = AgentState.GATHERING

        elif msg.data == AgentState.STOP:
            self.agent_state = AgentState.STOP

        else:
            self.get_logger().info("Error: Unknown Agent State.")

    def is_rviz_node_connected(self):
        return self.edge_pub.get_subscription_count() > 0 and self.task_pub.get_subscription_count() > 0

    def update_and_publish_communication_graph(self):
        
        #process the edges now that every agent is online
        self.graph_manager.compute_current_communication_edges(self.get_logger())
        changed = self.graph_manager.did_graph_change()
        if changed:
            self.graph_manager.update() # update to the currently computed edges

        if self.is_rviz_node_connected():
            msg = Int32MultiArray()
            msg.data = [i for pair in self.graph_manager.get_edge() for i in pair]
            self.edge_pub.publish(msg) # publish for rviz
        
        return changed

    def update_and_publish_task_graph(self, current_time, communication_graph_changed ):

        # Check if it is time for the recalulcation     
        if len(self.recalc_times) == 0:
            self.get_logger().info(f"{AnsiColor.BOLD_GREEN} All tasks concluded. Sending landing command {AnsiColor.RESET}",throttle_duration_sec=5.0)
            self.landing_command_pub.publish(Bool(data=True))
            return    
        
        if current_time > self.recalc_times[0]:
            self.recalc_times.pop(0) # empty list as tasks are finished
            if len(self.recalc_times): # when we reach the last recalculation time then the tasks are finnished
                self.recalculate_tasks(current_time)
                self.get_logger().info(f"{AnsiColor.BOLD_GREEN} Recaculating task at recalculation time : {current_time}" \
                                    f".Remaining recalculation times: {self.recalc_times} {AnsiColor.RESET}")
        
        if communication_graph_changed:
            is_critical = False
            for task in self.current_tasks:
                u,v = task.edges
                if (u,v) not in self.graph_manager.get_edge() and (v,u) not in self.graph_manager.get_edge():
                   is_critical = True
                   self.recalculate_tasks(current_time)
                   self.get_logger().info(f"{AnsiColor.BOLD_GREEN} Recaculating task  after communication graph changed, affecting current tasks {AnsiColor.RESET}")
                   break
            if not is_critical:
                self.get_logger().info(f"{AnsiColor.BOLD_GREEN} Non critical graph change. Tasks are not recomputed {AnsiColor.RESET}")
            
       
        # publish task data for the graph_rviz script if active
        if self.is_rviz_node_connected():

            msg = Int32MultiArray()
            msg.data = self.task_manager.get_active_agents(current_time)
            self.active_agents_pub.publish(msg)


    

    def request_barrier(self, tasks):
        req = BCompSrv.Request()
        for task in tasks:
            req.messages.append(self.task_manager.get_tmsg(task))

        future = self.barrier_client.call_async(req)
        future.add_done_callback(self.on_new_barrier)



    def recalculate_tasks(self, current_time):
        #TODO: In case task is not feasible retrun the minimum violating solution
        #calculate the current relative tasks based on the current weighted communication graph
        
        comm_graph = self.graph_manager.get_comm_graph()
        comm_edges = self.graph_manager.get_edge()
        self.get_logger().debug(f"{AnsiColor.BOLD_GREEN} Current communication edges: {comm_edges} {AnsiColor.RESET}")

        tasks, possible_task_paths, decomposition_needed = self.task_manager.get_active_tasks_and_decomposition_paths(current_time, comm_edges, self.get_logger())

        if not decomposition_needed:
            self.current_tasks = tasks
            self.request_barrier(tasks)

        for jj,task_paths in enumerate(possible_task_paths,start=1) :

            cvx_status, new_tasks = solve_task_decomposition(tasks, task_paths,self.BOX_WEIGHT, self.COMM_DISTANCE, self.get_logger())
            if cvx_status == "optimal" : 
                #compute the barrier function
                self.get_logger().info(f"{AnsiColor.BOLD_GREEN} All tasks can be decomposed successfully!{AnsiColor.RESET}")
                self.current_tasks = new_tasks
                self.request_barrier(new_tasks)
                
                if self.agent_state == AgentState.GATHERING :
                    self.gather_command_pub.publish(Bool(data=False))
                
                return # exit loop 
                
            else:
                self.get_logger().info(f"{AnsiColor.BOLD_YELLOW} Decomposition paths {task_paths} failed with status {cvx_status} . Trying another decomposition... {AnsiColor.RESET}")

        
        self.get_logger().error("All task decomposition attempts failed.")
        if self.agent_state != AgentState.GATHERING:
            self.get_logger().info(f"{AnsiColor.BOLD_RED} Call gathering command since no decomposition was successful. {AnsiColor.RESET}")
            self.gather_command_pub.publish(Bool(data=True))


    def mainloop(self):
        
        if self.manager_state == ManagerState.WAITING_FOR_ODOMETRY:
            #check whether all drones have published their odometry
            if self.graph_manager.are_all_agents_online():
                #change state
                self.get_logger().info(f"{AnsiColor.BOLD_GREEN}Received odom message from all agents, changing manager state to: {ManagerState.READY.name} {AnsiColor.RESET}")
                self.manager_state = ManagerState.READY

                #publish the initial communication graph
                self.update_and_publish_communication_graph()
            
            else:
                missing_agents =  self.graph_manager.offline_agents()
                self.get_logger().info(f"Waiting for odom messages from agents: {missing_agents}")

        elif self.manager_state == ManagerState.READY and (self.agent_state == AgentState.READY or self.agent_state == AgentState.GATHERING):

            
            self.get_logger().info(f"{AnsiColor.BOLD_GREEN} MPC currently mode {self.agent_state.name}. {AnsiColor.RESET}",throttle_duration_sec=5.0)
      
            if self.start_time is None: # time is considered since the start of the mission
                self.start_time = self.get_clock().now()
            
            current_time = self.get_clock().now() - self.start_time
            current_time = current_time.nanoseconds / 1e9
            
            comm_graph_changed = self.update_and_publish_communication_graph()
            self.update_and_publish_task_graph(current_time,comm_graph_changed)

        elif self.agent_state == AgentState.STOP :
            self.get_logger().info(f"{AnsiColor.BOLD_GREEN} MPC externally stopped . {AnsiColor.RESET}",throttle_duration_sec=5.0)

    def _load_tasks(self):
        """
        Load TASKS from ROS 2 parameters using prefix parsing.
        taken from https://gist.github.com/agrueneberg/d76fff493753fa531f1b5d33be0f07ed
        """

        params_task = self.get_parameters_by_prefix('TASKS')    # e.g., {"task_3.timespan":[1,2],"task_3.rel_position":[1,2], "task_2.rel_position"...}
        params_period = self.get_parameters_by_prefix('PERIODS')# e.g., {"period_1":[1,2],"period_2":[1,2], ...}

        if not params_task:
            self.get_logger().warn("No TASKS parameters found")
            return []

        tasks_by_name = {}

        for full_key, param in params_task.items():
            # Example: "task_3.timespan"
            task_name, field = full_key.split(".", 1)

            tasks_by_name.setdefault(task_name, {})[field] = param.value

            self.get_logger().info(
                f"Loaded param: {task_name}.{field} = {param.value}"
            )
        
        periods = []
        
        for full_key, param in params_period.items():
            # Example: "period_1" -> period are assumed to be given in order
            _, num = full_key.split("_", 1)
            num = int(num)
            periods.append(param.value)

            self.get_logger().info(
                f"Loaded period: {num} = {param.value}"
            )

        tasks = []
        for name, data in tasks_by_name.items():
            # Replace period indices with actual period values

            try:
                tasks.append(Task(
                    timespan    = periods[data['period_num']],
                    edges       = data['edges'],
                    rel_position= data['rel_position'],
                    size        = data['size'],
                    period_num  = data['period_num']
                ))
            except Exception as e:
                raise e

        # check phase : make sure periods are ordered and not overlapping
        for num, period in enumerate(periods):
            for other_num, other_period in enumerate(periods):
                if num < other_num:
                    if not (period[1] <= other_period[0]):
                        self.get_logger().error(
                            f"Periods {num} and {other_num} are overlapping or are unordred. PLease make sure the period are orderd in ascending order: {period} and {other_period}. Please provide "
                        )
        self.get_logger().info('\033[32m'+ f" All tasks loaded successfully !" + '\033[0m')
        return tasks, periods

    def wait_for_time(self):
        while self.get_clock().now().nanoseconds == 0:
            self.get_logger().info("Waiting for valid ROS time...")
            rclpy.spin_once(self, timeout_sec=0.1)



def signal_handler(sig, frame):
    print("Shutdown signal received, cleaning up...")
    rclpy.shutdown()
    sys.exit(0)


def main(args=None):
    rclpy.init(args=args)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    manager = Manager()

    rclpy.spin(manager)
    manager.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()