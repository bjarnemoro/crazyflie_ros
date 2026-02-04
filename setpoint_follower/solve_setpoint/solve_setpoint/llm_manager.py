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

from solve_setpoint.graph_manager import GraphManager
from solve_setpoint.task_manager import TaskManager, Task
from solve_setpoint.solvers.combined_solve import solve_task_decomposition
from solve_setpoint.utils import WorkingMode, AgentState, ManagerState, AnsiColor

import json
from ollama import chat
from std_msgs.msg import String

SYSTEM_PROMPT = """ You translate natural language commands into robot formation task specifications among agents.

CRITICAL OUTPUT RULES:
- Output ONLY valid JSON.
- Do NOT include markdown, code fences, comments, or explanations.
- The output must start with '{' and end with '}'.

Schema:
{
  "tasks": [
    {
      "rel_position": [float, float],
      "size": float,
      "period_num": int,
      "edges": [int, int],
      "operator": string,
      "timespan": [int, int]
    }
  ]
}


GENERAL RULES:
- Use ONLY agent indices explicitly mentioned in the user input.
- "edges" contains exactly two integers [i,j] to define a relative formation from i to j.
  - For single-agent tasks, repeat the same index twice.
- "rel_position" must be a list of two floats representing the relative formation from i to j. The norm should not exceed 3 and should not be below 0.3.
- "size" default is 0.1 meters if not specified.
- "operator" can be only "always" or "eventually" (default: "always").
- "timespan" is [start, end] in seconds, using integer approximations.
- Each distinct timespan corresponds to a unique period_num.
- period_num starts at 0 and increases sequentially.
- Timespans must be non-overlapping and sequential.

LOGICAL CONNECTIVES:
- AND → tasks share the same timespan and period_num.
- OR → choose one option and discard the other.
- "A then B" → two subsequent periods with different timespans.

TIME HANDLING:
- If timespans are not specified, create timespans of 10 seconds each.
- Ensure consecutive timespans are separated by at least 20 seconds.

FORMATION RULES:
- When asked for a rigid formation among multiple agents (e.g., triangle, square),
  create multiple pairwise tasks whose combined rel_positions realize the formation.
  Make sure the formation can be closed (i.e., the relative formations sum to 0 and the sequence of edges form a path of agents of the form [[i,j],[j,k] .. [l,i]]).
- Do NOT name the formation in the output.
- Do NOT invent geometry outside the allowed rel_position list.
"""


class llmManager(Node):
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
        # self.declare_parameter("MANAGER_TIMER", 0.1)
        # self.declare_parameter("SETPOINT_TIMER", 0.1)
        # self.declare_parameter("COMM_DISTANCE", 1.1)
        # self.declare_parameter("BOX_WEIGHT", 10)
        
        backend            = self.get_parameter("backend").value
        if backend == "sim":
            self.SYSTEM = WorkingMode.SIM
        elif backend == "hardware":
            self.SYSTEM = WorkingMode.REAL


        self.AGENTS_INDICES = self.get_parameter("AGENTS_INDICES").value
        self.DIM            = self.get_parameter("DIM").value
        self.MANAGER_TIMER  = self.get_parameter("MANAGER_TIMER").value
        self.COMM_DISTANCE  = self.get_parameter("COMM_DISTANCE").value
        self.BOX_WEIGHT     = self.get_parameter("BOX_WEIGHT").value
        self.NUM_AGENTS     = len(self.AGENTS_INDICES)

        self.get_logger().info("agents indices: {}".format(self.AGENTS_INDICES))

        
        self.tasks   = []
        self.periods = {}

        ##
        self.start_time = None
        self.ready_count = 0
        self.current_tasks = []
        self.barrier_request_id = 0

        self.triggered_times = set()

        #setup all the subscribers and publishers

        self.drones_names = ['{}{}'.format("/crazyflie", i) for i in  self.AGENTS_INDICES]

        self.graph_manager = GraphManager(self.NUM_AGENTS, self.COMM_DISTANCE)
        
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

        for i,drone in enumerate(self.drones_names, start=1):
            callback = lambda msg, idx=i: self.graph_manager.set_pos_callback(msg, idx, self.get_logger().info)
            self.odom_subscribers.append(self.create_subscription(
                odom_type[self.SYSTEM], drone + odom_name[self.SYSTEM], callback, 10))

        #start the main loops of the system with a timer method
        self.timer = self.create_timer(self.MANAGER_TIMER, self.mainloop)


        # callback to llm
        self.sub = self.create_subscription(
            String,
            '/operator_command',
            self.on_operator_command_callback,
            10
        )

        ####################################################
            
        #set the starting state
        self.manager_state = ManagerState.WAITING_FOR_ODOMETRY
        self.agent_state   = None
        modes      = {0: "simulation", 1: "real"}
        self.get_logger().info(f"Working in {self.SYSTEM.name} mode. Manager state {self.manager_state.name} !!")

    def on_new_barrier(self, future,req_id):

    
        if req_id != self.barrier_request_id:
            self.get_logger().warn(
                "Ignoring stale barrier response"
            )
            return


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
                self.get_logger().info(f"{AnsiColor.BOLD_GREEN} Recaculating task at recalculation time : {current_time}" \
                                    f". Remaining recalculation times: {self.recalc_times} {AnsiColor.RESET}")
                self.recalculate_tasks(current_time)
        
        if self.agent_state == AgentState.GATHERING:
            self.get_logger().info(f"{AnsiColor.BOLD_GREEN} Attempting to re-calculate tasks while gathering {AnsiColor.RESET}")
            self.recalculate_tasks(current_time)

        if communication_graph_changed:
            is_critical = False
            for task in self.current_tasks:
                u,v = task.edges
                if (u,v) not in self.graph_manager.get_edge() and (v,u) not in self.graph_manager.get_edge():
                   is_critical = True
                   self.get_logger().info(f"{AnsiColor.BOLD_GREEN} Recaculating task  after communication graph changed, affecting current tasks {AnsiColor.RESET}")
                   self.recalculate_tasks(current_time)
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

        self.barrier_request_id += 1
        req_id = self.barrier_request_id
        future = self.barrier_client.call_async(req)
        future.add_done_callback(
            lambda f, rid=req_id: self.on_new_barrier(f, rid))


    def recalculate_tasks(self, current_time):
        #TODO: In case task is not feasible retrun the minimum violating solution
        #calculate the current relative tasks based on the current weighted communication graph
        
        comm_graph = self.graph_manager.get_comm_graph()
        comm_edges = self.graph_manager.get_edge()
        self.get_logger().debug(f"{AnsiColor.BOLD_GREEN} Current communication edges: {comm_edges} {AnsiColor.RESET}")

        active_self_tasks, active_edge_tasks, possible_task_paths, decomposition_needed, is_disconnected = self.task_manager.get_active_tasks_and_decomposition_paths(current_time, comm_edges, self.get_logger())

        if not decomposition_needed:
            self.get_logger().info(f"{AnsiColor.BOLD_GREEN} No decomposition needed! requesting barriers {AnsiColor.RESET}")
            self.current_tasks = active_self_tasks + active_edge_tasks
            self.request_barrier(active_self_tasks + active_edge_tasks)
            return 

        if is_disconnected:
            #call for gathering 
            self.get_logger().info(f"{AnsiColor.BOLD_RED} Communication graph is disconnected, calling gathering command. {AnsiColor.RESET}")
            self.gather_command_pub.publish(Bool(data=True))
            return
        
        num_attempts = len(possible_task_paths)
        for jj,task_paths in enumerate(possible_task_paths,start=1) :

            cvx_status, new_edge_tasks = solve_task_decomposition(active_edge_tasks, task_paths,self.BOX_WEIGHT, self.COMM_DISTANCE, self.get_logger())
            if cvx_status == "optimal" : 
                #compute the barrier function
                self.get_logger().info(f"{AnsiColor.BOLD_GREEN} All tasks can be decomposed successfully!{AnsiColor.RESET}")
                self.current_tasks = active_self_tasks + new_edge_tasks
                self.request_barrier(self.current_tasks)
                
                if self.agent_state == AgentState.GATHERING :
                    self.gather_command_pub.publish(Bool(data=False))

                # print tasks
                for task in self.current_tasks:
                    self.get_logger().info(f"{AnsiColor.BOLD_GREEN} Assigned Task : Agents {task.edges} , rel_position {task.rel_position} , size {task.size} , timespan {task.timespan} {AnsiColor.RESET}")
                
                return # exit loop 
                
            else:
                self.get_logger().info(f"{AnsiColor.BOLD_YELLOW} Decomposition attempt {jj}/{num_attempts}: paths {task_paths} failed with status {cvx_status} . Trying another decomposition... {AnsiColor.RESET}")

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
                
                if len(self.tasks) == 0:
                    self.manager_state = ManagerState.WAITING_FOR_TASKS
                else:
                    self.manager_state = ManagerState.READY

                #publish the initial communication graph
                self.update_and_publish_communication_graph()
            
            else:
                missing_agents =  self.graph_manager.offline_agents()
                missing_agents_id = [self.AGENTS_INDICES[i] for i in missing_agents]
                self.get_logger().info(f"Waiting for odom messages from agents: {missing_agents_id}. Missing agents: {missing_agents}",throttle_duration_sec=5.0)

        elif self.manager_state == ManagerState.WAITING_FOR_TASKS:
            self.get_logger().info(f"{AnsiColor.BOLD_GREEN} MPC currently in mode {self.agent_state.name}. {AnsiColor.RESET}",throttle_duration_sec=5.0)
            if len(self.tasks) > 0:
                self.get_logger().info(f"{AnsiColor.BOLD_GREEN} Received tasks from operator, changing manager state to: {ManagerState.READY.name} {AnsiColor.RESET}")
                self.manager_state = ManagerState.READY
            else:
                self.get_logger().info(f"{AnsiColor.BOLD_YELLOW} No tasks to perform yet. Waiting for operator command. {AnsiColor.RESET}",throttle_duration_sec=5.0)
        
        
        elif self.manager_state == ManagerState.READY and (self.agent_state == AgentState.READY or self.agent_state == AgentState.GATHERING):
    
            self.get_logger().info(f"{AnsiColor.BOLD_GREEN} Working on task execution. MPC currently mode {self.agent_state.name}. {AnsiColor.RESET}",throttle_duration_sec=5.0)

            if self.start_time is None: # time is considered since the start of the mission
                self.start_time = self.get_clock().now()
            
            current_time = self.get_clock().now() - self.start_time
            current_time = current_time.nanoseconds / 1e9
            
            comm_graph_changed = self.update_and_publish_communication_graph()
            self.update_and_publish_task_graph(current_time,comm_graph_changed)

        elif self.agent_state == AgentState.STOP :
            self.get_logger().info(f"{AnsiColor.BOLD_GREEN} MPC externally stopped . {AnsiColor.RESET}",throttle_duration_sec=5.0)

        elif self.agent_state == AgentState.LANDING :
            self.get_logger().info(f"{AnsiColor.BOLD_GREEN} MPC landing mode . {AnsiColor.RESET}",throttle_duration_sec=5.0)

    
    
    def on_operator_command_callback(self, msg: String):
        
        self.get_logger().info(f"Received command from human operator: {msg.data}")

        self.tasks = []
        self.periods = set()

        try:
            raw_text = self.query_ollama(msg.data).strip()
            self.get_logger().info(f"Raw LLM output: {repr(raw_text)}")

            clean_text = strip_code_fences(raw_text)
            data = json.loads(clean_text)

            if "tasks" not in data:
                raise ValueError("No 'tasks' field in LLM output")

        except Exception as e:
            self.get_logger().error(f"LLM processing failed: {e}")
            return

        self.get_logger().info(f"Parsed LLM output: {raw_text}")

        for t in data["tasks"]:
            try:
                t['edges'] = (
                    self.AGENTS_INDICES.index(t['edges'][0]) + 1,
                    self.AGENTS_INDICES.index(t['edges'][1]) + 1
                )
            except ValueError as e:
                self.get_logger().error(
                    f"Invalid agent index in task edges {t['edges']}"
                )
                return

            self.tasks.append(Task(
                timespan=[int(x) for x in t["timespan"]],
                edges=[int(x) for x in t["edges"]],
                rel_position=[float(x) for x in t["rel_position"]],
                size=float(t["size"]),
                period_num=int(t["period_num"]),
                operator=t["operator"]
            ))

            self.periods.add((t["timespan"][0], t["timespan"][1]))
        
        self.periods = sorted(list(self.periods), key=lambda x: x[0])

        self.get_logger().info(f"periods : {self.periods}")

        # print tasks 
        for task in self.tasks:
            self.get_logger().info(f"{AnsiColor.BOLD_GREEN} Loaded Task : Agents {task.edges} , rel_position {task.rel_position} , size {task.size} , timespan {task.timespan} {AnsiColor.RESET}")

        # Setup task manager
        self.task_manager = TaskManager(
            self.DIM,
            self.COMM_DISTANCE,
            self.periods,
            self.tasks
        )
        self.recalc_times = self.task_manager.recalculate_at()

        self.get_logger().info("Tasks successfully updated from Ollama")

    def query_ollama(self, user_text: str) -> str:
        response = chat(
            model='gemma3:27b',
            messages=[
                {'role': 'system', 'content': SYSTEM_PROMPT},
                {'role': 'user', 'content': user_text},
            ],
            options={
                "temperature": 0.0,
                "num_predict": 1000
            }
        )
        return response['message']['content']

def signal_handler(sig, frame):
    print("Shutdown signal received, cleaning up...")
    rclpy.shutdown()
    sys.exit(0)

def strip_code_fences(text: str) -> str:
    text = text.strip()
    if text.startswith("```"):
        lines = text.splitlines()
        # remove first and last fence
        return "\n".join(lines[1:-1]).strip()
    return text


def main(args=None):
    rclpy.init(args=args)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    manager = llmManager()

    rclpy.spin(manager)
    manager.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()