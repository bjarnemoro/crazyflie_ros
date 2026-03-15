import sys
import rclpy
import signal
import numpy as np
import tf_transformations

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from motion_capture_tracking_interfaces.msg import NamedPoseArray
from geometry_msgs.msg import Twist
from crazyflie_interfaces.msg import Position, FullState
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

from std_msgs.msg import Int32, Bool

from incorporate_barrier.bMsg import bMsg, HyperCubeHandler
from barrier_msg.msg import BMsg, BMsglist
from incorporate_barrier.MPC_barrier import STLMPC
from crazyflie_interfaces.srv import Takeoff
from solve_setpoint.utils import WorkingMode, AgentState, ManagerState, AnsiColor

class MPC_agents(Node):
    def __init__(self):
        super().__init__("drones")

        self.declare_parameter("backend", "")
        self.declare_parameter("AGENTS_INDICES",[1])
        self.declare_parameter("DIM", 10)
        self.declare_parameter("NUM_AGENTS", 10)
        self.declare_parameter("SPEED", 0.4)
        self.declare_parameter("AGENT_TIMER", 0.1)
        self.declare_parameter("HOOVERING_HEIGHT",1.0)
        self.declare_parameter("COMM_DISTANCE", 1.0)


        backend  = self.get_parameter("backend").value
        if backend == "sim":
            self.SYSTEM = WorkingMode.SIM
        elif backend == "hardware":
            self.SYSTEM = WorkingMode.REAL

        self.DIM              = self.get_parameter("DIM").value
        self.SPEED            = self.get_parameter("SPEED").value
        self.NUM_AGENTS       = self.get_parameter("NUM_AGENTS").value
        self.AGENT_TIMER      = self.get_parameter("AGENT_TIMER").value
        self.HOOVERING_HEIGHT = self.get_parameter("HOOVERING_HEIGHT").value
        self.COMM_DISTANCE    = self.get_parameter("COMM_DISTANCE").value
        self.AGENTS_INDICES   = self.get_parameter("AGENTS_INDICES").value
        self.NUM_AGENTS       = len(self.AGENTS_INDICES)
        self.dt = 0.1

        self.landing_time = 10.0 # takes 8 seconds to land
        self.Z_SPEED      = 0.5

        self.mpc_solver = STLMPC(num_agents = self.NUM_AGENTS,
                                agents_dim = self.DIM,
                                dt         = self.dt,
                                horizon    = 10,
                                max_input  = self.SPEED,
                                communication_distance = self.COMM_DISTANCE*0.90)

        self.fast_cb_group = ReentrantCallbackGroup()
        self.mpc_cb_group = MutuallyExclusiveCallbackGroup()

        self.barrier_callback     = self.create_subscription(BMsglist, "/barriers", self.on_barrier_message, 10,  callback_group=self.mpc_cb_group)
        self.state_publisher      = self.create_publisher(Int32, "/agent_state", 10, callback_group=self.fast_cb_group)
        self.landing_command_sub  = self.create_subscription(Bool, "/landing_command", self.landing_command_callback, 10, callback_group=self.fast_cb_group)
        self.gather_command_sub   = self.create_subscription(Bool, "/gather_command", self.on_gather_callback, 10, callback_group=self.fast_cb_group)
        self.stop_command_sub     = self.create_subscription(Bool, "/stop_mpc", self.on_stop, 10, callback_group=self.fast_cb_group)


        self.drones_names = ['crazyflie{}'.format(i) for i in self.AGENTS_INDICES]
        self.name_to_index = {name: i for i, name in enumerate(self.drones_names)}

        odom_name = {WorkingMode.SIM: "/odom" , WorkingMode.REAL: "/pose"}
        odom_type = {WorkingMode.SIM: Odometry, WorkingMode.REAL: PoseStamped}

        cmd_name = {WorkingMode.SIM: "/cmd_vel", WorkingMode.REAL: "/cmd_position"}
        cmd_type = {WorkingMode.SIM: Twist     , WorkingMode.REAL: Position}

        self.odom_subscribers = []
        self.twist_publishers = []
        self.takeoff_services = []
        if self.SYSTEM == WorkingMode.REAL:
            self.hoover_heights   = [self.HOOVERING_HEIGHT  for idx in range(self.NUM_AGENTS)]
        else:
            self.hoover_heights   = [self.HOOVERING_HEIGHT + 0.1*idx for idx in range(self.NUM_AGENTS)]
        
        
        if self.SYSTEM == WorkingMode.SIM:
            for i, drone in enumerate(self.drones_names):
                callback = lambda msg, idx=i: self.odom_callback(msg, idx)
                self.odom_subscribers.append(self.create_subscription(
                    odom_type[self.SYSTEM], drone + odom_name[self.SYSTEM], callback, 10))
                
                self.twist_publishers.append(
                    self.create_publisher(cmd_type[self.SYSTEM],drone + cmd_name[self.SYSTEM],10)
                    )

        elif self.SYSTEM == WorkingMode.REAL:


            qos = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10)
            self.pose_sub = self.create_subscription(NamedPoseArray,"/poses", self.poses_callback,qos   )

            for drone in self.drones_names:

                self.twist_publishers.append(
                    self.create_publisher(cmd_type[self.SYSTEM], drone + cmd_name[self.SYSTEM], 10)
                )

                takeoffService = self.create_client(Takeoff, drone + '/takeoff')

                while not takeoffService.wait_for_service(timeout_sec=1.0):
                    self.get_logger().warn(
                        'takeoff service not available, waiting again... Make sure the crazyswarm is launched'
                    )
                self.takeoff_services.append(takeoffService)

        else :
            raise Exception("Invalid working mode. Please choose either 'sim' or 'hardware' as backend parameter.")
            
                
        self.pos = np.zeros((self.NUM_AGENTS, 3))
        self.angles = np.zeros((self.NUM_AGENTS, 3))

        self.set_recv = False
        self.timer = self.create_timer(self.AGENT_TIMER, self.MPC_callback,callback_group=self.mpc_cb_group)

        self.state = AgentState.TAKEOFF
        self.get_logger().info(f'Agent state : {self.state.name}')
        
        self.wait_for_time() # wait until the /clock message is correctly initialized by ros
        self.past_time = 0.
        
        self.start_time = self.get_clock().now()
        self.start_takeoff_time = None
        self.start_landing_time = None
        self.start_mission_time = None

        self.barriers = []

        self.k = 2
        self.active = False
        self.called_takeoff = False
        self.time_to_take_off = 3.

        self.start_set = False
        self.land_pos  = None
        self.get_logger().info('Finish startup')
        self.stop_mpc  = False
        self.gathering_pos = np.array([0.,0.,0.])

    def on_stop(self, msg):
        if msg.data == True:
            if self.state != AgentState.STOP:
                self.get_logger().info(f'{AnsiColor.RED} Received STOP command... {AnsiColor.RESET}')
                self.state = AgentState.STOP
                self.timer.cancel()
        elif msg.data == False:
            if self.state == AgentState.STOP:
                self.get_logger().info(f'{AnsiColor.GREEN} Resuming from STOP command... {AnsiColor.RESET}')
                self.state = AgentState.READY
                self.timer = self.create_timer(self.AGENT_TIMER, self.MPC_callback)

    def landing_command_callback(self, msg):
        if msg.data == True: 
            if self.state != AgentState.LANDING:
                self.get_logger().info(f'{AnsiColor.BLUE} Received landing command. Switching to LANDING state... {AnsiColor.RESET}')
                self.state = AgentState.LANDING

    def on_gather_callback(self, msg):
        if msg.data == True:
            if self.state != AgentState.GATHERING:
                self.get_logger().info(f'{AnsiColor.BLUE} Received gathering command. Switching to GATHERING state... {AnsiColor.RESET}')
                self.state = AgentState.GATHERING
        elif msg.data == False:
            if self.state == AgentState.GATHERING:
                self.get_logger().info(f'{AnsiColor.BLUE} Stopping gathering. Switching to READY state... {AnsiColor.RESET}')
                self.state = AgentState.READY

    def odom_callback(self, msg, idx):
        if type(msg) == Odometry:
            self.pos[idx, 0] = msg.pose.pose.position.x
            self.pos[idx, 1] = msg.pose.pose.position.y
            self.pos[idx, 2] = msg.pose.pose.position.z

            q = msg.pose.pose.orientation
        elif type(msg) == PoseStamped:
            self.pos[idx, 0] = msg.pose.position.x
            self.pos[idx, 1] = msg.pose.position.y
            self.pos[idx, 2] = msg.pose.position.z

            q = msg.pose.orientation
        
        euler = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.angles[idx, 0] = euler[0]
        self.angles[idx, 1] = euler[1]
        self.angles[idx, 2] = euler[2]


    def poses_callback(self, msg):
        for p in msg.poses:

            idx      = self.name_to_index.get(p.name)
            if idx is None:
                continue
            position = p.pose.position

            self.pos[idx, 0] = position.x
            self.pos[idx, 1] = position.y
            self.pos[idx, 2] = position.z


    def on_barrier_message(self, msg):
        self.get_logger().info(f"{AnsiColor.BLUE}Received new incoming barrier function messages. Starting MPC mode{AnsiColor.RESET}")

        ## the state must be ready before it is possible to start 
        if self.state !=  AgentState.READY :
            self.get_logger().info(f"{AnsiColor.BLUE}Not ready to start the mission. Message discarded...{AnsiColor.RESET}")

        self.barriers = []
        for bmsg in msg.messages:
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
            
            self.barriers.append(my_msg)
        
        self.mpc_solver.initialize_problem(self.barriers,self.get_logger())
        
        if self.start_mission_time is None: # at the first barrier message start the time
            self.start_mission_time = self.get_clock().now()

    
    def initiate_takeoff(self):
        if self.start_takeoff_time is None:
            self.start_takeoff_time = self.get_clock().now()
        
        self.time         = self.get_clock().now() - self.start_takeoff_time
        take_off_finished = False

        if self.SYSTEM == WorkingMode.SIM:
            for idx, publisher in enumerate(self.twist_publishers):
                msg = Twist()
                msg.linear.z = np.clip((self.hoover_heights[idx] - self.pos[idx, 2]), -self.Z_SPEED, self.Z_SPEED) # go to one meter altitude
                publisher.publish(msg)
        if self.SYSTEM == WorkingMode.REAL:
            if not self.called_takeoff:
                self.called_takeoff = True
                for idx, publisher in enumerate(self.twist_publishers):
                    self.takeoff(self.hoover_heights[idx], self.time_to_take_off, idx) # TODO: have a closer look at the height (ensure collsion avoidance)

        self.get_logger().info(f'{AnsiColor.BLUE} Taking off... Time elapsed: {self.time.nanoseconds / 1e9:.2f}s. Will finish at {self.time_to_take_off*2.0}s {AnsiColor.RESET}',throttle_duration_sec=2.0)

        if self.time.nanoseconds / 1e9 > self.time_to_take_off*2.0:
            self.get_logger().info(f' {AnsiColor.BLUE} Take off finished... {AnsiColor.RESET}')
            take_off_finished = True
        
        return take_off_finished

    def initiate_landing(self):

        if self.start_landing_time is None:
            self.start_landing_time = self.get_clock().now()

        self.time         = self.get_clock().now() - self.start_landing_time
        landing_finished  = False
        time_sec = self.time.nanoseconds / 1e9

        if self.SYSTEM == WorkingMode.SIM:
            for idx, publisher in enumerate(self.twist_publishers):
                msg = Twist()
                msg.linear.z = np.clip(-self.hoover_heights[idx]/self.landing_time, -self.Z_SPEED, self.Z_SPEED)
                publisher.publish(msg)

        if self.SYSTEM == WorkingMode.REAL:
            for idx, publisher in enumerate(self.twist_publishers):
                pos   = Position()
                pos.x = self.pos[idx, 0]
                pos.y = self.pos[idx, 1]
                pos.z = self.hoover_heights[idx]-self.hoover_heights[idx]/self.landing_time *(time_sec)
                pos.yaw = 0.
                publisher.publish(pos)
        
        if time_sec > self.landing_time:
            self.get_logger().info(f'{AnsiColor.BLUE} Landing finished... {AnsiColor.RESET}', throttle_duration_sec=5.0)
            landing_finished = True

        return landing_finished

    
    def run_mission(self) :

        if not self.mpc_solver.initialized :
            self.get_logger().info(f"MPC solver not initialized yet. Waiting for barrier messages...",throttle_duration_sec=3.0)
            return
        
        self.time = self.get_clock().now() - self.start_mission_time
        time_sec = self.time.nanoseconds / 1e9
        self.get_logger().info(f"{AnsiColor.BLUE} Running MPC mission... Time elapsed: {time_sec:.2f}s {AnsiColor.RESET}",throttle_duration_sec=5.0)

        if self.DIM == 2:
            x0 = self.pos[:,:2].flatten()
        elif self.DIM == 3:
            x0 = self.pos.flatten()

        if self.SYSTEM == WorkingMode.REAL :
            x_new = self.mpc_solver(x0, time_sec, return_val="x_next", logger=self.get_logger())
            x_new = self.apply_collision_barrier(x_new, k = 0.08, d = 0.35)

        elif self.SYSTEM == WorkingMode.SIM:

            u = self.mpc_solver(x0, time_sec, return_val="u0", logger=self.get_logger())

            # Predict next position
            pos_pred = self.pos[:, :self.DIM] + self.dt * u.reshape(self.NUM_AGENTS, self.DIM)

            # Apply collision avoidance
            pos_corrected = self.apply_collision_barrier(pos_pred.flatten(), k = 0.01, d = 0.35)
            pos_corrected = pos_corrected.reshape(self.NUM_AGENTS, self.DIM)

            # Convert corrected position back to velocity
            u = (pos_corrected - self.pos[:, :self.DIM]) / self.dt

            # Flatten to keep original indexing
            u = u.flatten()


        if self.SYSTEM == WorkingMode.SIM:

            for idx, publisher in enumerate(self.twist_publishers):

                msg = Twist()
                agent_index = self.DIM * idx

                msg.linear.x = float(u[agent_index + 0])
                msg.linear.y = float(u[agent_index + 1])

                msg.linear.z = float(np.clip(
                    self.hoover_heights[idx] - self.pos[idx, 2],
                    -self.Z_SPEED,
                    self.Z_SPEED
                ))

                msg.angular.z = float(np.clip(
                    0. - self.angles[idx, 2],
                    -self.SPEED,
                    self.SPEED
                ))

                publisher.publish(msg)


        if self.SYSTEM == WorkingMode.REAL:

            for idx, publisher in enumerate(self.twist_publishers):

                pos = Position()
                agent_index = self.DIM * idx

                pos.x = float(x_new[agent_index + 0])
                pos.y = float(x_new[agent_index + 1])
                pos.z = float(self.hoover_heights[idx])
                pos.yaw = 0.0

                publisher.publish(pos)
    
    def apply_collision_barrier(self, x_new, k, d):

        pos = x_new.reshape(self.NUM_AGENTS, self.DIM)

        d_safe = d
        k_rep  = k

        correction = np.zeros_like(pos)

        for i in range(self.NUM_AGENTS):
            for j in range(i+1, self.NUM_AGENTS):

                diff = pos[i] - pos[j]
                dist = np.linalg.norm(diff)

                if dist < d_safe and dist > 1e-6:

                    direction = diff / dist
                    strength = k_rep * (1/dist - 1/d_safe)

                    corr = strength * direction

                    correction[i] += corr
                    correction[j] -= corr

        # cut the correction to avoid too large values
        max_correction = 0.2
        correction = np.clip(correction, -max_correction, max_correction)
        
        pos = pos + correction

        return pos.flatten()

    def gathering(self):

        if self.start_mission_time is None:
            self.start_mission_time = self.get_clock().now()
        self.time = self.get_clock().now() - self.start_mission_time
        time_sec = self.time.nanoseconds / 1e9

        self.get_logger().info(f"{AnsiColor.BLUE} Gathering initiated {time_sec:.2f}s {AnsiColor.RESET}",throttle_duration_sec=5.0)
        
        if self.SYSTEM == WorkingMode.SIM:
            k_p = 0.05
            for idx, publisher in enumerate(self.twist_publishers):
                msg = Twist()
                agent_index  = self.DIM*idx
                msg.linear.x = (self.gathering_pos[0] - self.pos[idx,0]) * k_p
                msg.linear.y = (self.gathering_pos[1] - self.pos[idx,1]) * k_p
                msg.linear.z = np.clip(self.hoover_heights[idx] - self.pos[idx, 2], -self.Z_SPEED, self.Z_SPEED)
                msg.angular.z = np.clip(0. - self.angles[idx, 2], -self.SPEED, self.SPEED)
                publisher.publish(msg)
        
        if self.SYSTEM == WorkingMode.REAL:
            v  = 0.05
            for idx, publisher in enumerate(self.twist_publishers):
                
                pos = Position()
                direction = np.array([self.gathering_pos[0] - self.pos[idx,0], self.gathering_pos[1] - self.pos[idx,1]])/ np.linalg.norm(np.array([self.gathering_pos[0] - self.pos[idx,0], self.gathering_pos[1] - self.pos[idx,1]]))
                agent_index  = self.DIM*idx
                pos.x = self.pos[idx,0] + direction[0]*v * self.dt
                pos.y = self.pos[idx,1] + direction[1]*v * self.dt
                pos.z = self.hoover_heights[idx]
                pos.yaw = 0.
                publisher.publish(pos)

        
    def MPC_callback(self):

        self.time      = self.get_clock().now() - self.start_time
        self.state_publisher.publish(Int32(data=int(self.state)))

        self.get_logger().info(f"{AnsiColor.BLUE}MPC STATE: {self.state.name} {AnsiColor.RESET}", throttle_duration_sec=3.0)

        if self.land_pos is None:
            self.land_pos = self.pos.copy()

        if self.state == AgentState.TAKEOFF:
            is_finished = self.initiate_takeoff()
            if is_finished:
                self.state = AgentState.READY
                self.get_logger().info(f'{AnsiColor.BLUE} Switching to READY state. Starting MPC mission...{AnsiColor.RESET}')
                self.start_time = self.get_clock().now()

        elif self.state == AgentState.LANDING:
            is_finished = self.initiate_landing()
            if is_finished:
                self.get_logger().info(f'{AnsiColor.BLUE} Landing completed. Shutting down node...{AnsiColor.RESET}', throttle_duration_sec=5.0)
           
        elif self.state == AgentState.READY:
            self.run_mission()
            pass
        
        elif self.state == AgentState.GATHERING:
            self.gathering()
            pass
            
        
    def update_full_state(self, twist, idx):
        self.full_state = FullState()

        self.full_state.pose.position.x = self.pos[idx, 0]
        self.full_state.pose.position.y = self.pos[idx, 1]
        self.full_state.pose.position.z = self.pos[idx, 2]
        self.full_state.twist.linear.x = twist.linear.x
        self.full_state.twist.linear.y = twist.linear.y
        self.full_state.twist.linear.z = twist.linear.z
        self.full_state.pose.orientation.x = 0.
        self.full_state.pose.orientation.y = 0.
        self.full_state.pose.orientation.z = 0.
        self.full_state.pose.orientation.w = 1.
        self.full_state.twist.angular.x = 0.
        self.full_state.twist.angular.y = 0.
        self.full_state.twist.angular.z = twist.angular.z
        
        self.twist_publishers[idx].publish(self.full_state)

    def takeoff(self, targetHeight, duration, idx, groupMask=0):
        req            = Takeoff.Request()
        req.group_mask = groupMask
        req.height     = targetHeight
        req.duration   = rclpy.duration.Duration(seconds=duration).to_msg()
        # Wait until service call completes
        self.takeoff_services[idx].call_async(req)


    
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

    executor = MultiThreadedExecutor(num_threads=4)
    agents = MPC_agents()

    executor.add_node(agents)

    try:
        executor.spin()
    finally:
        agents.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()