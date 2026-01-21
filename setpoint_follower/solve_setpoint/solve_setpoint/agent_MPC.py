import sys
import rclpy
import signal
import numpy as np
import tf_transformations

from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from crazyflie_interfaces.msg import Position, FullState
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Int32

#from barrier_msg.msg import Config
from barrier_msg.msg import BMsg, BMsglist
from solve_setpoint.bMsg import bMsg, HyperCubeHandler
from incorporate_barrier.MPC_barrier import MPCsolver
from crazyflie_interfaces.srv import Takeoff
from solve_setpoint.utils import WorkingMode, AgentState, ManagerState

LAND_TIME = 200
HEIGHT    = 1.0

class MPC_agents(Node):
    def __init__(self):
        super().__init__("drones")

        self.declare_parameter('robot_prefix', '/crazyflie')
        self.declare_parameter("SYSTEM", 2)
        self.declare_parameter("DIM", 10)
        self.declare_parameter("NUM_AGENTS", 10)
        self.declare_parameter("SPEED", 0.4)
        self.declare_parameter("AGENT_TIMER", 0.1)
        self.declare_parameter("HOOVERING_HEIGHT",1.0)

        self.robot_prefix = self.get_parameter('robot_prefix').value
        self.SYSTEM = self.get_parameter("SYSTEM").value
        self.DIM = self.get_parameter("DIM").value
        self.SPEED = self.get_parameter("SPEED").value
        self.NUM_AGENTS = self.get_parameter("NUM_AGENTS").value
        self.AGENT_TIMER = self.get_parameter("AGENT_TIMER").value
        self.HOOVERING_HEIGHT = self.get_parameter("HOOVERING_HEIGHT").value

        self.Z_SPEED = 0.5

        self.solver = MPCsolver()

        self.barrier_callback = self.create_subscription(BMsglist, "/barriers", self.on_barrier_message, 10)
        self.state_publisher  = self.create_publisher(Int32, "/agent_state", 1)

        if self.SYSTEM == WorkingMode.SIM:
            self.drones = ['{}{}'.format(self.robot_prefix, i) for i in range(1,self.NUM_AGENTS+1)]
        elif self.SYSTEM == WorkingMode.REAL:
            self.drones = []

            for srv_name, srv_types in self.get_service_names_and_types():
                if 'crazyflie_interfaces/srv/StartTrajectory' in srv_types:
                    # remove '/' and '/start_trajectory'
                    cfname = srv_name[1:-17]
                    if cfname != 'all':
                        self.drones.append(cfname)

        odom_name = {WorkingMode.SIM: "/odom" , WorkingMode.REAL: "/pose"}
        odom_type = {WorkingMode.SIM: Odometry, WorkingMode.REAL: PoseStamped}

        cmd_name = {WorkingMode.SIM: "/cmd_vel", WorkingMode.REAL: "/cmd_position"}
        cmd_type = {WorkingMode.SIM: Twist     , WorkingMode.REAL: Position}

        self.odom_subscribers = []
        self.twist_publishers = []
        self.takeoff_services = []
        
        for i, drone in enumerate(self.drones):
            callback = lambda msg, idx=i: self.odom_callback(msg, idx)
            self.odom_subscribers.append(self.create_subscription(
                odom_type[self.SYSTEM], drone + odom_name[self.SYSTEM], callback, 10))
            self.twist_publishers.append(self.create_publisher(cmd_type[self.SYSTEM], drone + cmd_name[self.SYSTEM], 10))

            if self.SYSTEM == WorkingMode.REAL:
                takeoffService = self.create_client(Takeoff, drone + '/takeoff')
                while not takeoffService.wait_for_service(timeout_sec=1.0):
                    self.get_logger().info('takeoff service not available, waiting again... Make sure the crazyswarm is launched')
                self.takeoff_services.append(takeoffService)
            
                
        self.pos = np.zeros((self.NUM_AGENTS, 3))
        self.angles = np.zeros((self.NUM_AGENTS, 3))

        self.set_recv = False
        self.timer = self.create_timer(self.AGENT_TIMER, self.MPC_callback)

        self.state = AgentState.TAKEOFF
        self.get_logger().info(f'Agent state : {self.state.name}')
        
        self.wait_for_time() # wait until the /clock message is correctly initialized by ros
        self.past_time = 0.
        self.start_time = self.get_clock().now()

        self.barriers = []

        self.k = 2
        self.active = False
        self.is_set = False
        self.called_takeoff = False
        self.time_to_take_off = 2

        self.start_set = False
        self.start_pos = np.zeros((3, self.NUM_AGENTS))
        
        self.saved_pos = self.start_pos.copy()
        self.get_logger().info('Finish startup')

    def odom_callback(self, msg, idx=0):
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


    def on_barrier_message(self, msg):
        self.get_logger().info(f"Received Incoming barrier function messages. Starting MPC mode")

        ## the state must be ready before it is possible to start 
        if self.state !=  AgentState.READY :
            self.get_logger().info(f"Not ready to start the mission. Message discarded...")

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

        self.active = False
        for bmsg in self.barriers:
            candidates = [bmsg2 for bmsg2 in self.barriers if bmsg2.edge_j == bmsg.edge_i]
            if candidates:
                bmsg.add_neighbour(candidates[0])
        
        self.start_time = self.get_clock().now()

        mpc_horizon  = 20
        initial_time = 0.
        agents_id    = [i for i in range(self.NUM_AGENTS)]

        self.solver.initialize_problem(mpc_horizon, initial_time, self.barriers, agents_id , dt=0.1, MAX_INPUT=0.3)
        self.active = True

        

    def MPC_callback(self):
        
        self.time      = self.get_clock().now() - self.start_time
        time_sec       = self.time.nanoseconds / 1e9
        dt_sec         = 0.1
        self.state_publisher.publish(Int32(data=int(self.state)))

        if time_sec > LAND_TIME:
            self.state = AgentState.LANDING
            self.get_logger().info(f'Forced Landing after time {LAND_TIME}. Strating Landing .... Current time {time_sec}. Initial time {self.start_time}')
            
            if not self.is_set:
                self.land_pos = self.pos.copy()
                self.saved_pos = self.pos.copy()
            self.is_set = True

        if self.state == AgentState.TAKEOFF:
            if self.SYSTEM == WorkingMode.SIM:
                for idx, publisher in enumerate(self.twist_publishers):
                    msg = Twist()
                    msg.linear.z = np.clip((self.HOOVERING_HEIGHT - self.pos[idx, 2]), -self.Z_SPEED, self.Z_SPEED) # go to one meter altitude
                    publisher.publish(msg)
            if self.SYSTEM == WorkingMode.REAL:
                if not self.called_takeoff:
                    for idx, publisher in enumerate(self.twist_publishers):
                        self.takeoff(self.HOOVERING_HEIGHT, self.time_to_take_off, idx) # TODO: have a closer look at the height (ensure collsion avoidance)

                self.called_takeoff = True

            if self.time.nanoseconds / 1e9 > self.time_to_take_off*2:
                self.state = AgentState.READY
                self.get_logger().info(f'Takeoff finished. Switching to READY.')
                self.start_time = self.get_clock().now()
        

        elif self.state == AgentState.READY:
            self.get_logger().info('Running MPC controller')
            if self.DIM == 2:
                x0 = self.pos[:,:2].flatten()
            elif self.DIM == 3:
                x0 = self.pos.flatten()

            if self.active:
                if self.SYSTEM == WorkingMode.REAL :
                    x_new = self.solver.recompute(20, x0, time_sec, self.barriers, dt=dt_sec, return_val="x_next")
                elif self.SYSTEM == WorkingMode.SIM:
                    u     = self.solver.recompute(20, x0, time_sec, self.barriers, dt=dt_sec, return_val="u0")
            else :
                return

            if self.SYSTEM == WorkingMode.SIM:
                for idx, publisher in enumerate(self.twist_publishers):
                    msg = Twist()
                    msg.linear.x = u[self.DIM*idx+0]
                    msg.linear.y = u[self.DIM*idx+1]
                    msg.linear.z = np.clip(self.HOOVERING_HEIGHT - self.pos[idx, 2], -self.Z_SPEED, self.Z_SPEED)
                    msg.angular.z = np.clip(0. - self.angles[idx, 2], -self.SPEED, self.SPEED)
                    publisher.publish(msg)
            if self.SYSTEM == WorkingMode.REAL:
                for idx, publisher in enumerate(self.twist_publishers):
                    pos = Position()
                    pos.x = x_new[self.DIM*idx+0]#self.start_pos[idx][0]
                    pos.y = x_new[self.DIM*idx+1]#self.start_pos[idx][1]
                    pos.z = min(1.+0.2, 1.)
                    pos.yaw = 0.
                    publisher.publish(pos)

        elif self.state == AgentState.LANDING:
            if self.SYSTEM == WorkingMode.SIM:
                for idx, publisher in enumerate(self.twist_publishers):
                    msg = Twist()
                    msg.linear.z = np.clip(0. - self.pos[idx, 2], -self.Z_SPEED, self.Z_SPEED)
                    publisher.publish(msg)
            if self.SYSTEM == WorkingMode.REAL:
                for idx, publisher in enumerate(self.twist_publishers):
                    pos = Position()
                    pos.x = self.saved_pos[idx][0]
                    pos.y = self.saved_pos[idx][1]
                    pos.z = min(self.HOOVERING_HEIGHT-self.HOOVERING_HEIGHT*(time_sec-LAND_TIME)/2, 1.)
                    pos.yaw = 0.
                    publisher.publish(pos)

        else:
            raise ValueError("mode supposed to be startup or MPC")
        
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
        req = Takeoff.Request()
        req.group_mask = groupMask
        req.height = targetHeight
        req.duration = rclpy.duration.Duration(seconds=duration).to_msg()
        self.takeoff_services[idx].call_async(req)

    def setpoint_callback(self, msg):
        msg = PoseStamped()
        msg.pose.linear.x = 0
        msg.pose.linear.y = 0
        msg.pose.linear.z = 0
        msg.pose.angular.z = 0
    
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

    agents = MPC_agents()

    executor = MultiThreadedExecutor()
    executor.add_node(agents)

    executor.spin()

    agents.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()