import sys
import rclpy
import signal
import numpy as np
import tf_transformations

from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from crazyflie_interfaces.msg import FullState
from rclpy.executors import MultiThreadedExecutor

#from barrier_msg.msg import Config
from barrier_msg.msg import BMsg, BMsglist
from solve_setpoint.bMsg import bMsg, HyperCubeHandler
from incorporate_barrier.MPC_barrier import MPCsolver

class Mode:
    SIM = 0
    REAL = 1

class MPC_agents(Node):
    def __init__(self):
        super().__init__("drones")

        self.declare_parameter('robot_prefix', '/crazyflie')
        self.declare_parameter("SYSTEM", 2)
        self.declare_parameter("DIM", 10)
        self.declare_parameter("NUM_AGENTS", 10)
        self.declare_parameter("SPEED", 0.4)
        self.declare_parameter("AGENT_TIMER", 0.1)

        self.robot_prefix = self.get_parameter('robot_prefix').value
        self.SYSTEM = self.get_parameter("SYSTEM").value
        self.DIM = self.get_parameter("DIM").value
        self.SPEED = self.get_parameter("SPEED").value
        self.NUM_AGENTS = self.get_parameter("NUM_AGENTS").value
        self.AGENT_TIMER = self.get_parameter("AGENT_TIMER").value

        self.Y_SPEED = 0.5

        self.solver = MPCsolver()

        self.barrier_callback = self.create_subscription(BMsglist, "/barriers", self.set_boundary_msgs, 10)
        #self.create_subscription(Odometry, self.robot_prefix + '/odom', self.odom_callback, 10)
        #self.create_subscription(Odometry, self.robot_prefix + '/set', self.setpoint_callback, 10)

        if self.SYSTEM == Mode.SIM:
            self.drones = ['{}{}'.format(self.robot_prefix, i) for i in range(1,self.NUM_AGENTS+1)]
        elif self.SYSTEM == Mode.REAL:
            self.drones = [self.robot_prefix + '{0:0=2d}'.format(i) for i in range(1,self.NUM_AGENTS+1)]

        odom_name = {Mode.SIM: "/odom", Mode.REAL: "/pose"}
        odom_type = {Mode.SIM: Odometry, Mode.REAL: PoseStamped}

        cmd_name = {Mode.SIM: "/cmd_vel", Mode.REAL: "/cmd_full_state"}
        cmd_type = {Mode.SIM: Twist, Mode.REAL: FullState}

        self.odom_subscribers = []
        self.twist_publishers = []
        for i, drone in enumerate(self.drones):
            callback = lambda msg, idx=i: self.odom_callback(msg, idx)
            self.odom_subscribers.append(self.create_subscription(
                odom_type[self.SYSTEM], drone + odom_name[self.SYSTEM], callback, 10))
            self.twist_publishers.append(self.create_publisher(cmd_type[self.SYSTEM], drone + cmd_name[self.SYSTEM], 10))
            
                
        self.pos = np.zeros((self.NUM_AGENTS, 3))
        self.angles = np.zeros((self.NUM_AGENTS, 3))

        self.set_recv = False
        self.timer = self.create_timer(self.AGENT_TIMER, self.MPC_callback)

        self.mode = "startup"
        self.past_time = 0
        self.start_time = self.get_clock().now().nanoseconds

        self.barriers = []

        self.k = 2
        self.active = False
        self.is_set = False

        self.start_pos = np.array([[0.0, 0.0, 0.0],
                                   [0.0, -0.4, 0.0],
                                   [0.4, -0.4, 0.0]])
        
        self.saved_pos = self.start_pos.copy()

        self.heights = [0.4, 0.5, 0.6]

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

    def set_boundary_msgs(self, msg):
        self.get_logger().info(f"received barrier msgs")

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
        
        self.start_time = self.get_clock().now().nanoseconds
        self.mode = "MPC"

        
        self.solver.initialize_problem(20, 0., self.barriers, [i for i in range(self.NUM_AGENTS)], dt=0.1, MAX_INPUT=0.3)
        self.active = True

        

    def MPC_callback(self):
        self.time = self.get_clock().now().nanoseconds - self.start_time
        self.dt = self.time - self.past_time
        self.past_time = self.time
        time_sec = self.time / 1e9
        dt_sec = self.dt / 1e9

        dt_sec = 0.1

        self.get_logger().info(f"{time_sec}")

        #if time_sec > 5:
        #    self.mode = "MPC"
        if time_sec > 12:
            self.mode = "land"
            
            if not self.is_set:
                self.land_pos = self.pos.copy()
                self.saved_pos = self.pos.copy()
            self.is_set = True

        if self.mode == "startup":
            if self.SYSTEM == Mode.SIM:
                for idx, publisher in enumerate(self.twist_publishers):
                    msg = Twist()
                    msg.linear.z = np.clip(1. - self.pos[idx, 2], -self.Y_SPEED, self.Y_SPEED)
                    publisher.publish(msg)
            if self.SYSTEM == Mode.REAL:
                for idx in range(len(self.twist_publishers)):
                    twist = Twist()
                    h = 0.5
                    twist.linear.z = np.clip(self.k * (self.heights[idx] - self.pos[idx, 2]), -self.Y_SPEED, self.Y_SPEED)
                    twist.linear.x = np.clip(self.k * (self.start_pos[idx, 0] - self.pos[idx, 0]), -self.SPEED, self.SPEED)
                    twist.linear.y = np.clip(self.k * (self.start_pos[idx, 1] - self.pos[idx, 1]), -self.SPEED, self.SPEED)
                    twist.angular.z = np.clip(0. - self.angles[idx, 2], -self.SPEED, self.SPEED)
                    self.update_full_state(twist, idx)

        elif self.mode == "MPC":
            if self.DIM == 2:
                x0 = self.pos[:,:2].flatten()
            elif self.DIM == 3:
                x0 = self.pos.flatten()
            return_val = "input"

            if self.active:
                u = self.solver.recompute(20, x0, time_sec+3, self.barriers, dt=dt_sec, return_val=return_val)
            else:
                u = np.zeros((self.NUM_AGENTS*self.DIM,))

            if self.SYSTEM == Mode.SIM:
                for idx, publisher in enumerate(self.twist_publishers):
                    msg = Twist()
                    msg.linear.x = u[self.DIM*idx+0]
                    msg.linear.y = u[self.DIM*idx+1]
                    msg.linear.z = np.clip(1. - self.pos[idx, 2], -self.Y_SPEED, self.Y_SPEED)

                    msg.angular.z = np.clip(0. - self.angles[idx, 2], -self.SPEED, self.SPEED)
                    publisher.publish(msg)
            if self.SYSTEM == Mode.REAL:
                for idx in range(len(self.twist_publishers)):
                    twist = Twist()
                    h = 0.5

                    #if np.linalg.norm(u[self.DIM*idx:self.DIM*idx+self.DIM]) > 0.05:
                    twist.linear.z = np.clip(self.k * (self.heights[idx] - self.pos[idx, 2]), -self.Y_SPEED, self.Y_SPEED)
                    twist.linear.x = u[self.DIM*idx+0]
                    twist.linear.y = u[self.DIM*idx+1]
                    twist.angular.z = np.clip(0. - self.angles[idx, 2], -self.SPEED, self.SPEED)
                    self.update_full_state(twist, idx)
                    self.saved_pos = self.pos.copy()
                    # else:
                    #     self.get_logger().info(f"not moving! {idx}")
                    #     twist.linear.z = np.clip(self.k * (h - self.pos[idx, 2]), -self.SPEED, self.SPEED)
                    #     twist.linear.x = np.clip(self.k * (self.saved_pos[idx, 0] - self.pos[idx, 0]), -self.SPEED, self.SPEED)
                    #     twist.linear.y = np.clip(self.k * (self.saved_pos[idx, 1] - self.pos[idx, 1]), -self.SPEED, self.SPEED)
                    #     twist.angular.z = np.clip(0. - self.angles[idx, 2], -self.SPEED, self.SPEED)
                    #     self.update_full_state(twist, idx)

            

        elif self.mode == "land":
            if self.SYSTEM == Mode.SIM:
                for idx, publisher in enumerate(self.twist_publishers):
                    msg = Twist()
                    msg.linear.z = np.clip(0. - self.pos[idx, 2], -self.Y_SPEED, self.Y_SPEED)
                    publisher.publish(msg)
            if self.SYSTEM == Mode.REAL:
                for idx in range(len(self.twist_publishers)):
                    twist = Twist()
                    h = 0.05
                    twist.linear.z = np.clip(self.k * (h - self.pos[idx, 2]), -self.Y_SPEED, self.Y_SPEED)
                    twist.linear.x = np.clip(self.k * (self.land_pos[idx, 0] - self.pos[idx, 0]), -self.SPEED, self.SPEED)
                    twist.linear.y = np.clip(self.k * (self.land_pos[idx, 1] - self.pos[idx, 1]), -self.SPEED, self.SPEED)
                    twist.angular.z = np.clip(0. - self.angles[idx, 2], -self.SPEED, self.SPEED)
                    self.update_full_state(twist, idx)

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

    def setpoint_callback(self, msg):
        msg = PoseStamped()
        msg.pose.linear.x = 0
        msg.pose.linear.y = 0
        msg.pose.linear.z = 0
        msg.pose.angular.z = 0


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