import sys
import rclpy
import signal
import numpy as np
import tf_transformations

from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor

from barrier_msg.msg import Config
from barrier_msg.msg import BMsg, BMsglist
from solve_setpoint.bMsg import bMsg, HyperCubeHandler
from incorporate_barrier.MPC_barrier import optimize_path_first



class MPC_agents(Node):
    def __init__(self):
        super().__init__("drones")
        self.declare_parameter('robot_prefix', '/crazyflie')
        self.robot_prefix = "crazyflie1"

        self.barrier_callback = self.create_subscription(BMsglist, "/barriers", self.set_boundary_msgs, 10)
        
        self.twist_publisher = self.create_publisher(Twist, self.robot_prefix + '/cmd_vel', 10)
        #self.create_subscription(Odometry, self.robot_prefix + '/odom', self.odom_callback, 10)
        #self.create_subscription(Odometry, self.robot_prefix + '/set', self.setpoint_callback, 10)


        self.drones = ['/crazyflie{}'.format(i) for i in range(1,Config.NUM_AGENTS+1)]
        self.odom_subscribers = []
        self.twist_publishers = []
        for i, drone in enumerate(self.drones):
            callback = lambda msg, idx=i: self.odom_callback(msg, idx)
            self.odom_subscribers.append(self.create_subscription(
                Odometry, drone + '/odom', callback, 10))
            self.twist_publishers.append(self.create_publisher(Twist, drone + '/cmd_vel', 10))
            

                
        self.pos = np.zeros((10, 3))
        self.angles = np.zeros((10, 3))

        self.set_recv = False
        self.timer = self.create_timer(Config.AGENT_TIMER, self.timer_callback)
        self.MPCtimer = self.create_timer(0.02, self.MPC_callback)

        self.mode = "startup"
        self.past_time = 0
        self.start_time = 0

        self.barriers = []

    def timer_callback(self):
        time = self.get_clock().now().nanoseconds
        time_sec = time / 1e6

        if self.set_recv:
            msg = Twist()
            msg.linear.x = np.clip(self.setpoint_pos[0] - self.pos[0], -Config.SPEED, Config.SPEED)
            msg.linear.y = np.clip(self.setpoint_pos[1] - self.pos[1], -Config.SPEED, Config.SPEED)
            msg.linear.z = np.clip(self.setpoint_pos[2] - self.pos[2], -Config.SPEED, Config.SPEED)
            self.twist_publisher.publish(msg)

    def odom_callback(self, msg, idx=0):
        self.pos[idx, 0] = msg.pose.pose.position.x
        self.pos[idx, 1] = msg.pose.pose.position.y
        self.pos[idx, 2] = msg.pose.pose.position.z
        
        q = msg.pose.pose.orientation
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

        for bmsg in self.barriers:
            candidates = [bmsg2 for bmsg2 in self.barriers if bmsg2.edge_j == bmsg.edge_i]
            if candidates:
                bmsg.add_neighbour(candidates[0])
        
        self.start_time = self.get_clock().now().nanoseconds
        self.mode = "MPC"

        #x0 = self.pos[:,:2].flatten()
        #agents = [i for i in range(10)]
        #pos = optimize_path_first(65, x0, 0, self.barriers, agents, dt=0.1, return_val="pos")
        #self.get_logger().info(f"{pos}")
        #raise ValueError

    def MPC_callback(self):
        self.time = self.get_clock().now().nanoseconds - self.start_time
        self.dt = self.time - self.past_time
        self.past_time = self.time
        time_sec = self.time / 1e9
        dt_sec = self.dt / 1e9

        dt_sec = 0.02

        if self.mode == "startup":
            for idx, publisher in enumerate(self.twist_publishers):
                msg = Twist()
                msg.linear.z = np.clip(1. - self.pos[idx, 2], -Config.SPEED, Config.SPEED)
                publisher.publish(msg)

        elif self.mode == "MPC":
            if Config.DIM == 2:
                x0 = self.pos[:,:2].flatten()
            agents = [i for i in range(10)]
            return_val = "input"
            u = optimize_path_first(40, x0, time_sec, self.barriers, agents, dt=dt_sec, return_val=return_val)

            self.get_logger().info(f"{time_sec}")
            for idx, publisher in enumerate(self.twist_publishers):
                msg = Twist()
                msg.linear.x = u[Config.DIM*idx+0]
                msg.linear.y = u[Config.DIM*idx+1]
                msg.linear.z = np.clip(1. - self.pos[idx, 2], -Config.SPEED, Config.SPEED)
                publisher.publish(msg)
        else:
            raise ValueError("mode supposed to be startup or MPC")

    def setpoint_callback(self, msg):
        pass
        


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