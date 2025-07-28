import sys
import rclpy
import signal
import numpy as np
import tf_transformations

from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor

from solve_setpoint.config import Config

class Agent(Node):
    def __init__(self, drone):
        super().__init__(drone[1:])
        self.declare_parameter('robot_prefix', '/crazyflie')
        self.robot_prefix = drone

        self.twist_publisher = self.create_publisher(Twist, self.robot_prefix + '/cmd_vel', 10)
        self.create_subscription(Odometry, self.robot_prefix + '/odom', self.odom_callback, 10)
        self.create_subscription(Odometry, self.robot_prefix + '/set', self.setpoint_callback, 10)
                
        self.pos = [0., 0., 0.]
        self.angles = [0., 0., 0.]

        self.setpoint_pos = [0., 0., 0.]
        self.setpoint_angles = [0., 0., 0.]

        self.set_recv = False
        self.timer = self.create_timer(Config.AGENT_TIMER, self.timer_callback)

    def timer_callback(self):
        time = self.get_clock().now().nanoseconds
        time_sec = time / 1e6

        if self.set_recv:
            msg = Twist()
            msg.linear.x = np.clip(self.setpoint_pos[0] - self.pos[0], -Config.SPEED, Config.SPEED)
            msg.linear.y = np.clip(self.setpoint_pos[1] - self.pos[1], -Config.SPEED, Config.SPEED)
            msg.linear.z = np.clip(self.setpoint_pos[2] - self.pos[2], -Config.SPEED, Config.SPEED)
            self.twist_publisher.publish(msg)

    def odom_callback(self, msg):
        self.pos[0] = msg.pose.pose.position.x
        self.pos[1] = msg.pose.pose.position.y
        self.pos[2] = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        euler = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.angles[0] = euler[0]
        self.angles[1] = euler[1]
        self.angles[2] = euler[2]

    def setpoint_callback(self, msg):
        self.set_recv = True
        self.setpoint_pos[0] = msg.pose.pose.position.x
        self.setpoint_pos[1] = msg.pose.pose.position.y
        self.setpoint_pos[2] = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        euler = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.setpoint_angles[0] = euler[0]
        self.setpoint_angles[1] = euler[1]
        self.setpoint_angles[2] = euler[2]


def signal_handler(sig, frame):
    print("Shutdown signal received, cleaning up...")
    rclpy.shutdown()
    sys.exit(0)

def main(args=None):
    rclpy.init(args=args)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    drones = ['/crazyflie{}'.format(i) for i in range(1,Config.NUM_DRONES+1)]
    agents = [Agent(drone) for drone in drones]

    executor = MultiThreadedExecutor()
    for agent in agents:
        executor.add_node(agent)

    executor.spin()

    for agent in agents:
        agent.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()