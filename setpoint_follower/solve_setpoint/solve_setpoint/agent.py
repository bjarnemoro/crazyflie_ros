import rclpy
import numpy as np
import tf_transformations

from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

SPEED = 0.5

class Agent(Node):
    def __init__(self):
        super().__init__('agent')
        self.declare_parameter('robot_prefix', '/crazyflie')
        self.robot_prefix = self.get_parameter('robot_prefix').value

        self.twist_publisher = self.create_publisher(Twist, self.robot_prefix + '/cmd_vel', 10)
        self.create_subscription(Odometry, self.robot_prefix + '/odom', self.odom_callback, 10)
        self.create_subscription(Odometry, self.robot_prefix + '/set', self.setpoint_callback, 10)
                
        self.pos = [0., 0., 0.]
        self.angles = [0., 0., 0.]

        self.setpoint_pos = [0., 0., 0.]
        self.setpoint_angles = [0., 0., 0.]

        self.set_recv = False
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        time = self.get_clock().now().nanoseconds
        time_sec = time / 1e6

        if self.set_recv:
            msg = Twist()
            msg.linear.x = np.clip(self.setpoint_pos[0] - self.pos[0], -SPEED, SPEED)
            msg.linear.y = np.clip(self.setpoint_pos[1] - self.pos[1], -SPEED, SPEED)
            msg.linear.z = np.clip(self.setpoint_pos[2] - self.pos[2], -SPEED, SPEED)
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


def main(args=None):
    rclpy.init(args=args)

    agent = Agent()

    rclpy.spin(agent)

    rclpy.shutdown()

if __name__ == "__main__":
    main()