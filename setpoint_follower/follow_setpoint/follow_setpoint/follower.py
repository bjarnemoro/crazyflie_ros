import rclpy
import tf_transformations
import numpy as np

from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from copy import deepcopy

Z_SPEED = 0.8
XY_SPEED = 0.3

class Follower(Node):
    def __init__(self):
        super().__init__('setpoint_follower')
        #self.declare_parameter('robot_prefix', '/crazyflie1')
        #robot_prefix = self.get_parameter('robot_prefix').value

        self.get_logger().info("\n\nStarting!!\n\n")

        self.drones = ['/crazyflie{}'.format(i) for i in range(1,11)]

        self.positions = [[0, 0, 0] for i in range(10)]
        self.angles = [[0, 0, 0] for i in range(10)]
        self.setpoints = [[np.cos(i*1.8*np.pi/10), np.sin(i*1.8*np.pi/10), 1] for i in range(10)]

        self.odom_subscribers = []
        self.twist_publishers = []
        for i, drone in enumerate(self.drones):
            callback = deepcopy(lambda msg, idx=i: self.odom_subscribe_callback(msg, idx))
            self.odom_subscribers.append(self.create_subscription(
                Odometry, drone + '/odom', callback, 10))
            self.twist_publisher = self.create_publisher(Twist, drone + '/cmd_vel', 10)
            self.twist_publishers.append(self.twist_publisher)

        #self.get_logger().info("{}".format(self.odom_subscribers.callback))

        self.count = 0
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("\n\nStarting!!\n\n")
        

    def timer_callback(self):
        self.count += 0.04
        z_i = 0
        self.setpoints[0:5] = [[0.15+0.4*np.cos(i*2*np.pi/5+self.count), 0.4*np.sin(i*1.8*np.pi/5+self.count), 1+0.3*np.sin(i*4*np.pi/10+self.count)] for i in range(0, 5)]
        self.setpoints[5:10] = [[-0.15+0.4*np.cos(i*2*np.pi/5+self.count), 0.4*np.sin(i*1.8*np.pi/5+self.count), 1-0.3*np.sin(i*4*np.pi/10+self.count)] for i in range(5, 10)]

        for drone in self.drones:
            idx = self.drones.index(drone)

            msg = Twist()

            z_err = self.setpoints[idx][2] - self.positions[idx][2]
            msg.linear.z = np.clip(z_err, -Z_SPEED, Z_SPEED)

            msg.angular.z = -0.1*self.angles[idx][2]

            if z_err < 0.5:
                x_err = self.setpoints[idx][0] - self.positions[idx][0]
                y_err = self.setpoints[idx][1] - self.positions[idx][1]
                msg.linear.x = np.clip(x_err, -XY_SPEED, XY_SPEED)
                msg.linear.y = np.clip(y_err, -XY_SPEED, XY_SPEED)
            self.twist_publishers[idx].publish(msg)

            #if idx == 1:
            #    self.get_logger().info("{}, {}, {}".format(msg.linear.x, msg.linear.y, msg.linear.z))

    def odom_subscribe_callback(self, msg, idx):
        self.positions[idx][0] = msg.pose.pose.position.x
        self.positions[idx][1] = msg.pose.pose.position.y
        self.positions[idx][2] = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        euler = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.angles[idx][0] = euler[0]
        self.angles[idx][1] = euler[1]
        self.angles[idx][2] = euler[2]

        #self.get_logger().info("{}".format(idx))
    

def main(args=None):
    #print('Hi from follow_setpoint.')
    print("\n\nAttempting to follow setpoint!\n\n")

    rclpy.init(args=args)
    follower = Follower()
    rclpy.spin(follower)
    rclpy.destroy_node()
    #follower.destory_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
