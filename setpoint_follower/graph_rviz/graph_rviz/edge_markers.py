import rclpy
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

from std_msgs.msg import Int32MultiArray

class GraphMarkerPublisher(Node):
    def __init__(self):
        super().__init__('graph_marker_publisher')
        self.publisher = self.create_publisher(MarkerArray, 'visualization_marker', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.drones = ['/crazyflie{}'.format(i) for i in range(1,11)]
        self.odom_subscribers = []
        for i, drone in enumerate(self.drones):
            callback = lambda msg, idx=i: self.pos_callback(msg, idx)
            self.odom_subscribers.append(self.create_subscription(
                Odometry, drone + '/odom', callback, 10))

        self.drone_pos = np.zeros((10, 3))
        self.edges = []

        self.create_subscription(Int32MultiArray, "/graph_edges", self.set_edges, 10)

    def timer_callback(self):
        marker_array = MarkerArray()
        marker_id = 0

        for edge in self.edges:
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "edges"
            marker.id = marker_id
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.02  # Line width

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            # Replace with actual drone positions

            p1 = Point(x=self.drone_pos[edge[0]][0], y=self.drone_pos[edge[0]][1], z=self.drone_pos[edge[0]][2])
            p2 = Point(x=self.drone_pos[edge[1]][0], y=self.drone_pos[edge[1]][1], z=self.drone_pos[edge[1]][2])

            marker.points.append(p1)
            marker.points.append(p2)

            marker_array.markers.append(marker)
            marker_id += 1

        self.publisher.publish(marker_array)

    def pos_callback(self, msg, idx):
        self.drone_pos[idx][0] = msg.pose.pose.position.x
        self.drone_pos[idx][1] = msg.pose.pose.position.y
        self.drone_pos[idx][2] = msg.pose.pose.position.z

    def set_edges(self, msg):
        data = msg.data
        self.edges = [(data[i], data[i+1]) for i in range(0, len(data), 2)]

def main(args=None):
    rclpy.init(args=args)
    node = GraphMarkerPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()