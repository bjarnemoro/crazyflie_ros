import sys
import rclpy
import signal
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

from std_msgs.msg import Int32MultiArray
from msg_interface.msg import TaskEdge, TaskEdgeList
from barrier_msg.msg import TMsglist

def line_strip(time, marker_id, pos1, pos2, color):
    marker = Marker()
    marker.header.frame_id = "world"
    marker.header.stamp = time
    marker.ns = "edges"
    marker.id = marker_id
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = 0.02  # Line width

    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = 1.0

    # Replace with actual drone positions
    p1 = Point(x=pos1[0], y=pos1[1], z=pos1[2])
    p2 = Point(x=pos2[0], y=pos2[1], z=pos2[2])

    marker.points.append(p1)
    marker.points.append(p2)

    return marker

def sphere(time, marker_id, pos, color):
    marker = Marker()
    marker.header.frame_id = "world"
    marker.header.stamp = time
    marker.type = Marker.SPHERE
    marker.ns = "edges"
    marker.id = marker_id

    marker.action = Marker.ADD

    marker.pose.position.x = pos[0]
    marker.pose.position.y = pos[1]
    marker.pose.position.z = pos[2]
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    marker.scale.x = 0.15
    marker.scale.y = 0.15
    marker.scale.z = 0.15

    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = 1.0

    return marker

def delete_strip(time, marker_id):
    marker = Marker()
    marker.header.frame_id = "world"
    marker.header.stamp = time
    marker.ns = "edges"
    marker.id = marker_id
    marker.action = Marker.DELETE

    return marker

def text_marker(time, marker_id, pos, agent_id, color):
    marker = Marker()
    marker.header.frame_id = "world"
    marker.header.stamp = time
    marker.ns = "text"
    marker.id = marker_id
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD

    marker.pose.position.x = pos[0]
    marker.pose.position.y = pos[1]
    marker.pose.position.z = pos[2]
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    marker.scale.z = 0.2  # Text height

    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = 1.0

    marker.text = "CF{}".format(agent_id)

    return marker

class GraphMarkerPublisher(Node):
    def __init__(self):
        super().__init__('graph_marker_publisher')

        self.declare_parameter('robot_prefix', '/crazyflie')
        self.declare_parameter("SYSTEM", 2)
        self.declare_parameter("DIM", 10)
        self.declare_parameter("NUM_AGENTS", 10)
        self.declare_parameter("SPEED", 0.4)
        self.declare_parameter("AGENT_TIMER", 0.1)
        self.declare_parameter("HOOVERING_HEIGHT",1.0)
        self.declare_parameter("COMM_DISTANCE", 1.0)

        self.robot_prefix = self.get_parameter('robot_prefix').value
        self.SYSTEM = self.get_parameter("SYSTEM").value
        self.DIM = self.get_parameter("DIM").value
        self.SPEED = self.get_parameter("SPEED").value
        self.NUM_AGENTS = self.get_parameter("NUM_AGENTS").value
        self.AGENT_TIMER = self.get_parameter("AGENT_TIMER").value
        self.HOOVERING_HEIGHT = self.get_parameter("HOOVERING_HEIGHT").value
        self.COMM_DISTANCE = self.get_parameter("COMM_DISTANCE").value

        self.publisher = self.create_publisher(MarkerArray, 'visualization_marker', 10)
        self.task_publisher = self.create_publisher(MarkerArray, 'task_visualization_marker', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        

        self.drones = ['/crazyflie{}'.format(i) for i in range(1,self.NUM_AGENTS+1)]
        self.odom_subscribers = []
        for i, drone in enumerate(self.drones):
            callback = lambda msg, idx=i: self.pos_callback(msg, idx)
            self.odom_subscribers.append(self.create_subscription(
                Odometry, drone + '/odom', callback, 10))

        self.drone_pos = np.zeros((self.NUM_AGENTS, 3))
        self.edges = []
        self.task_msgs = []
        self.prev_marker_count = 0
        self.prev_edge_count = 0
        self.prev_edges = []

        self.create_subscription(Int32MultiArray, "/graph_edges", self.set_edges, 10)
        self.create_subscription(TMsglist, "/task_edges", self.set_task, 10)
        self.markers_id_dict = {}
        self.marker_id = 0

    def timer_callback(self):
        marker_array = MarkerArray()
        #add lines where needed
        for edge in self.edges:
            
            marker_id_edge = self.markers_id_dict.get(edge, None)
            if marker_id_edge is None:
                marker_id_edge = self.marker_id
                self.markers_id_dict[edge] = marker_id_edge
                self.marker_id += 1
            
            time   = self.get_clock().now().to_msg()
            pos1   = self.drone_pos[edge[0]-1] # -1 for 0-based indexing
            pos2   = self.drone_pos[edge[1]-1] # -1 for 0-based indexing
            color  = [1.0, 0.0, 0.0]
            marker = line_strip(time, marker_id_edge, pos1, pos2, color)
            marker_array.markers.append(marker)

        for edge in self.markers_id_dict:
            if edge not in self.edges:
                marker_id_edge = self.markers_id_dict[edge]
                time = self.get_clock().now().to_msg()
                marker = delete_strip(time, marker_id_edge)
                marker_array.markers.append(marker)
  
        self.publisher.publish(marker_array)

        # TODO: add the task marker logic

        ## add marker for each agent 
        for i in range(self.NUM_AGENTS):
            time = self.get_clock().now().to_msg()
            pos = self.drone_pos[i] + np.array([0.0, 0.0, 0.2])
            color = [0.0, 0.0, 1.0]
            marker = text_marker(time, i, pos, i+1, color)
            marker_array.markers.append(marker)

        self.task_publisher.publish(marker_array)
        self.prev_marker_count = len(self.task_msgs)

    def pos_callback(self, msg, idx):
        self.drone_pos[idx][0] = msg.pose.pose.position.x
        self.drone_pos[idx][1] = msg.pose.pose.position.y
        self.drone_pos[idx][2] = msg.pose.pose.position.z

    def set_edges(self, msg):
        data = msg.data
        self.edges = [tuple(sorted((data[i], data[i+1]))) for i in range(0, len(data), 2)]

    def set_task(self, msg):
        self.task_msgs = msg.messages

    def set_main_agents(self, msg):
        self.main_agents = msg

def signal_handler(sig, frame):
    print("Shutdown signal received, cleaning up...")
    rclpy.shutdown()
    sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    node = GraphMarkerPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()