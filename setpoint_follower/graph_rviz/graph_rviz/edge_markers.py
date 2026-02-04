import sys
import rclpy
import signal
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

from std_msgs.msg import Int32MultiArray
from barrier_msg.msg import TMsglist
from solve_setpoint.utils import WorkingMode, AgentState, ManagerState, AnsiColor

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
    marker.ns = "agents"
    marker.id = marker_id

    marker.action = Marker.ADD

    marker.pose.position.x = pos[0]
    marker.pose.position.y = pos[1]
    marker.pose.position.z = pos[2]
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    marker.scale.x = 0.3
    marker.scale.y = 0.3
    marker.scale.z = 0.3

    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = 0.2


    return marker

def delete_sphere(time, marker_id):
    marker = Marker()
    marker.header.frame_id = "world"
    marker.header.stamp = time
    marker.ns = "agents"
    marker.id = marker_id
    marker.action = Marker.DELETE

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

        self.declare_parameter("backend", "sim")
        self.declare_parameter("AGENTS_INDICES", [1])

        backend            = self.get_parameter("backend").value
        if backend == "sim":
            self.SYSTEM = WorkingMode.SIM
        elif backend == "hardware":
            self.SYSTEM = WorkingMode.REAL


        self.AGENTS_INDICES = self.get_parameter("AGENTS_INDICES").value
        self.NUM_AGENTS     = len(self.AGENTS_INDICES)

        self.publisher      = self.create_publisher(MarkerArray, 'visualization_marker', 10)
        self.task_publisher = self.create_publisher(MarkerArray, 'task_visualization_marker', 10)
        self.timer          = self.create_timer(0.1, self.timer_callback)

    
        self.drones_names = ['{}{}'.format("/crazyflie", i) for i in  self.AGENTS_INDICES]
        odom_name   = {WorkingMode.SIM: "/odom", WorkingMode.REAL: "/pose"}
        odom_type   = {WorkingMode.SIM: Odometry, WorkingMode.REAL: PoseStamped}

        self.odom_subscribers = []
        for jj,drone in enumerate(self.drones_names):
            callback = lambda msg, idx=jj:self.pos_callback(msg, idx)
            self.odom_subscribers.append(self.create_subscription(
                odom_type[self.SYSTEM], drone + odom_name[self.SYSTEM], callback, 10))

        self.drone_pos = np.zeros((self.NUM_AGENTS, 3))
        self.edges = []
        self.main_agents = []
        self.task_msgs = []
        self.prev_marker_count = 0
        self.prev_edge_count = 0
        self.prev_edges = []

        self.create_subscription(Int32MultiArray, "/graph_edges", self.set_edges, 10)
        self.create_subscription(TMsglist, "/task_edges", self.set_task, 10)
        self.create_subscription(Int32MultiArray, "/active_agents",self.set_main_agents, 10)
        
        self.edges_set  = set()
        self.agents_set = set()



    def timer_callback(self):

        ################################################
        # Draw communicaiton graph
        ###############################################
        
        marker_array = MarkerArray()
        for edge in self.edges:
            
            marker_id_edge = int(str(edge[0]) + str(edge[1])) 
            time   = self.get_clock().now().to_msg()
            pos1   = self.drone_pos[edge[0]-1] # -1 for 0-based indexing
            pos2   = self.drone_pos[edge[1]-1] # -1 for 0-based indexing
            color  = [1.0, 0.0, 0.0]
            marker = line_strip(time, marker_id_edge, pos1, pos2, color)
            marker_array.markers.append(marker)
            self.edges_set.add(edge)

        for edge in self.edges_set:
            if edge not in self.edges:
                marker_id_edge = int(str(edge[0]) + str(edge[1])) 
                time = self.get_clock().now().to_msg()
                marker = delete_strip(time, marker_id_edge)
                marker_array.markers.append(marker)
  
        self.publisher.publish(marker_array)

        # # TODO: add the task marker logic
        
        ################################################
        # Draw sphere around active agents and add agents numbers
        ###############################################
        ## add marker TEXT for each agent 
        marker_array = MarkerArray()
        for i in range(self.NUM_AGENTS):
            time = self.get_clock().now().to_msg()
            pos = self.drone_pos[i] + np.array([0.0, 0.0, 0.2])
            color = [0.0, 0.0, 1.0]
            marker = text_marker(time, i, pos, self.AGENTS_INDICES[i], color)
            marker_array.markers.append(marker)


        for agent in self.main_agents:
            
            marker_id_agent = agent
            
            time   = self.get_clock().now().to_msg()
            pos    = self.drone_pos[agent-1] # -1 for 0-based indexing
            color  = [0.0, 1.0, 0.0]
            marker = sphere(time, marker_id_agent, pos, color)
            marker_array.markers.append(marker)
            self.agents_set.add(agent)

        for agent in self.agents_set:
            if agent not in self.main_agents:
                marker_id_agent = agent
                time = self.get_clock().now().to_msg()
                marker = delete_sphere(time, marker_id_agent)
                marker_array.markers.append(marker)
  
        self.task_publisher.publish(marker_array)

    
    def pos_callback(self, msg, idx):
        """
        set the position of an agent by index. Index starts from 1 to N_agents so 
        we need to subtract 1 to access the array
        """
        if type(msg) == Odometry:
            self.drone_pos[idx][0] = msg.pose.pose.position.x
            self.drone_pos[idx][1] = msg.pose.pose.position.y
            self.drone_pos[idx][2] = msg.pose.pose.position.z

        elif type(msg) == PoseStamped:
            self.drone_pos[idx][0] = msg.pose.position.x
            self.drone_pos[idx][1] = msg.pose.position.y
            self.drone_pos[idx][2] = msg.pose.position.z

    def set_edges(self, msg):
        data = msg.data
        self.edges = {tuple(sorted((data[i], data[i+1]))) for i in range(0, len(data), 2)}

    def set_task(self, msg):
        self.task_msgs = msg.messages

    def set_main_agents(self, msg):
        self.main_agents = {i for i in msg.data}

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