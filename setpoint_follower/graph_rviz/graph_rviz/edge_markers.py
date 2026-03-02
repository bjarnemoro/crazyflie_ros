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

    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2

    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = 0.1


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


def arrow_marker(time, marker_id, agent_i_pos, i_to_j_formation, color):
    marker = Marker()
    marker.header.frame_id = "world"
    marker.header.stamp = time
    marker.ns = "task_arrows"
    marker.id = marker_id
    marker.type = Marker.ARROW
    marker.action = Marker.ADD

    marker.scale.x = 0.03   # shaft diameter
    marker.scale.y = 0.06   # head diameter
    marker.scale.z = 0.1    # head length

    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = 1.0

    p1 = Point(x=agent_i_pos[0], y=agent_i_pos[1], z=agent_i_pos[2])
    p2 = Point(x=agent_i_pos[0] + i_to_j_formation[0], 
               y=agent_i_pos[1] + i_to_j_formation[1], 
               z=agent_i_pos[2] + i_to_j_formation[2])

    marker.points.append(p1)
    marker.points.append(p2)

    return marker


def cube_marker(time, marker_id, center, size, color):
    marker = Marker()
    marker.header.frame_id = "world"
    marker.header.stamp = time
    marker.ns = "task_boxes"
    marker.id = marker_id
    marker.type = Marker.CUBE
    marker.action = Marker.ADD

    marker.pose.position.x = center[0]
    marker.pose.position.y = center[1]
    marker.pose.position.z = center[2]

    marker.pose.orientation.w = 1.0

    marker.scale.x = size[0]*2
    marker.scale.y = size[1]*2
    marker.scale.z = size[2]*2

    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = 0.2  # semi-transparent

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


        ################################################
        # Draw task formations (arrow + box)
        ################################################

        task_marker_array = MarkerArray()
        current_task_ids = set()

        for k, task in enumerate(self.task_msgs):

            i = task.edge_i
            j = task.edge_j

            # agent positions
            pos_i = self.drone_pos[i-1]
            pos_j = self.drone_pos[j-1]
            relative_pos = np.append(np.array(task.center),0.0) # add z=0 for 3D visualization

            time = self.get_clock().now().to_msg()

            # unique ids
            arrow_id = 1000 + k
            box_id   = 2000 + k

            # Arrow i -> j
            arrow = arrow_marker(
                time,
                arrow_id,
                pos_i,
                relative_pos,
                color=[1.0, 1.0, 0.0]  # yellow
            )

            # Box
            cube = cube_marker(
                time,
                box_id,
                pos_i + relative_pos,
                task.size,
                color=[0.0, 1.0, 1.0]  # cyan
            )

            task_marker_array.markers.append(arrow)
            task_marker_array.markers.append(cube)

            current_task_ids.add(arrow_id)
            current_task_ids.add(box_id)


        # Delete old markers
        if hasattr(self, "prev_task_ids"):
            for old_id in self.prev_task_ids:
                if old_id not in current_task_ids:
                    delete_marker = Marker()
                    delete_marker.header.frame_id = "world"
                    delete_marker.header.stamp = self.get_clock().now().to_msg()
                    delete_marker.id = old_id

                    if old_id < 2000:
                        delete_marker.ns = "task_arrows"
                    else:
                        delete_marker.ns = "task_boxes"

                    delete_marker.action = Marker.DELETE
                    task_marker_array.markers.append(delete_marker)

        self.prev_task_ids = current_task_ids

        self.publisher.publish(task_marker_array)

    
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