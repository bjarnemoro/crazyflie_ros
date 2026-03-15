import numpy as np
from copy import deepcopy
from collections import defaultdict

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import rclpy.logging

class GraphManager:
    """"
    The graph manager takes care of storing the current position for all the agents by index.
    What it does it is

    1) compute the edges of the current communication graph based on the positions and communication distance
    2) provide a position callback to update the state of each agent by messages from the simulator/real world crazyflie

    
    """

    def __init__(self, num_agents, comm_distance):
        self.NUM_AGENTS = num_agents
        self.COMM_DISTANCE = comm_distance

        self.__agent_pos                                = np.zeros((self.NUM_AGENTS, 3))
        self.__comm_edges         :list[tuple(int,int)] = None
        self.__comm_graph         :dict[int,int]        = None
        self.__current_comm_edges :list[tuple(int,int)] = None

        self.online_status = np.zeros(self.NUM_AGENTS)

    def compute_current_communication_edges(self,logger):
        edges = []
        for i in range(1,self.NUM_AGENTS +1):
            for j in range(i+1, self.NUM_AGENTS +1):
                if np.linalg.norm(self.__agent_pos[i-1]-self.__agent_pos[j-1]) < self.COMM_DISTANCE:
                    edges.append((i,j))
        self.__current_comm_edges = edges
        if self.__comm_edges is None:
            self.__comm_edges = self.__current_comm_edges
    
    def did_graph_change(self):
        sorted_current_edges = [tuple(sorted(edge)) for edge in self.__current_comm_edges]
        sorted_previous_edges = [tuple(sorted(edge)) for edge in self.__comm_edges]

        # Compare the sorted edge lists
        return sorted_current_edges != sorted_previous_edges
    
    def update(self):
        
        self.__comm_edges = self.__current_comm_edges
        self.__comm_graph = defaultdict(list)

        for edge in self.__comm_edges:
            # create undirected graph. Append to each other neighbours
            self.__comm_graph[edge[0]].append(edge[1])
            self.__comm_graph[edge[1]].append(edge[0])

    
    
    def set_poses(self, poses: np.ndarray):
        self.__agent_pos = poses.copy()

    def get_pos(self):
        return self.__agent_pos

    def get_comm_graph(self):
        return self.__comm_graph

    def get_edge(self):
        return self.__comm_edges

    def are_all_agents_online(self):
        return np.all(self.online_status)
        
    def offline_agents(self):
        return np.where(self.online_status==0)[0] 