import numpy as np
from copy import deepcopy
from collections import defaultdict

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

#from barrier_msg.msg import Config

class GraphManager:
    #check if all agents are receiving there position
    

    def __init__(self, num_agents, comm_distance):
        self.NUM_AGENTS = num_agents
        self.COMM_DISTANCE = comm_distance

        self.__agent_pos = np.zeros((self.NUM_AGENTS, 3))
        self.__comm_edges = None
        self.__comm_graph = None

        self.online_status = np.zeros(self.NUM_AGENTS)

    def calc_edges(self):
        edges = []
        for i in range(len(self.__agent_pos)):
            for j in range(i+1, len(self.__agent_pos)):
                if np.abs(np.linalg.norm(self.__agent_pos[i]-self.__agent_pos[j])) < self.COMM_DISTANCE:
                    edges.append((i,j))

        if edges != self.__comm_edges:
            self.__set_edges(edges)
            return True
        else:
            return False

    def set_pos_callback(self, msg, idx, log):
        """set the position of an agent by index"""
        
    
        if not self.online_status[idx]:
            self.online_status[idx] = 1

        if type(msg) == Odometry:
            self.__agent_pos[idx][0] = msg.pose.pose.position.x
            self.__agent_pos[idx][1] = msg.pose.pose.position.y
            self.__agent_pos[idx][2] = msg.pose.pose.position.z
        elif type(msg) == PoseStamped:
            self.__agent_pos[idx][0] = msg.pose.position.x
            self.__agent_pos[idx][1] = msg.pose.position.y
            self.__agent_pos[idx][2] = msg.pose.position.z

    def get_weights(self):
        weights = {}

        for edge in self.__comm_edges:
            weights[tuple(edge)] = np.abs(np.linalg.norm(self.__agent_pos[edge[0]] - self.__agent_pos[edge[1]]))
            weights[(edge[1], edge[0])] = np.abs(np.linalg.norm(self.__agent_pos[edge[0]] - self.__agent_pos[edge[1]]))

        return weights

    def get_pos(self):
        return self.__agent_pos

    def get_comm_graph(self):
        return self.__comm_graph

    def get_edge(self):
        return self.__comm_edges

    def __set_edges(self, edges):
        self.__comm_edges = edges
        self.__comm_graph = defaultdict(list)

        for edge in self.__comm_edges:
            self.__comm_graph[edge[0]].append(edge[1])
            self.__comm_graph[edge[1]].append(edge[0])