import numpy as np
from copy import deepcopy
from collections import defaultdict

from barrier_msg.msg import Config

class GraphManager:
    #check if all agents are receiving there position
    

    def __init__(self):
        self.__agent_pos = np.zeros((Config.NUM_AGENTS, 3))
        self.__set_pos = np.zeros((Config.NUM_AGENTS, 3))
        self.__comm_edges = None
        self.__comm_graph = None
        self.__task_edges = None
        self.__result_doubled = None
        self.__comm_graph_res = None

        self.online_status = np.zeros(10)

    def init_setpoints(self):
        self.__set_pos = deepcopy(self.__agent_pos)
        self.__set_pos[:,2] = 1

    def compute_setpoint(self, result):
        edges = [edge for edge, _ in result]
        self.dim = len(result[0][1])
        self.__result_doubled = {tuple(edge):tuple(val) for edge, val in result}
        self.__result_doubled.update({(edge[1], edge[0]): tuple(-val) for edge, val in result})

        self.__comm_graph_res = defaultdict(list)

        set_list = deepcopy(self.__agent_pos)

        for u, v in edges:
            self.__comm_graph_res[u].append(v)
            self.__comm_graph_res[v].append(u)

    def get_setpoints(self):
        if self.__result_doubled == None:
            return self.__set_pos
        else:
            setpoints = deepcopy(self.__agent_pos)

            for key in self.__comm_graph_res:
                tot_diff = np.zeros(self.dim)
                for n in self.__comm_graph_res[key]:
                    desired = np.array(self.__result_doubled[(key, n)])
                    actual = self.__agent_pos[n][:self.dim] - self.__agent_pos[key][:self.dim]
                    difference = actual - desired
                    tot_diff += difference

                setpoints[key][:self.dim] += tot_diff

            return setpoints

    def calc_edges(self):
        edges = []
        for i in range(len(self.__agent_pos)):
            for j in range(i+1, len(self.__agent_pos)):
                if np.abs(np.linalg.norm(self.__agent_pos[i]-self.__agent_pos[j])) < Config.COMM_DISTANCE:
                    edges.append((i,j))

        if edges != self.__comm_edges:
            self.__set_edges(edges)
            return True
        else:
            return False

    def set_pos_callback(self, msg, idx):
        """set the position of an agent by index"""
        if not self.online_status[idx]:
            self.online_status[idx] = 1

        self.__agent_pos[idx][0] = msg.pose.pose.position.x
        self.__agent_pos[idx][1] = msg.pose.pose.position.y
        self.__agent_pos[idx][2] = msg.pose.pose.position.z

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
        self.__comm_graph = [[] for _ in range(len(self.__comm_edges))]

        for edge in self.__comm_edges:
            self.__comm_graph[edge[0]].append(edge[1])
            self.__comm_graph[edge[1]].append(edge[0])