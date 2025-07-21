import numpy as np
import matplotlib.pyplot as plt

from copy import deepcopy
from collections import defaultdict

from solve_setpoint.solvers.shortest_path import get_shortest_distance

NUM_DRONES = 10

class GraphManager:
    #check if all agents are receiving there position
    online_status = np.zeros(10)

    def __init__(self):
        self.__agent_pos = np.zeros((NUM_DRONES, 3))
        self.__set_pos = np.zeros((NUM_DRONES, 3))
        self.__comm_edges = None
        self.__comm_graph = None
        self.__task_edges = None
        self.__tasks = []
        self.__task_paths = []
        self.__task_pos = []

        self.result_doubled = None

    def set_pos(self, msg, idx):
        """set the position of an agent by index"""
        if not self.online_status[idx]:
            self.online_status[idx] = 1

        self.__agent_pos[idx][0] = msg.pose.pose.position.x
        self.__agent_pos[idx][1] = msg.pose.pose.position.y
        self.__agent_pos[idx][2] = msg.pose.pose.position.z

    def set_edges(self, edges):
        self.__comm_edges = edges
        self.__comm_graph = [[] for _ in range(len(self.__comm_edges))]

        for edge in self.__comm_edges:
            self.__comm_graph[edge[0]].append(edge[1])
            self.__comm_graph[edge[1]].append(edge[0])

    def show_graph(self, show_task=False, pos=None):
        if pos is None:
            positions = self.__agent_pos
        else:
            positions = pos
        pos_list = [[positions[idx[0]][i], positions[idx[1]][i]] for idx in self.__comm_edges for i in range(2)]

        plt.plot(*pos_list, linestyle="dotted", color="blue")
        plt.scatter(positions[:,0], positions[:,1])
        for i, pos in enumerate(positions[:,:2]):
            plt.annotate("{}".format(i), pos+np.array([0.01, 0.01]))

        if show_task and len(self.__task_pos) > 0:
            task_idx = [(self.__task_paths[j][0][0], self.__task_paths[j][-1][1]) for j in range(len(self.__task_paths))]
            task_list = [[positions[idx[0],i], positions[idx[1],i]] for idx in task_idx for i in range(2)]
            plt.plot(*task_list, linestyle="dotted", color="green")

        plt.show()

    def load_tasks(self, tasks):
        for task in tasks:
            shortest_path = get_shortest_distance(self.__comm_graph, task[1][0], task[1][1], len(self.__agent_pos))
            shortest_edges = [[shortest_path[i], shortest_path[i+1]] for i in range(len(shortest_path)-1)]
            self.__task_paths.append(shortest_edges)
            self.__task_pos.append(task[2])

    def get_pos(self):
        return self.__agent_pos

    def get_comm_graph(self):
        return self.__comm_graph

    def compute_setpoint(self, result):
        edges = [edge for edge, _ in result]
        self.result_doubled = {tuple(edge):tuple(val) for edge, val in result}
        self.result_doubled.update({(edge[1], edge[0]): tuple(-val) for edge, val in result})

        self.comm_graph_res = defaultdict(list)

        set_list = deepcopy(self.__agent_pos)

        for u, v in edges:
            self.comm_graph_res[u].append(v)
            self.comm_graph_res[v].append(u)

    def get_setpoints(self):
        if self.result_doubled == None:
            return self.__set_pos
        else:
            setpoints = deepcopy(self.__agent_pos)

            for key in self.comm_graph_res:
                tot_diff = np.zeros(2)
                for n in self.comm_graph_res[key]:
                    desired = np.array(self.result_doubled[(key, n)])
                    actual = self.__agent_pos[n][:2] - self.__agent_pos[key][:2]
                    difference = actual - desired
                    tot_diff += difference

                setpoints[key][:2] += tot_diff

            return setpoints

    def rel_setpoint(self, log):
        results = {(0,3):(-0.66,-0.33),(3,0):(0.66,0.33), (3,6):(-0.66,-0.33), (6,3):(0.66,0.33), (6,9):(-0.66,-0.33), (9,6):(0.66,0.33)}
        comm_graph = {0:[3], 3:[0,6], 6:[3, 9], 9:[6]}

        setpoints = deepcopy(self.__agent_pos)

        for key in comm_graph:
            tot_diff = np.zeros(2)
            for n in comm_graph[key]:
                desired = np.array(results[(key, n)])
                actual = self.__agent_pos[n][:2] - self.__agent_pos[key][:2]
                difference = actual - desired
                tot_diff += difference

            setpoints[key][:2] += tot_diff

            log("for {} the total difference is {}".format(key, tot_diff))

        return setpoints

    def set_setpoints(self):
        self.__set_pos = deepcopy(self.__agent_pos) #TEMP!!