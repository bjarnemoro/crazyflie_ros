import os
import json
import time
import dataclasses
import numpy as np
from collections import  defaultdict
from dataclasses import dataclass
from barrier_msg.msg import BMsg, TMsg
from barrier_msg.msg import Config

from solve_setpoint.solvers.shortest_path import dijkstra

def flatten(xss):
    return [x for xs in xss for x in xs]

@dataclass
class Task:
    timespan: list[int, int]
    edges: list[int, int]
    rel_position: list[int, int] | list[int, int, int]


class TaskManager():
    def __init__(self, tasks=None, task_path=None):
        """load the the tasks either via a json file or an array of tasks with the following structure:
        [([],[],[]), ..., ([],[],[])] with each tuple having time period, agent idx, relative pos
        i.e [0,3], [0, 4], [10, 20] so from time 0 to 3 sec agent 0 has a postion of [10, 20] compared to agent 4"""
        if task_path is not None:
            self.load_task_path(task_path)
        elif tasks is not None:
            self.__tasks = tasks

    def recalculate_at(self):
        timespans = [task.timespan for task in self.__tasks]
        return set(flatten(timespans))

    def obtain_current_tasks(self, time, comm_graph, weights):
        active_tasks = []
        for task in self.__tasks:
            if time >= task.timespan[0] and time < task.timespan[1]:
                active_tasks.append(task)

        task_paths = [self.__relative_tasks(task, comm_graph, weights) for task in active_tasks]
        task_pos = [task.rel_position for task in active_tasks]
        task_rad = [[10 for i in range(2*len(task.rel_position))] for task in active_tasks]

        self.time_lookup = defaultdict(list)
        for task, path in zip(active_tasks, task_paths):
            for edge in path:
                self.time_lookup[tuple(edge)].append(task.timespan)

        return (task_paths, task_pos, task_rad)
    

    def __relative_tasks(self, task: Task, comm_graph, weights):
        """convert the absolute tasks to relative tasks, absolute tasks are defined between any agent
        relative tasks are defined between agents connected along the communication graph"""
        #shortest_path = get_shortest_distance(comm_graph, task.edges[0], task.edges[1], max(flatten(comm_graph))+1)
        shortest_path = dijkstra(comm_graph, weights, task.edges[0], task.edges[1])
        if shortest_path != "impossible":
            shortest_edges = [[shortest_path[i], shortest_path[i+1]] for i in range(len(shortest_path)-1)]
            return shortest_edges
        else:
            return "impossible"

    def load_task_path(self, task_path):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        output_path = os.path.join(current_dir, task_path)

        with open(output_path) as read_file:
            task_dicts = json.load(read_file)

        self.__tasks = [Task(**d) for d in task_dicts]

    def save_tasks(self, task_path):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        output_path = os.path.join(current_dir, task_path)

        data = [dataclasses.asdict(task) for task in self.__tasks]

        with open(output_path, mode="w", encoding="utf-8") as write_file:
            written = json.dump(data, write_file)

    def get_tasks(self, time):
        active_tasks = []
        for task in self.__tasks:
            if time >= task.timespan[0] and time < task.timespan[1]:
                active_tasks.append(task)

        return[(task.edges, task.rel_position) for task in active_tasks]
    
    def get_tmsg(self, t, edge_pos, edge_box):
        msgs = []
        for edge, pos in edge_pos:
            for timespan in self.time_lookup[tuple(edge)]:
                tmsg = TMsg()
                tmsg.center.extend([float(e) for e in pos])
                tmsg.size.extend([2. for _ in range(Config.DIM * 2)])
                tmsg.start = float(4)
                tmsg.end = float(6)
                tmsg.edge_i = int(edge[0])
                tmsg.edge_j = int(edge[1])
                tmsg.type = "always"
                msgs.append(tmsg)

        return msgs

    def __repr__(self):
        return "{}".format(self.__tasks)