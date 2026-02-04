import os
import json
import time
import itertools
import dataclasses
import numpy as np
from collections import  defaultdict
from dataclasses import dataclass
from barrier_msg.msg import BMsg, TMsg

from solve_setpoint.solvers.shortest_path import dijkstra
from solve_setpoint.solvers.graph_search import build_graph, select_communication_inconsistent_tasks, list_decomposition_paths_per_task


class Task:
    __counter = 0
    def __init__(self, timespan: list[int, int], 
                       edges: list[int, int], 
                       rel_position: list[float, float] | list[float, float, float], 
                       size: float, 
                       period_num:int,
                       operator: str = "always"):
        self.timespan     = timespan
        self.edges        = edges
        self.rel_position = rel_position
        self.size         = size
        self.period_num   = period_num
        self.operator     = operator
        self.ID           = Task.__counter
        Task.__counter  += 1

class TaskManager():
    def __init__(self, dim, comm_dist, periods: list[tuple[int,int]], tasks=None, task_path=None):
        """
        Load the the tasks either via a json file or an array of tasks with the following structure:
        [([],[],[]), ..., ([],[],[])] with each tuple having time period, agent idx, relative pos
        i.e [0,3], [0, 4], [10, 20] so from time 0 to 3 sec agent 0 has a postion of [10, 20] compared to agent 4
        """

        self.DIM           = dim
        self.COMM_DISTANCE = comm_dist
        self.periods       = periods

        if task_path is not None:
            self.load_task_path(task_path)
        elif tasks is not None:
            self.__tasks = tasks    
        else :
            raise Exception("Either tasks or task_path must be provided to load the tasks")

        # divide tasks by time periods
        self.task_per_period = defaultdict(list)
        for task in self.__tasks:
            self.task_per_period[task.period_num].append(task)

    def recalculate_at(self):
        """
        Trigger a recalculation every time ther task ends
        """
        timespans = [task.timespan[1] for task in self.__tasks] + [0] # every time a task end we can recalculate the tasks. Added zero for initial calculation
        timespans = list(set(timespans))                              # remove duplicates and retrun ordered list
        timespans.sort()
        return timespans

    
    def get_active_tasks(self, current_time):
        """
        Takes current time as input.

        The funciton computes the currently active tasks (i.e. not yet concluded).

        The returned values are:

        - active_tasks: list of active tasks
        """
        
        active_tasks = []
        # find corrent period
        active_period_num = None
        for j,period in enumerate(self.periods):
            if current_time < period[1] :
                active_period_num = j
                break
        active_tasks = self.task_per_period[active_period_num]
        return active_tasks

    def get_active_agents(self, current_time):
        """
        Takes current time as input.

        The funciton computes the currently active agents (i.e. agents involved in not yet concluded tasks).

        The returned values are:

        - active_agents: list of active agents
        """
        
        active_tasks = self.get_active_tasks(current_time)
        active_agents = set()
        for task in active_tasks:
            active_agents.add(task.edges[0])
            active_agents.add(task.edges[1])
        return list(active_agents)
    
    def get_active_tasks_and_decomposition_paths(self, current_time, comm_edges, log):

        """
        Takes current time, communication edges as input.

        The funciton computes the currently active tasks (i.e. not yet concluded) and computes for each task the decomposition path.
        The decomposition path is the path through the communication edges that connects the agents involved in the task.

        The returned values are:

        - task_paths   : list of list of edges for each active task
        - task_pos     : list of relative positions for each active task
        - task_box_size: list of box sizes for each active task e.g [1.,1.] to indicate the relative tolerance is 1 by 1 meters

        """

        active_tasks = self.get_active_tasks(current_time)
        active_edge_tasks = [task for task in active_tasks if task.edges[0] != task.edges[1]]
        active_self_tasks = [task for task in active_tasks if task.edges[0] == task.edges[1]]

        for i, taski in enumerate(active_edge_tasks):
            for j, taskj in enumerate(active_edge_tasks):
                if i < j:
                    if taski.edges == taskj.edges or taski.edges == (taskj.edges[1], taskj.edges[0]):
                        raise Exception("Multiple tasks assigned to the same edge {} at priod {}".format(taski.edges,taski.period_num))

        possible_paths, decomposition_needed, is_disconnected  = self._compute_task_paths(active_edge_tasks, comm_edges, log) # for each active task it computes the decomposition path. 
        
        # add self tasks 
        for task in active_self_tasks:
            for decomption_attempt in  possible_paths :
                decomption_attempt.append([task.edges])
        
        return active_self_tasks, active_edge_tasks, possible_paths, decomposition_needed, is_disconnected
        
    def _compute_task_paths(self, active_tasks: list[Task], comm_edges, log):
        """
        Divides the tasks in consistent and inconsistent tasks based on the current communication graph.
        A task is consistent if 
            1. There exists a direct communication edge between the agents involved in the task 
            2. The required relative configuration by the tasks is such that the communication will not limit its satisfaction
        """

        graph                                = build_graph(comm_edges, active_tasks)
        inconsistent_tasks, consistent_tasks = select_communication_inconsistent_tasks(graph, active_tasks, self.COMM_DISTANCE, log)
        
        decomposition_needed = False 
        is_diconnected       = False


        if not len(inconsistent_tasks):
            decomposition_paths_dict = {}
            return decomposition_paths_dict, decomposition_needed, is_diconnected
       
        
        decomposition_needed         = True
        decomposition_paths_dict     = list_decomposition_paths_per_task(inconsistent_tasks, graph, self.COMM_DISTANCE, log) # returns a list of possible decomposition
        possible_decomposition_paths = []
        attempts = 100
        
        for _ in range(attempts) : # try multiple times to find decomposition paths
            decomposition_paths_attempt = []
            for task in active_tasks:
                if task in consistent_tasks:
                    decomposition_paths_attempt.append([task.edges])
                if task in inconsistent_tasks:
                    possible_paths = decomposition_paths_dict[task.ID]
                    
                    if len(possible_paths) == 0:
                        path = []
                        decomposition_paths_attempt.append(path)
                        is_diconnected = True
                    else :
                        # sample a path biasing for shorter paths
                        idx = int(np.random.exponential(scale=1.0))
                        idx = min(idx, len(possible_paths) - 1)
                        path = [[possible_paths[idx][i], possible_paths[idx][i+1]] for i in range(len(possible_paths[idx])-1)]
                        decomposition_paths_attempt.append(path)
            
            possible_decomposition_paths.append(decomposition_paths_attempt) # define list of n = attempts decomposition path. Each attempt defined a path of decomositoon for the task

        return possible_decomposition_paths, decomposition_needed , is_diconnected

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
    
    def get_tmsg(self, task):
        
        tmsg = TMsg()
        
        tmsg.center.extend([ i for i in task.rel_position])
        tmsg.size.extend([max(2*task.size,0.1) for _ in range(self.DIM * 2)])
        
        tmsg.start   = float(task.timespan[0])
        tmsg.end     = float(task.timespan[1])
        tmsg.edge_i  = int(task.edges[0])
        tmsg.edge_j  = int(task.edges[1])
        tmsg.type    = task.operator

        return tmsg

    def __repr__(self):
        return "{}".format(self.__tasks)