
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import random,itertools
from typing import List
from copy import deepcopy

Path = List[float]  # type alias

# communication radius
R = 1.0

class Task:
    def __init__(self, type : str, center : np.ndarray, start : int, end : int, size : float, edge : tuple[int,int]):
        
        self.type   = type    # always/eventually
        self.center = center  # (x,y)
        self.start  = start   # start time
        self.end    = end     # end time
        self.size   = size    # size of the task region
        self.edge   = edge    # (u,v) edge where the task is located


def build_graph(comm_edges : list[tuple[int,int]], task_list: list[Task] = []) -> dict[dict]:
    """
    Use this to build a simple communication graph
    """
    graph = dict()
    for u, v in comm_edges:
        graph.setdefault(u, dict())[v] = 1 # counts the number of tasks passing through this edge
        graph.setdefault(v, dict())[u] = 1

    for task in task_list:
        u, v = task.edges

        try:
            graph[u][v] += 1
            graph[v][u] += 1
        except KeyError:
            pass # the task is not consistent with the communication graph so we do not add anything

    return graph


def select_communication_inconsistent_tasks(comm_graph : dict[dict,int], task_list: list[Task], comm_dist, log) -> list[Task]:
    """
    Select tasks that are inconsistent with the communication graph.
    """
    inconsistent_tasks = []
    consistent_tasks   = []
    
    for task in task_list:
        u, v = task.edges
        if v not in comm_graph[u]: # check if the task has an edge in the communication graph
            inconsistent_tasks.append(task)

        elif np.linalg.norm(task.rel_position) > comm_dist: # a task is inconsistent also if it requires a relative configuration that spans a bigger distance than the communication radius
            inconsistent_tasks.append(task)
        else:
            consistent_tasks.append(task)

    return inconsistent_tasks, consistent_tasks


def all_paths(graph, start, end, path=None, cost=0):
    """
    Returns all simple paths from start to end and their costs.
    graph[u] = {v: weight}
    """

    if path is None:
        path = []

    path = path + [start]

    if start == end:
        return [path], [cost]

    if start not in graph:
        return [], []

    paths = []
    costs = []

    for neighbor, weight in graph[start].items():
        if neighbor not in path:
            new_paths, new_costs = all_paths(
                graph,
                neighbor,
                end,
                path,
                cost + weight
            )
            paths.extend(new_paths)
            costs.extend(new_costs)

    return paths, costs

def add_weight_to_path(graph : dict[dict], path : list) -> int:
    """
        This function takes the communication graph and adds a plus one in weight
        to the edges in the given path. This is used once a path was used as decomposition path
        to increase the weight over that edge.
    """

    if len(path) < 2  :
        return 0

    for i in range(len(path) - 1):
        graph[path[i]][path[i+1]] += 10
        graph[path[i+1]][path[i]] += 10

    return graph



class DecompositionTrial:
    def __init__(self, permutation_of_tasks: list[Task], 
                       paths_dict          : dict[Task, tuple[Path,float]],                       
                       will_be_feasible    : bool,
                       total_cost          : float):
        
        self.permutation_of_tasks = permutation_of_tasks   # gives the original permutation of tasks that created this decomposition (we will need it just for visualization)
        self.paths_dict           = paths_dict             # for each task it contains (path, cost)
        self.will_be_feasible     = will_be_feasible       # if in the current communicaiton graph there is at least one task can not be bridged due to insufficient number of agents, then this is marked as unfeasible
        self.total_cost           = total_cost


def decomposition_path_guesses(inconsistent_tasks : list[Task], graph : dict[dict], communication_radious : float, logger = None) -> list[DecompositionTrial]:
    """
    Generate decomposition path guesses for a set of inconsistent tasks :

    1. At first we create all the possible permutations of the inconsistent tasks.
       This is because for each permutation, the weighting of the communication graph will 
       be done differently. leading to different choices of the decomposition path.
    
    2. For each task permutation we take the inconsistent tasks one by one. For each task we find
       all the paths between the source and sink and we get the cost of the path. The cost of the path is based on the number of tasks 
       assigned to each edge of the task.

    3. For each inconsistent edge we select the path based on two conditions:

       a. The path must be valid in the current communication graph. So the path length needs to be long enough based on the communication radious conditon norm(center)/comm_radious
       b. The path must minimize the cost, taking into account the weights assigned to the edges.
    4. After an optimal communication path is choosen, then a weight is added to each edge along the path ovr the communicaiton graph to represent that this edge was already used
       and it is expensive to reuse it.

       This process is repeated for each task in the permutation, resulting in a set of paths for each task.

       if there is at least one task for which there is no sufficiently long path that satisfies the minimum length condition norm(center)/comm_radious, then 
       the whole decomposition is declared already infeasible before the actual convex solver is called. We can still use this decomposition using slack variables.

    """
    

    logger.debug("Number of inconsistent tasks: {}".format(len(inconsistent_tasks)))
    paths_dict   = {} # for each task it contains a list of possible valid decomposition paths
    is_feasible  = True
    total_cost   = 0.
    
    for task in inconsistent_tasks:
        u, v = task.edges
        paths, costs = all_paths(graph , u, v)
        lengths = [len(p) - 1 for p in paths]

        center            = np.array(task.rel_position)
        min_length        = np.ceil(np.linalg.norm(center) / (communication_radious * 0.9))
        valid_paths_costs = [ (p,c) for p, c, l in zip(paths, costs, lengths) if l >= min_length]
        
        if not len(valid_paths_costs):
            logger.debug(f"No valid path found for task {task.edges} with min length {min_length}. Marking decomposition as infeasible.")
            paths_dict[task.ID] = paths

        # sort by cost
        valid_paths_costs.sort(key=lambda x: x[1])

        # keep only the paths
        valid_paths = [ p for p, c in valid_paths_costs]

        paths_dict[task.ID] = valid_paths

    return paths_dict




def plot_graph(graph, pos=None, ax=None):
    """
    Plot a weighted graph using matplotlib.
    
    Parameters:
        graph: dict of dict, {u: {v: weight}}
        pos: dict, optional node positions {node: (x,y)}
        ax: matplotlib axis
    """
    if ax is None:
        fig, ax = plt.subplots(figsize=(6,6))

    # If no positions given, arrange nodes in a circle
    if pos is None:
        n = len(graph)
        pos = {}
        for i, node in enumerate(graph.keys()):
            angle = 2 * 3.14159 * i / n
            pos[node] = (np.cos(angle), np.sin(angle))

    # Plot nodes
    for node, (x, y) in pos.items():
        ax.scatter(x, y, s=200, c="skyblue", zorder=3)
        ax.text(x, y, str(node), fontsize=12,
                ha="center", va="center", zorder=4)

    # Plot edges with thickness based on weight
    for u, neighbors in graph.items():
        for v, w in neighbors.items():
            if u < v:  # avoid double drawing undirected edges
                x1, y1 = pos[u]
                x2, y2 = pos[v]
                ax.plot([x1, x2], [y1, y2],
                        linewidth=1 + w,  # thickness grows with weight
                        color="black", alpha=0.7, zorder=2)
                # Show weight in the middle of the edge
                xm, ym = (x1+x2)/2, (y1+y2)/2
                ax.text(xm, ym, str(w), fontsize=10,
                        ha="center", va="center", color="red")

    ax.set_aspect("equal")
    ax.axis("off")

def plot_path(graph, path : Path) :
    """
    Plot a path in the communication graph.
    """
    fig, ax = plt.subplots(figsize=(6, 6))
    pos = {}
    for i, node in enumerate(graph.keys()):
        angle = 2 * np.pi * i / len(graph)
        pos[node] = (np.cos(angle), np.sin(angle))

    # Plot the graph
    plot_graph(graph, pos, ax)

    # Highlight the path
    for i in range(len(path) - 1):
        u, v = path[i], path[i + 1]
        x1, y1 = pos[u]
        x2, y2 = pos[v]
        ax.plot([x1, x2], [y1, y2], color="red", linewidth=2, zorder=5)

    plt.show()


# def plot_graph(graph, pos, ax):
#     ax.clear()

#     # Plot nodes
#     for node, (x, y) in pos.items():
#         ax.scatter(x, y, s=200, c="skyblue", zorder=3)
#         ax.text(x, y, str(node), fontsize=12,
#                 ha="center", va="center", zorder=4)

#     # Plot edges with thickness based on weight
#     for u, neighbors in graph.items():
#         for v, w in neighbors.items():
#             if u < v:  # avoid double-drawing undirected edges
#                 x1, y1 = pos[u]
#                 x2, y2 = pos[v]
#                 ax.plot([x1, x2], [y1, y2],
#                         linewidth=1 + w,  # thickness grows with weight
#                         color="black", alpha=0.7, zorder=2)
#                 xm, ym = (x1 + x2) / 2, (y1 + y2) / 2
#                 ax.text(xm, ym, str(w), fontsize=10,
#                         ha="center", va="center", color="red")

#     ax.set_aspect("equal")
#     ax.axis("off")

def animate_graph_updates(graph, kt_list, pos=None, interval=1500):
    """
    Animate graph updates for a list of (k, r) pairs.
    """
    fig, ax = plt.subplots(figsize=(6, 6))

    if pos is None:
        n = len(graph)
        pos = {}
        for i, node in enumerate(graph.keys()):
            angle = 2 * np.pi * i / n
            pos[node] = (np.cos(angle), np.sin(angle))

    # State for animation
    step = {"i": 0}

    def update(frame):
        ax.clear()
        i = step["i"]

        if i < len(kt_list):
            k, r = kt_list[i]
            paths, costs = all_paths(graph, k, r)

            if paths:  # if paths exist
                idx = max(range(len(costs)), key=lambda j: costs[j])
                chosen_path, chosen_cost = paths[idx], costs[idx]
                print(f"Step {i}: updating path {chosen_path} (cost {chosen_cost})")

                # Update graph with chosen path
                add_weight_to_path(graph, chosen_path)

            step["i"] += 1

        plot_graph(graph, pos, ax)

    ani = animation.FuncAnimation(fig, update,
                                  frames=len(kt_list) + 1,
                                  interval=interval,
                                  repeat=False)
    plt.show()


if __name__ == "__main__":


    # define a K with 5 agents
    #
    #  5    4
    #  
    #  3
    #
    #  1    2

    tasks   = [Task(type="always", center=np.array([0., -10]), start=10, end=20, size=0.2, edge=(1, 3)),
               Task(type="always", center=np.array([0., 10]), start=10, end=20, size=0.2, edge=(3, 5)),
               Task(type="always", center=np.array([10., 0]), start=10, end=20, size=0.2, edge=(5, 4)),
               Task(type="always", center=np.array([10., 0]), start=10, end=20, size=0.2, edge=(1, 2))]

    

    edges = [(0, 1), (0, 2), (1, 2),(2, 3),(4,9) ,(2,4), (1,4),(7,3),(9,1),(9,5),(7,3),(6,4),(5,6),(3,6)]
    comm_graph = build_graph(edges)
    inconsistent_tasks = select_communication_inconsistent_tasks(comm_graph, tasks)
    print("Initial number of tasks:", len(tasks))
    print("Got number of inconsistent tasks:", len(inconsistent_tasks))
    
    # communication radius
    R = 6.0
    list_of_decompositions = decomposition_path_guesses(inconsistent_tasks, comm_graph, R)
    for decomposition in list_of_decompositions:
        for task, path in decomposition.paths_dict.items():
            print(f"Task {task.edge} assigned path {path[0]} with cost {path[1]}")
            plot_path(comm_graph, path[0])

    plot_graph(comm_graph)
    plt.show()
    
    
    # # Define sequence of (k, r) queries
    # kt_list = [(0, 3), (1, 3), (0, 2), (2, 3),(1,6),(1,4)]

    # animate_graph_updates(comm_graph, kt_list)
