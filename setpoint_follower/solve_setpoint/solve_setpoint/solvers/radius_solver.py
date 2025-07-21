import cvxpy as cp
import numpy as np
import matplotlib.pyplot as plt


def used_agent(task_paths):
    used_agents = []
    for task_path in task_paths:
        for vals in task_path:
            for i in range(2):
                if vals[i] not in used_agents:
                    used_agents.append(vals[i])

    return used_agents

def solve_radius(agents, task_paths, task_rad, return_mode):
    """
    solves an optimization problem to find the most optimal boundary box radius of all agents
    given a set of tasks, each task is defined over a set of agents along their communication 
    graph.

    agents: list(Agent)
    task_paths = [[tuple], [tuple], ...]
    task_pos = [[float, flaot], ...]
    return_mode = "relative" or "absolute"
    """
    #create a list of the used agents
    used_agents = used_agent(task_paths)
    num_areas = len(used_agents)

    areas = cp.Variable((1, num_areas))
    
    #------------------------------------------------------------------
    #    contruct the main matrices to perform the optimizations
    #------------------------------------------------------------------
    #a path contains unique indices in order of a task path
    paths = []
    for path in task_paths:
        flattened = np.array([num for a in path for num in a])
        _, idx = np.unique(path, return_index=True)
        my_path = flattened[np.sort(idx)]
        paths.append(my_path)

    #create an array with a 1 representing an agent being used in a task
    task_paths = np.zeros((len(task_paths), num_areas))
    for i, path in enumerate(paths):
        used_path = [used_agents.index(p) for p in path]
        task_paths[i][used_path] = 1 

    summed_task = np.sum(task_paths, axis=0)

    #perform optimization
    objective = cp.Minimize(-cp.sum(summed_task @ areas.T) + cp.norm(areas-np.max(task_rad)/num_areas, 2))#
    prob = cp.Problem(objective, [areas >= 0, areas @ task_paths.T <= task_rad]) 

    result = prob.solve()

    #-----------------------------------------------------------------------------
    #   Return the results either in absolute positions or in relative positions
    #-----------------------------------------------------------------------------

    if return_mode == "relative":
        return_vals = []
        for path in paths:
            #path_idx = [paths.index(p) for p in path[:i] for i in range(len(path))]
            #summed_areas = [np.sum(areas.value[0][path[:i]]) for i in range(len(path))]
            #TODO!!!!
            path = None
            summed_areas = None
            return_vals.append((path, summed_areas))
        return return_vals
    elif return_mode == "absolute":
        return_val = [(agent, value) for agent, value in zip(used_agents, areas.value[0])]
        return return_val
    else:
        raise ValueError("set return_mode to either \"relative\" or \"absolute\"")