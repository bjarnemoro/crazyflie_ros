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

def solve_task(agents, task_paths, task_pos, return_mode):
    """
    solves an optimization problem to find the most optimal center position of all agents
    given a set of tasks, each task is defined over a set of agents along their communication 
    graph.

    agents: list(Agent)
    start: np.array([[float, float]])
    task_paths = [[tuple], [tuple], ...]
    task_pos = [[float, flaot], ...]
    return_mode = "relative" or "absolute"
    """
    #create a list with a unique index of all agents that are used
    used_agents = used_agent(task_paths)

    if type(task_pos) == list:
        task_pos = np.array(task_pos)

    #create a variable for every agents exept the first, which provides a reference
    num_vars = len(used_agents) - 1
    dim = len(agents[0])
    
    cp_start_par = cp.Parameter((1, dim))
    cp_vars = cp.Variable((num_vars, dim))
    
    #cp_start_par = np.array([agents[task_paths[0][0][0]].get_position()])
    cp_start_par = np.array([agents[task_paths[0][0][0]][:dim]])
    par_var = cp.vstack([cp_start_par, cp_vars])

    #-----------------------------------------------------------------------
    #    Construct A and B matrices representing the paths along a task
    #-----------------------------------------------------------------------
    tot_path_length = len([item for sublist in task_paths for item in sublist]) #flatten the task path array

    A = np.zeros((tot_path_length, len(used_agents)))
    B = np.zeros((len(task_paths), len(used_agents)))

    counter = 0
    for i, task_path in enumerate(task_paths):
        #fill the A matrix with all agents along a path
        for edge in task_path:
            l = len(task_path)
            idx1 = used_agents.index(edge[0])
            idx2 = used_agents.index(edge[1])
            A[counter][idx1] = -1
            A[counter][idx2] = +1
            counter += 1

        #fill B matrix with first and final agent of a task
        B[i][used_agents.index(task_path[0][0])] = -1
        B[i][used_agents.index(task_path[-1][1])] = +1
    

    #perform the optimization
    objective = cp.Minimize(cp.norm(A @ par_var))
    prob = cp.Problem(objective, [B @ par_var == task_pos])

    result = prob.solve()

    #-----------------------------------------------------------------------------
    #   Return the results either in absolute positions or in relative positions
    #-----------------------------------------------------------------------------

    if return_mode == "relative":
        _, A_idx = np.unique(A, axis=0, return_index=True)
        
        path_flattened = np.array([item for sublist in task_paths for item in sublist])
        path_min = path_flattened[np.sort(A_idx)]

        A_min = A[np.sort(A_idx)]
        new_val = A_min @ par_var.value

        resulting_task = [(edge, val) for edge, val in zip(path_min, new_val)]
        return resulting_task

    elif return_mode == "absolute":
        resulting = [(used_agents[i], val) for i, val in enumerate(par_var.value)]
        return resulting

    else:
        raise ValueError("set return_mode to either \"relative\" or \"absolute\"")