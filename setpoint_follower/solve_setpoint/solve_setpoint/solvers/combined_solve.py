import cvxpy as cp
import numpy as np
import matplotlib.pyplot as plt

def flatten(i):
    return [k for j in i for k in j]

def used_agent(task_paths):
    used_agents = []
    for task_path in task_paths:
        for vals in task_path:
            for i in range(2):
                if vals[i] not in used_agents:
                    used_agents.append(vals[i])

    return used_agents
    
def task_vars(task_paths, used_agents) -> np.ndarray | np.ndarray:
    tot_path_length = len(flatten(task_paths)) #flatten the task path array

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

    return A, B

def box_vars(task_paths, task_box) -> np.ndarray:
    unique_path, indices = np.unique(flatten(task_paths), return_index=True, axis=0)

    idx_path = []
    for i in range(len(task_paths)):
        idx_path.append([np.where(np.all(unique_path==edge,axis=1))[0][0] for edge in task_paths[i]])

    tasks_selector = np.zeros((len(task_box), len(indices)))
    for i, path in enumerate(idx_path):
        for idx in path:
            tasks_selector[i][idx] = 1

    return tasks_selector, indices

def vertices(bbox, box_dim):
    if box_dim == 4:
        v1 = cp.hstack([bbox[:,0:1], bbox[:,2:3]])
        v2 = cp.hstack([bbox[:,0:1], -bbox[:,3:4]])
        v3 = cp.hstack([-bbox[:,1:2], bbox[:,2:3]])
        v4 = cp.hstack([-bbox[:,1:2], -bbox[:,3:4]])
        bbox_vertices = cp.vstack([v1, v2, v3, v4])

    elif box_dim == 6:
        v1 = cp.hstack([bbox[:,0:1], bbox[:,2:3], bbox[:,4:5]])
        v2 = cp.hstack([bbox[:,0:1], -bbox[:,3:4], bbox[:,4:5]])
        v3 = cp.hstack([-bbox[:,1:2], bbox[:,2:3], bbox[:,4:5]])
        v4 = cp.hstack([-bbox[:,1:2], -bbox[:,3:4], bbox[:,4:5]])
        v5 = cp.hstack([bbox[:,0:1], bbox[:,2:3], -bbox[:,5:6]])
        v6 = cp.hstack([bbox[:,0:1], -bbox[:,3:4], -bbox[:,5:6]])
        v7 = cp.hstack([-bbox[:,1:2], bbox[:,2:3], -bbox[:,5:6]])
        v8 = cp.hstack([-bbox[:,1:2], -bbox[:,3:4], -bbox[:,5:6]])
        bbox_vertices = cp.vstack([v1, v2, v3, v4, v5, v6, v7, v8])

    return bbox_vertices


def solve_combined(task_paths: np.ndarray, task_pos: np.ndarray, task_box: np.ndarray, return_mode: str, BOX_WEIGHT, COMM_DISTANCE) -> bool:
    """
    
    """
    assert len(task_paths[0][0]) == 2, "make sure your task path is a list of a path of edges"
    assert len(task_paths) == len(task_pos)
    assert len(task_pos) == len(task_box)
    assert len(task_box[0]) == 2*len(task_pos[0]) #box and position have same dimension (2d / 3d)

    
    if type(task_pos) != np.ndarray:
        task_pos = np.array(task_pos)

    if type(task_box) != np.ndarray:
        task_box = np.array(task_box)

    #create a list with a unique index of all agents that are used
    used_agents = used_agent(task_paths)

    #----------------------------------------
    #    obtain the main selector matrices
    #----------------------------------------
    rel_task_pos_select, abs_task_pos_select = task_vars(task_paths, used_agents)
    tasks_selector, rel_indices = box_vars(task_paths, task_box)

    
    #-------------------------------------------
    #    Variables for position optimization
    #-------------------------------------------

    #create a variable for every agents exept the first, which provides a reference
    num_vars = len(used_agents) - 1
    dim = len(task_pos[0])
    
    cp_start_par = np.zeros((1,dim))
    cp_vars = cp.Variable((num_vars, dim))
    agent_pos = cp.vstack([cp_start_par, cp_vars])
    
    #---------------------------------------
    #    Variables for box optimization
    #---------------------------------------
    box_dim = len(task_box[0])
    unique_tasks = len(np.unique(flatten(task_paths), axis=0))
    bbox = cp.Variable((unique_tasks, box_dim))#n boxes x, -x, y, -y

    bbox_vertices = vertices(bbox, box_dim)

    tasks = rel_task_pos_select[rel_indices] @ agent_pos
    num_vert = bbox_vertices.shape[0] // unique_tasks
    task_stacked = cp.vstack([tasks for i in range(num_vert)])
    
    
    #--------------------------------------------------------------------------
    #    Perform the main optimization of the positions as well as the boxes
    #--------------------------------------------------------------------------

    #define the objective, box size is prioritesed over edge length
    task_objective = cp.norm(rel_task_pos_select @ agent_pos, 'fro')
    #box_objective = -cp.sum(bbox) -MIN_WEIGHT1*cp.sum(cp.min(bbox, axis=0)) -MIN_WEIGHT2*cp.sum(cp.min(bbox)) #cp.sum(task_box - tasks_selector @ bbox)
    box_objective = -cp.sum(bbox) + cp.dotsort(cp.min(bbox, axis=0), np.array([-(i+1)**2 for i in range(box_dim)]))

    objective = cp.Minimize(task_objective+BOX_WEIGHT*box_objective)

    #define the constraints 
    task_constraint = [
        abs_task_pos_select @ agent_pos == task_pos]
    
    box_constraint = [
        tasks_selector @ bbox <= task_box,
        cp.norm(bbox_vertices+task_stacked, axis=1)<=COMM_DISTANCE,
        bbox >= 0]
                                   
    
    prob = cp.Problem(objective, task_constraint + box_constraint)
    result = prob.solve(solver=cp.CLARABEL)

    #check whether optimization was succesful
    if prob.status in ["infeasible", "unbounded"]:
        succes = False
        return succes, None, None
    
    succes = True
    
    if return_mode == "relative":
        rel_positions = rel_task_pos_select[rel_indices] @ agent_pos.value
        rel_box = bbox.value
        rel_edges = np.array(flatten(task_paths))[rel_indices]

        pos_edge_list = [(edge, pos) for edge, pos in zip(rel_edges, rel_positions)]
        box_edge_list = [(edge, box) for edge, box in zip(rel_edges, rel_box)]

        return succes, pos_edge_list, box_edge_list


    if return_mode == "absolute":
        return agent_pos.value