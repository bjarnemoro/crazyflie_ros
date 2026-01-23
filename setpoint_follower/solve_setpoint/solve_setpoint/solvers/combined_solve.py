import cvxpy as cp
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict
import copy


def get_H_and_b(box_dim):
    
    H = np.vstack((np.eye(box_dim),-np.eye(box_dim)))
    b = np.ones(2*box_dim) * 0.5
    return H,b
      

def vertices(dim):
    """
    Returns all vertices of a hypercube centered at zero
    with side length 1, for dimension 2 or 3.
    """
    if dim == 2:
        return [
            np.array([-0.5, -0.5]),
            np.array([-0.5,  0.5]),
            np.array([ 0.5, -0.5]),
            np.array([ 0.5,  0.5]),
        ]

    elif dim == 3:
        return [
            np.array([-0.5, -0.5, -0.5]),
            np.array([-0.5, -0.5,  0.5]),
            np.array([-0.5,  0.5, -0.5]),
            np.array([-0.5,  0.5,  0.5]),
            np.array([ 0.5, -0.5, -0.5]),
            np.array([ 0.5, -0.5,  0.5]),
            np.array([ 0.5,  0.5, -0.5]),
            np.array([ 0.5,  0.5,  0.5]),
        ]

    else:
        raise ValueError("Dimension must be 2 or 3")


class SimpleGraph:
    def __init__(self, tasks, task_paths):

        self.edges = defaultdict(lambda: defaultdict(list))
        self.edge_list = []

        edge_vars  = {} # disctionaty of task and related edge variables
        scale_vars = {}
        for task, path in zip(tasks, task_paths):
            edge_vars[task] = [cp.Variable(len(task.rel_position)) for edge in path]
            scale_vars[task] = [cp.Variable() for edge in path]

        for task, path in zip(tasks, task_paths):
            for jj,edge in enumerate(path):
                u,v            = edge
                directed_edge = (u,v) # it is important to give the same direction to the edge

                if not len(self.edges[u][v]): # only add the task once
                    self.edges[u][v] = [edge_vars[task][jj], scale_vars[task][jj], directed_edge]
                    self.edges[v][u] = [edge_vars[task][jj], scale_vars[task][jj], directed_edge]
                    self.edge_list.append(directed_edge)

    def find_all_cycles(self):
        """
        Finds all simple cycles in the graph.
        Only the start/end node may repeat.
        Returns a list of cycles (node lists).
        """
        cycles = []

        def dfs(start, current, path):
            for neighbor in self.edges[current]:
                if neighbor == start and len(path) > 2:
                    cycles.append(path + [start])
                elif neighbor not in path:
                    dfs(start, neighbor, path + [neighbor])

        for node in self.edges:
            dfs(node, node, [node])

        return cycles
            
def solve_task_decomposition(tasks , task_paths: list[tuple[int,int]], BOX_WEIGHT, COMM_DISTANCE, logger) -> bool:

    #create a list with a unique index of all agents that are used
    dim           = len(tasks[0].rel_position)
    H,b           = get_H_and_b(box_dim= dim)
    vertices_list = vertices(dim)
    
    graph         = SimpleGraph(tasks, task_paths)
  
    # close the loop with every task 
    constraints = []
    logger.info(f"{task_paths}")
    for task, path in zip(tasks, task_paths):
       
        e_sum       = 0
        alpha_sum   = 0
        
        e_rel       = task.rel_position
        edge        = task.edges
        path_length = len(path)
        
        for edge in path:
            u,v = edge
            edge_vars_pair = graph.edges[u][v]
            
            e_var         = edge_vars_pair[0]
            scale_var     = edge_vars_pair[1]
            directed_edge = edge_vars_pair[2]
            if directed_edge != (u,v): # invert direction if you traverse the edge backwards
                e_var = -e_var
            
            e_sum     += e_var
            alpha_sum += scale_var
            
            constraints += [cp.norm(e_var) <= COMM_DISTANCE]
            constraints += [scale_var >= 0]

        # containment constraint
        for vertex in vertices_list:
            vv = e_sum + alpha_sum * task.size * vertex
            constraints.append(H @ (e_rel- vv) <= b * task.size )

    # find cycles TODO: implement cycle overload
    cycles = graph.find_all_cycles()
    logger.info(f"Found {len(cycles)} cycles in the graph")

    # define cost
    cost = 0.
    C_norm = 0.1
    epsilon = 0.01
    for edge in graph.edge_list:
        i,j = edge
        edge_vars_pair = graph.edges[i][j]
        e_var, scale_var,_ = edge_vars_pair
        
        t = cp.Variable()
        constraints += [t >= 0]

        cost += -BOX_WEIGHT *scale_var 
        cost += C_norm * cp.norm(e_var)


    prob = cp.Problem(cp.Minimize(cost), constraints)
    prob.solve(solver=cp.CLARABEL, verbose=False)
    
    if prob.status != cp.OPTIMAL:
        return prob.status, []
    
    # create a new sets of tasks
    new_tasks   = []
    added_edges = []

    for task, path in zip(tasks, task_paths):
        for edge in path:
            
            u,v = edge
            edge_vars_pair = graph.edges[u][v]
            
            e_var     =   edge_vars_pair[0]
            scale_var = edge_vars_pair[1]

            e_value     = e_var.value
            scale_value = scale_var.value
            if edge != (u,v): # invert direction if you traverse the edge backwards
                e_value = -e_value
            
            if (u,v) in added_edges or (v,u) in added_edges:
                continue # do not add the task twice
            
            new_task = copy.deepcopy(task)
            
            # add the new task!
            new_task.edges        = (u,v)
            new_task.rel_position = e_value.flatten()
            new_task.size         = scale_value 
            
            new_tasks.append(new_task)
            added_edges.append((u,v))
            logger.info(f"Norm of edge {u}-{v}: {np.linalg.norm(e_value)} with scale {scale_value}")
        
    return prob.status, new_tasks   