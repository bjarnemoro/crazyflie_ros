import cvxpy as cp
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict
import copy
from solve_setpoint.task_manager import Task


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
        vertices =  [
            np.array([-1., -1.0]),
            np.array([-1.,  1.0]),
            np.array([ 1., -1.0]),
            np.array([ 1.,  1.0]),
        ]

        normalized_vertices = [v / np.linalg.norm(v) for v in vertices]
        return normalized_vertices

    elif dim == 3:
        vertices =  [
            np.array([-1., -1., -1.]),
            np.array([-1., -1.,  1.]),
            np.array([-1.,  1., -1.]),
            np.array([-1.,  1.,  1.]),
            np.array([ 1., -1., -1.]),
            np.array([ 1., -1.,  1.]),
            np.array([ 1.,  1., -1.]),
            np.array([ 1.,  1.,  1.]),
        ]

        normalized_vertices = [v / np.linalg.norm(v) for v in vertices]
        return normalized_vertices

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
                u,v           = edge
                directed_edge = [u,v] # it is important to give the same direction to the edge

                if not len(self.edges[u][v]): # only add the task once
                    self.edges[u][v] = [edge_vars[task][jj], scale_vars[task][jj], directed_edge]
                    self.edges[v][u] = [edge_vars[task][jj], scale_vars[task][jj], directed_edge]
                    self.edge_list.append(directed_edge)

    def find_all_cycles(self):
        cycles = []
        unique_cycles = set()

        def get_canonical(cycle):
            # 1. Remove the repeated end node to work with just the vertices
            nodes = cycle[:-1]
            # 2. Find the rotation that starts with the smallest node
            min_idx = nodes.index(min(nodes))
            rotated = nodes[min_idx:] + nodes[:min_idx]
            # 3. Check both directions and pick the lexicographically smaller one
            rev = rotated[:1] + rotated[1:][::-1]
            return tuple(min(rotated, rev))

        def dfs(start, current, path):
            for neighbor in self.edges[current]:
                if neighbor == start and len(path) > 2:
                    # Create a canonical version of the cycle to track uniqueness
                    canonical = get_canonical(path + [start])
                    if canonical not in unique_cycles:
                        unique_cycles.add(canonical)
                        cycles.append(list(canonical) + [canonical[0]])
                elif neighbor not in path:
                    # Optimization: To avoid redundant work, only visit neighbors
                    # with a higher index than the start node
                    if neighbor > start: 
                        dfs(start, neighbor, path + [neighbor])

        # Sort nodes to ensure a consistent starting point for the search
        nodes = sorted(self.edges.keys())
        for node in nodes:
            dfs(node, node, [node])

        return cycles
                
def solve_task_decomposition(tasks , task_paths: list[tuple[int,int]], BOX_WEIGHT, COMM_DISTANCE, agents_position, logger) -> bool:

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
            u1,v1         = edge_vars_pair[2]
            
            if (u1,v1) != (u,v):
                e_sum -= e_var 
            else:
                e_sum     += e_var
            
            alpha_sum += scale_var
            
            constraints += [cp.norm(e_var) <= COMM_DISTANCE]
            constraints += [scale_var >= 0]

        # containment constraint
        for vertex in vertices_list:
            vv = e_sum + alpha_sum * vertex
            constraints.append(H @ (e_rel- vv) <= b * task.size )

    # find cycles TODO: implement cycle overload
    cycles = graph.find_all_cycles()
    logger.debug(f"Found {len(cycles)} cycles in the graph")
    for i, cycle in enumerate(cycles):
        logger.info(f"Cycle {i}: {cycle}")

    # add constraint on the cycle
    for cycle in cycles:
        cycle_sum = 0.
        cycle_scale_sum = 0.
        cycle_path = [cycle[i:i+2] for i in range(len(cycle)-1)]

        for edge in cycle_path:
            u,v = edge
            edge_vars_pair = graph.edges[u][v]
            e_var         = edge_vars_pair[0]
            scale_var     = edge_vars_pair[1]
            u1,v1         = edge_vars_pair[2]
            
            if (u1,v1) != (u,v):
                cycle_sum -= e_var
            else:
                cycle_sum += e_var

            cycle_scale_sum += scale_var
        
        constraints += [-H@cycle_sum <= 2*cycle_scale_sum/np.sqrt(2) * b]

    # define cost
    cost = 0.
    C_norm = 0.1
    epsilon = 0.01
    for edge in graph.edge_list:
        i,j = edge
        edge_vars_pair = graph.edges[i][j]
        e_var, scale_var,_ = edge_vars_pair

        e_var         = edge_vars_pair[0]
        scale_var     = edge_vars_pair[1]
        u1,v1         = edge_vars_pair[2]
        current_relative_position = agents_position[v1-1] - agents_position[u1-1]

        cost += -BOX_WEIGHT *scale_var 
        cost += C_norm * cp.norm(e_var - current_relative_position[:dim])


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
            
            if (u,v) in added_edges or (v,u) in added_edges:
                continue # do not add the task twice
            

            edge_vars_pair = graph.edges[u][v]
            
            e_var         = edge_vars_pair[0]
            scale_var     = edge_vars_pair[1]
            u1,v1         = edge_vars_pair[2]

            e_value     = e_var.value
            scale_value = scale_var.value

            if (u,v) != (u1,v1) : # invert direction if you traverse the edge backwards
                e_value = -e_value
            
            new_task = Task(
                edges        = [u,v],
                rel_position = e_value.flatten(),
                size         = 2*scale_value/np.sqrt(2.),
                timespan     = task.timespan,
                period_num   = task.period_num,
                operator     = task.operator,
            )
            
            new_tasks.append(new_task)
            added_edges.append((u,v))
        
    return prob.status, new_tasks   