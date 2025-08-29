import time
import cvxpy as cp
import numpy as np
from scipy.signal import cont2discrete
from scipy.sparse import csc_matrix

from incorporate_barrier.bMsg import bMsg, HyperCubeHandler
#from bMsg import bMsg, HyperCubeHandler

A_s = np.array(
    [[1,0],
    [-1,0],
    [0,1],
    [0,-1]])

A_s = csc_matrix(A_s)

b_s = np.array([[1, 1, 1, 1]]).T

# def boundaries(squares, t, b_pos, c_list):
#     active_square = []
#     for square in squares:
#         if square.timespan[0]-square.init_time <= t < square.timespan[1]:
#             active_square.append(square)

#     for square in active_square:
#         b = square.b * square.size(t)

#         c = np.array([[square.pos[0]], [square.pos[1]]])

#         b_pos.append(b)
#         c_list.append(c)

def new_boundary(self):
    pass
        
# def optimize_path_old(horizon, squares: list[HyperCubeHandler], agents: list[int], dt):
#     u = cp.Variable((len(agents)*2, horizon-1))
#     x = cp.Variable((len(agents)*4, horizon))

#     #x0 = np.array([0, 0, 5, 10, 15, 10, 0, 0, 0, 0, 0, 0])
#     x0 = np.zeros((len(agents)*4))

#     A = np.zeros((len(agents)*4, len(agents)*4))
#     for i in range(len(agents)*2):
#         A[i,i+len(agents)*2] = 1
#     B = np.zeros((len(agents)*4, len(agents)*2))
#     for i in range(len(agents)*2):
#         B[i+len(agents)*2,i] = 1

#     system_discrete = cont2discrete((A, B, np.eye(4), np.zeros((4, 2))), dt)
#     A_d, B_d, _, _, _ = system_discrete

#     b_pos = []
#     c_list = []
#     x_select = []

#     num_const = 0
#     constraints = [x[:,0] == x0]
#     constraints += [x[:,1:] == A_d @ x[:,:-1] + B_d @ u]
    
#     b = b_s*20

#     C = np.zeros((horizon, num_const))

#     c = 0

#     for k in range(horizon-1):
#         t = k*dt 
#         for square in squares:
#             if square.time_grid[0] <= t < square.time_grid[1]:
#                 num_const += 1

#                 S_i = np.zeros((len(agents)*4, 2))
#                 S_j = np.zeros((len(agents)*4, 2))
#                 idx_i = agents.index(square.edge_j)
#                 idx_j = agents.index(square.edge_i)
#                 S_i[idx_i*2,0] = 1
#                 S_i[idx_i*2+1,1] = 1
#                 S_j[idx_j*2,0] = 1
#                 S_j[idx_j*2+1,1] = 1

#                 b_pos.append(square.compute_offset_vector(t))
#                 x_select.append(x[:,k] @ S_i - x[:,k] @ S_j)
        
#         #boundaries(squares, k*dt, b_pos, c_list)

#     b_stack = cp.vstack(b_pos).T
#     x_select = cp.vstack(x_select).T
        
#     constraints += [A_s @ (x_select) <= b_stack]

#     objective = cp.Minimize(cp.norm(u))
#     prob = cp.Problem(objective, constraints)

#     start_time = time.perf_counter()

#     result = prob.solve(solver=cp.OSQP)

#     print(result)
#     print("compute time: {}s".format(time.perf_counter()-start_time))

#     return x.value[:len(agents)*2].T


def optimize_path(horizon, squares: list[HyperCubeHandler], agents: list[int], dt):
    u = cp.Variable((len(agents)*2, horizon-1))
    x = cp.Variable((len(agents)*4, horizon))

    #x0 = np.array([0, 0, 5, 10, 15, 10, 0, 0, 0, 0, 0, 0])
    x0 = np.zeros((len(agents)*4))

    A = np.zeros((len(agents)*4, len(agents)*4))
    for i in range(len(agents)*2):
        A[i,i+len(agents)*2] = 1
    B = np.zeros((len(agents)*4, len(agents)*2))
    for i in range(len(agents)*2):
        B[i+len(agents)*2,i] = 1

    system_discrete = cont2discrete((A, B, np.eye(4), np.zeros((4, 2))), dt)
    A_d, B_d, _, _, _ = system_discrete

    A_d = csc_matrix(A_d)
    B_d = csc_matrix(B_d)

    b_pos = []
    x_select = []

    constraints = [x[:,0] == x0]
    constraints += [x[:,1:] == A_d @ x[:,:-1] + B_d @ u]

    idx_i_list = []
    idx_j_list = []
    actual_i = []
    actual_j = []
    ks = []
    N = 0
    for k in range(horizon-1):
        t = k*dt 
        for square in squares:
            if square.time_grid[0] <= t < square.time_grid[1]:
                N += 1
                idx_i = agents.index(square.edge_i)
                idx_j = agents.index(square.edge_j)

                b_vec = square.compute_offset_vector(t)#.reshape(4, 1)
                x_vec = cp.reshape(x[idx_i*2:idx_i*2+2,k] - x[idx_j*2:idx_j*2+2,k], (2, 1), order='C')
                
                b_pos.append(b_vec)
                x_select.append(x_vec)

                idx_i_list.append([idx_i*2,idx_i*2+1])
                idx_j_list.append([idx_j*2,idx_j*2+1])

                actual_i.append(idx_i)
                actual_j.append(idx_j)
                ks.append(k)

                #constraints += [A_s @ x_vec <= b_vec]
    
    #x_select = cp.hstack(x_select)
    b_stack = cp.hstack(b_pos)
        
    #constraints += [A_s @ x_select <= b_stack]

    idx_i_list = np.array(idx_i_list).T
    idx_j_list = np.array(idx_j_list).T

    A_big = np.zeros((4*N, len(agents)*horizon*4))
    l_state = len(agents)*4
    
    for n, (i, j, k) in enumerate(zip(actual_i, actual_j, ks)):
        A_big[4*n, l_state*k + 2*j] = 1
        A_big[4*n, l_state*k + 2*i] = -1
        A_big[4*n+1, l_state*k + 2*j] = -1
        A_big[4*n+1, l_state*k + 2*i] = 1
        A_big[4*n+2, l_state*k + 2*j+1] = 1
        A_big[4*n+2, l_state*k + 2*i+1] = -1
        A_big[4*n+3, l_state*k + 2*j+1] = -1
        A_big[4*n+3, l_state*k + 2*i+1] = 1
    
    #constraints += [A_s @ (x[idx_j_list, ks]-x[idx_i_list, ks]) <= b_stack]

    constraints += [A_big @ cp.vec(x.T, order="C") <= b_stack]

    #objective = cp.Minimize(cp.norm(u)) not OSQP compliant
    objective = cp.Minimize(cp.sum_squares(u))
    prob = cp.Problem(objective, constraints)

    start_time = time.perf_counter()

    #result = prob.solve(solver=cp.OSQP, max_iter=100000)
    result = prob.solve(solver=cp.CLARABEL, max_iter=100000)

    print(result)
    print("compute time: {}s".format(time.perf_counter()-start_time))

    return x.value[:len(agents)*2].T

def main():
    horizon = 10 
    squares = None
    agents = [1, 4, 6] 

    optimize_path(horizon, squares, agents)


if __name__ == "__main__":
    main()