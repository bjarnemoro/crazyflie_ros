import time
import cvxpy as cp
import numpy as np
from scipy.signal import cont2discrete
from scipy.sparse import csc_matrix
from barrier_msg.msg import Config

from incorporate_barrier.bMsg import bMsg, HyperCubeHandler
#from bMsg import bMsg, HyperCubeHandler

A_s = np.array(
    [[1,0],
    [-1,0],
    [0,1],
    [0,-1]])

A_s = csc_matrix(A_s)

b_s = np.array([[1, 1, 1, 1]]).T


def optimize_path(horizon, x0, t0, squares: list[HyperCubeHandler], agents: list[int], dt, return_val):
    assert type(x0) == np.ndarray

    DIM = len(squares[0].compute_offset_vector(0))//2

    state_size = len(agents)*2*DIM
    input_size = len(agents)*DIM

    u = cp.Variable((input_size, horizon-1))
    x = cp.Variable((state_size, horizon))

    

    #x0 = np.array([0, 0, 5, 10, 15, 10, 0, 0, 0, 0, 0, 0])
    #x0 = np.zeros((state_size))

    A = np.zeros((state_size, state_size))
    for i in range(input_size):
        A[i,i+input_size] = 1
    B = np.zeros((state_size, input_size))
    for i in range(input_size):
        B[i+input_size,i] = 1

    system_discrete = cont2discrete((A, B, np.eye(4), np.zeros((4, 2))), dt)
    A_d, B_d, _, _, _ = system_discrete

    A_d = csc_matrix(A_d)
    B_d = csc_matrix(B_d)

    b_pos = []
    actual_i = []
    actual_j = []
    ks = []
    N = 0
    for k in range(1, horizon-1):
        t = k*dt + t0
        for square in squares:
            if square.time_grid[0] <= t < square.time_grid[1]:
                N += 1

                b_vec = square.compute_offset_vector(t)#.reshape(4, 1)
                b_pos.append(b_vec)

                actual_i.append(agents.index(square.edge_i))
                actual_j.append(agents.index(square.edge_j))
                ks.append(k)
    
    b_stack = cp.hstack(b_pos)

    A_big = np.zeros((2*DIM*N, state_size*horizon))
    
    for n, (i, j, k) in enumerate(zip(actual_i, actual_j, ks)):
        A_big[2*DIM*n, state_size*k + DIM*j] = 1
        A_big[2*DIM*n, state_size*k + DIM*i] = -1
        A_big[2*DIM*n+1, state_size*k + DIM*j] = -1
        A_big[2*DIM*n+1, state_size*k + DIM*i] = 1
        A_big[2*DIM*n+2, state_size*k + DIM*j+1] = 1
        A_big[2*DIM*n+2, state_size*k + DIM*i+1] = -1
        A_big[2*DIM*n+3, state_size*k + DIM*j+1] = -1
        A_big[2*DIM*n+3, state_size*k + DIM*i+1] = 1

        if DIM == 3:
            A_big[2*DIM*n+4, state_size*k + DIM*j+2] = 1
            A_big[2*DIM*n+4, state_size*k + DIM*i+2] = -1
            A_big[2*DIM*n+5, state_size*k + DIM*j+2] = -1
            A_big[2*DIM*n+5, state_size*k + DIM*i+2] = 1


    constraints = [x[:,0] == x0]
    constraints += [x[:,1:] == A_d @ x[:,:-1] + B_d @ u]

    constraints += [A_big @ cp.vec(x.T, order="C") <= b_stack]

    #objective = cp.Minimize(cp.norm(u)) not OSQP compliant
    objective = cp.Minimize(cp.sum_squares(u))
    prob = cp.Problem(objective, constraints)

    start_time = time.perf_counter()

    #result = prob.solve(solver=cp.OSQP, max_iter=100000)
    result = prob.solve(solver=cp.CLARABEL, warm_start=True)

    print(result)
    print("compute time: {}s".format(time.perf_counter()-start_time))

    if return_val == "pos":
        return x.value[:len(agents)*DIM].T
    if return_val == "x0":
        return x.value[:,1]
    elif return_val == "input":
        return u.value[0]

#chatGPT
# def build_prediction_matrices(A, B, horizon):
#     n_x, n_u = B.shape
#     Phi_x = np.zeros((n_x*horizon, n_x))
#     Phi_u = np.zeros((n_x*horizon, n_u*(horizon-1)))

#     for k in range(horizon):
#         Phi_x[k*n_x:(k+1)*n_x] = A**k

#         for j in range(k):
#             Phi_u[k*n_x:(k+1)*n_x, j*n_u:(j+1)*n_u] = A**(k-1-j) @ B

#     return Phi_x, Phi_u

# def optimize_path(horizon, squares: list[HyperCubeHandler], agents: list[int], dt):
#     u = cp.Variable((len(agents)*2 * (horizon-1), 1))
#     #x = cp.Variable((len(agents)*4 * horizon, 1))

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

    

#     Phi_x, Phi_u = build_prediction_matrices(A_d, B_d, horizon)
#     X = Phi_x @ x0 + Phi_u @ u
#     print(X.shape)

#     #A_d = csc_matrix(A_d)
#     #B_d = csc_matrix(B_d)

#     b_pos = []

#     constraints = []
#     #constraints = [x[:,0] == x0]
#     #constraints += [x[:,1:] == A_d @ x[:,:-1] + B_d @ u]
    
#     actual_i = []
#     actual_j = []
#     ks = []
#     N = 0
#     for k in range(horizon-1):
#         t = k*dt 
#         for square in squares:
#             if square.time_grid[0] <= t < square.time_grid[1]:
#                 N += 1
#                 idx_i = agents.index(square.edge_i)
#                 idx_j = agents.index(square.edge_j)

#                 b_vec = square.compute_offset_vector(t)#.reshape(4, 1)
                
#                 b_pos.append(b_vec)

#                 actual_i.append(idx_i)
#                 actual_j.append(idx_j)
#                 ks.append(k)

#                 #constraints += [A_s @ x_vec <= b_vec]
    
#     b_stack = cp.hstack(b_pos)

#     A_big = np.zeros((4*N, len(agents)*horizon*4))
#     l_state = len(agents)*4
    
#     for n, (i, j, k) in enumerate(zip(actual_i, actual_j, ks)):
#         A_big[4*n, l_state*k + 2*j] = 1
#         A_big[4*n, l_state*k + 2*i] = -1
#         A_big[4*n+1, l_state*k + 2*j] = -1
#         A_big[4*n+1, l_state*k + 2*i] = 1
#         A_big[4*n+2, l_state*k + 2*j+1] = 1
#         A_big[4*n+2, l_state*k + 2*i+1] = -1
#         A_big[4*n+3, l_state*k + 2*j+1] = -1
#         A_big[4*n+3, l_state*k + 2*i+1] = 1

#     #constraints += [A_big @ cp.vec(x.T, order="C") <= b_stack]
#     A_big = csc_matrix(A_big)
#     constraints += [A_big @ X <= b_stack]

#     #objective = cp.Minimize(cp.norm(u)) not OSQP compliant
#     objective = cp.Minimize(cp.sum_squares(u))
#     prob = cp.Problem(objective, constraints)

#     start_time = time.perf_counter()

#     #result = prob.solve(solver=cp.OSQP, max_iter=100000)
#     result = prob.solve(solver=cp.OSQP, max_iter=100000, verbose=True)

#     print(result)
#     print("compute time: {}s".format(time.perf_counter()-start_time))

#     return x.value[:len(agents)*2].T

def main():
    horizon = 10 
    squares = None
    agents = [1, 4, 6] 

    optimize_path(horizon, squares, agents)


if __name__ == "__main__":
    main()