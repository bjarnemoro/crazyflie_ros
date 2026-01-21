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


class MPCsolver:
    def __init__(self):
        self.solver = cp.CLARABEL
        self.initialized = False

    def initialize_problem(self, horizon, t0, squares: list[HyperCubeHandler], agents: list[int], dt, MAX_INPUT):
        self.initialized = True
        DIM = len(squares[0].compute_offset_vector(0))//2

        self.state_size = len(agents)*DIM
        self.input_size = len(agents)*DIM
        
        self.u = cp.Variable((self.input_size, horizon-1))
        self.slack = cp.Variable((self.input_size, horizon-1))
        self.x = cp.Variable((self.state_size, horizon))

        num = (horizon-2)*len(squares)*4
        self.b_pos = cp.Parameter((num))
        self.x0 = cp.Parameter((self.state_size))

        A = np.zeros((self.state_size, self.state_size))
        B = np.eye(self.state_size)

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

        if b_pos:
            A_big = np.zeros((num, self.state_size*horizon))
            
            for n, (i, j, k) in enumerate(zip(actual_i, actual_j, ks)):
                A_big[2*DIM*n, self.state_size*k + DIM*j] = 1
                A_big[2*DIM*n, self.state_size*k + DIM*i] = -1
                A_big[2*DIM*n+1, self.state_size*k + DIM*j] = -1
                A_big[2*DIM*n+1, self.state_size*k + DIM*i] = 1
                A_big[2*DIM*n+2, self.state_size*k + DIM*j+1] = 1
                A_big[2*DIM*n+2, self.state_size*k + DIM*i+1] = -1
                A_big[2*DIM*n+3, self.state_size*k + DIM*j+1] = -1
                A_big[2*DIM*n+3, self.state_size*k + DIM*i+1] = 1

                if DIM == 3:
                    A_big[2*DIM*n+4, self.state_size*k + DIM*j+2] = 1
                    A_big[2*DIM*n+4, self.state_size*k + DIM*i+2] = -1
                    A_big[2*DIM*n+5, self.state_size*k + DIM*j+2] = -1
                    A_big[2*DIM*n+5, self.state_size*k + DIM*i+2] = 1

            constraints = [self.x[:,0] == self.x0]
            constraints += [self.x[:,1:] == A_d @ self.x[:,:-1] + B_d @ (self.u+self.slack)]
            constraints +=  [self.u <= MAX_INPUT]
            constraints +=  [-self.u <= MAX_INPUT]
            constraints += [A_big @ cp.vec(self.x.T, order="C") <= self.b_pos]

            objective = cp.Minimize(cp.sum_squares(self.u) + 1e3*cp.sum_squares(self.slack)+cp.sum_squares(cp.vec(self.u[:,1:], order="C")-cp.vec(self.u[:,:-1], order="C")))
            self.prob = cp.Problem(objective, constraints)

        
    def recompute(self, horizon, x0, t0, squares: list[HyperCubeHandler], dt, return_val):
        assert self.initialized

        DIM = len(squares[0].compute_offset_vector(0))//2
        b_pos = []

        for k in range(1, horizon-1):
            t = k*dt + t0
            for square in squares:
                if square.time_grid[0] <= t < square.time_grid[1]:
                    b_vec = square.compute_offset_vector(t)
                    b_pos.append(b_vec)
                else:
                    b_pos.append(square.compute_offset_vector(square.time_grid[1]-0.01)+np.ones(DIM*2)*0.02)

        if b_pos:
            b_stack = np.hstack(b_pos)

            self.b_pos.value = b_stack
            self.x0.value = x0

            self.prob.solve(solver=self.solver, warm_start=True)

            if return_val == "pos":
                return self.x.value[:self.state_size].T
            if return_val == "x_next":
                return self.x.value[:,1]
            elif return_val == "u0":
                return self.u.value[:,0]



def main():
    pass


if __name__ == "__main__":
    main()