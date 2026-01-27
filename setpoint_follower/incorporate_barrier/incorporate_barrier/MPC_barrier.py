import time
import cvxpy as cp
import numpy as np
from scipy.signal import cont2discrete
from scipy.sparse import csc_matrix

from barrier_msg.msg import Config
from incorporate_barrier.bMsg import bMsg, HyperCubeHandler


class MPCsolver:
    def __init__(
        self,
        num_agents: int,
        agents_dim: int,
        dt: float,
        horizon: int,
        max_input: float,
        communication_distance: float,
    ):
        self.solver      = cp.CLARABEL
        self.initialized = False
        self.dt          = dt
        self.horizon     = horizon
        self.max_input   = max_input
        self.communication_distance = communication_distance

        self.states_dim      = agents_dim
        self.inputs_dim      = agents_dim
        self.num_agents      = num_agents
        self.total_state_dim = self.states_dim * self.num_agents

        self.squares = []
        self.num_tv = 0

        # Single-integrator dynamics
        A = np.zeros((self.states_dim, self.states_dim))
        B = np.eye(self.inputs_dim)
        C = np.eye(self.states_dim)
        D = np.zeros((self.states_dim, self.inputs_dim))

        system_discrete = cont2discrete((A, B, C, D), dt)
        A_d, B_d, _, _, _ = system_discrete

        self.A_d = csc_matrix(A_d)
        self.B_d = csc_matrix(B_d)

    def agent_index(self, agent_id: int):
        return agent_id * self.states_dim

    def initialize_problem(self, squares: list[HyperCubeHandler], logger=None):
        """
        Each hypercube represents a constraint of the form
        H (x_j - x_i) <= b(t)
        """

        if len(squares) == 0:
            raise ValueError("No time varying constraints provided to MPCsolver")

        self.initialized = True
        self.squares = squares
        self.num_tv = len(squares)

        rows_per_constraint = 2 * self.states_dim
        nx = rows_per_constraint * self.num_tv

        # Decision variables
        self.u = cp.Variable((self.inputs_dim * self.num_agents, self.horizon))
        self.x = cp.Variable((self.total_state_dim, self.horizon + 1))
        self.x0 = cp.Parameter(self.total_state_dim)

        # Time-varying constraint data
        self.task_slack = cp.Variable((nx, self.horizon + 1), nonneg=True)
        self.comm_slack = cp.Variable((self.horizon + 1), nonneg=True)
        self.b_offset = cp.Parameter((nx, self.horizon + 1))

        constraints = []

        # Dynamics + input bounds
        for k in range(self.horizon):
            for i in range(self.num_agents):
                idx = self.agent_index(i)

                x_k    = self.x[idx : idx + self.states_dim, k]
                x_next = self.x[idx : idx + self.states_dim, k + 1]
                u_k    = self.u[idx : idx + self.inputs_dim, k]

                constraints += [
                    x_next == self.A_d @ x_k + self.B_d @ u_k,
                    u_k <= self.max_input,
                    -u_k <= self.max_input,
                ]

        # Initial condition
        constraints += [self.x[:, 0] == self.x0]
        H = np.array([[1., 0], 
                      [-1., 0], 
                      [0, 1.], 
                      [0, -1.]])

        # Time-varying relative constraints
        logger.info(f"Number of time-varying constraints: {len(self.squares)}")
        for k in range(self.horizon + 1):
            row = 0

            for qq,square in enumerate(self.squares):
                agent_i = square.edge_i - 1
                agent_j = square.edge_j - 1
                
                C = relative_matrix(self.states_dim,self.num_agents,agent_j, agent_i)

                A_block = H @ C
                
                constraints += [
                    A_block @ self.x[:, k] <= self.b_offset[qq*rows_per_constraint :qq*rows_per_constraint + rows_per_constraint, k] + self.task_slack[qq*rows_per_constraint :qq*rows_per_constraint + rows_per_constraint, k]
                ]

                # add communication distance constraint
                constraints += [ cp.sum_squares(C @ self.x[:, k]) <= self.communication_distance**2 + self.comm_slack[k] ]

        # Objective
        objective = cp.Minimize(1e3 * cp.sum_squares(self.task_slack) +  cp.sum_squares(self.u) + 1e3 * cp.sum_squares(self.comm_slack))

        self.prob = cp.Problem(objective, constraints)

    def solve(self, x0, t0, return_val="u0",logger=None):
        if not self.initialized:
            raise RuntimeError("MPCsolver not initialized")
        
        b_mat = []
        for k in range(self.horizon + 1):
            t = k * self.dt + t0
            b_vec = []

            for square in self.squares:
                if square.time_grid[0] <= t < square.time_grid[1]:
                    b_vec.append(square.compute_offset_vector(t).reshape(-1,1))
                else:
                    b_vec.append(np.ones((2*self.states_dim,1)) * 1e4)  # Large value to effectively disable the constraint
            
            b_mat.append(np.vstack(b_vec))

        self.b_offset.value = np.hstack(b_mat)
        # if logger is not None:
        #     logger.info(f"b_offset value:\n{np.hstack(b_mat)}")

        self.x0.value = x0

        self.prob.solve(solver=self.solver, warm_start=True)

        if return_val == "x_next":
            return self.x.value[:, 1]
        elif return_val == "u0":
            return self.u.value[:, 0]
        else:
            raise ValueError("return_val must be 'x_next' or 'u0'")

    def __call__(self, x0, t0, return_val="u0",logger=None):
        res = self.solve(x0, t0, return_val,logger)
        # if logger is not None:
        #     logger.info(f"slack value:\n{self.task_slack.value}")
        return res


def relative_matrix(agent_dim, num_agents, index_j, index_i):

    """
    Returns matrix C such that C @ x = x_j - x_i
    where x is the stacked state of all agents.
    """
    C = np.zeros((agent_dim, agent_dim * num_agents))

    j_start = index_j * agent_dim
    i_start = index_i * agent_dim

    C[:, j_start:j_start + agent_dim] = np.eye(agent_dim)
    C[:, i_start:i_start + agent_dim] = -np.eye(agent_dim)

    return C