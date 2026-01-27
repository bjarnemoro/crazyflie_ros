import numpy as np


class bMsg:
    """
    Barrier message class holding parameters for a barrier function task. 
    This class it is just to show how you can use the content of a message to obtain the 
    the constraint for an MPC controller.
    """

    def __init__(self,
                 slopes:   list[float],
                 gamma0:   list[float],
                 r:        float,
                 slack:    float,
                 b_vector: list[float],
                 time_grid:list[float],
                 task_id:  int,
                 edge_i:   int,
                 edge_j:   int):
        
        self.slopes    = np.array(slopes)      # Slopes of the gamma function
        self.gamma0    = np.array(gamma0)      # Gamma0 values
        self.r         = r                     # Robustness variable
        self.slack     = slack                 # Slack violation for barrier (positive if can't be fully followed)
        self.b_vector  = np.array(b_vector)    # b vector for Ax <= b (with b = b' + Ac)
        self.time_grid = np.array(time_grid)   # Time grid for evaluating the barrier function
        self.task_id   = task_id               # Task ID
        self.edge_i    = edge_i                # Assigned to edge e_{ij} = x_i - x_j (i)
        self.edge_j    = edge_j                # Assigned to edge e_{ij} = x_i - x_j (j)

        if len(gamma0) == 4 :
            self.slopes = self.slopes.reshape(4,-1)
        elif len(gamma0) == 6 :
            self.slopes = self.slopes.reshape(6,-1)

        self.delta_t = np.diff(self.time_grid)  # Time differences for the time grid


    def __repr__(self):
        return (f"bMsg(task_id={self.task_id}, edge=({self.edge_i}, {self.edge_j}), "
                f"r={self.r}, slack={self.slack}, slopes={self.slopes}, gamma0={self.gamma0}, "
                f"b_vector={self.b_vector}, time_grid={self.time_grid})")
    
    def compute_offset_vector(self,t :float):

        """
        minimal example for computing the offset vector b + g(t) of the experssion Ax<= b + g(t) representing a time varying constraint
        """

        if t < self.time_grid[0] or t > self.time_grid[-1]:
            # we can change this from giving an error to giving a value of infinity for example
            raise ValueError("Time t is out of bounds of the time grid.")

        tt            = np.zeros(self.slopes.shape[1])
        idx           = np.searchsorted(self.time_grid, t, side='left')-1
        
        if idx < 0:
            tt[0] = t-self.time_grid[0]

        else:
            for ii in range(idx):
                tt[ii] = self.delta_t[ii]

            tt[idx] = t-self.time_grid[idx]


        return self.slopes @ tt + self.gamma0 + self.b_vector #-> this is what you willl put in the MPC as Ax <= (self.slopes @ tt + self.gamma0 + self.b_vector)


class HyperCubeHandler(bMsg):
    def __init__(self,
                 slopes:   list[float],
                 gamma0:   list[float],
                 r:        float,
                 slack:    float,
                 b_vector: list[float],
                 time_grid:list[float],
                 task_id:  int,
                 edge_i:   int,
                 edge_j:   int):

        super().__init__(slopes, gamma0, r, slack, b_vector, time_grid, task_id, edge_i, edge_j)
        self.neighbour = None
        

    def add_neighbour(self, neighbour):
        self.neighbour = neighbour

    def get_offset(self, t):
        if self.neighbour is not None:
            offset = self.neighbour.get_offset(t)
            return self.compute_offset_vector(t) + offset
        else:
            return self.compute_offset_vector(t)
        
    def get_offset_trans(self, t, pos):
        if self.neighbour is not None:
            offset = self.neighbour.get_offset_trans(t, pos)
            return self.compute_offset_vector(t) + offset
        else:
            if len(pos) == 2:
                trans = np.array([pos[0], -pos[0], pos[1], -pos[1], pos[2], -pos[2]])
            elif len(pos) == 3:
                trans = np.array([pos[0], -pos[0], pos[1], -pos[1], pos[2], -pos[2]])
            return self.compute_offset_vector(t) + trans

    def rel_offset_trans(self, t, pos):
        if len(pos) == 2:
            trans = np.array([pos[0], -pos[0], pos[1], -pos[1], pos[2], -pos[2]])
        elif len(pos) == 3:
            trans = np.array([pos[0], -pos[0], pos[1], -pos[1], pos[2], -pos[2]])
        return self.compute_offset_vector(t) + trans