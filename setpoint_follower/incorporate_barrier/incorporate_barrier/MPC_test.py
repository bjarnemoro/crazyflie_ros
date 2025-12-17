from xml.sax.handler import DTDHandler
import cdd
import time
import numpy as np
import cvxpy as cp
from scipy.signal import cont2discrete
import matplotlib.pyplot as plt
import matplotlib.animation as animation

SIZE_GROWTH = 5
METHOD = 0

def flatten(x):
    return [item for e in x for item in e]

class Polytope:
    def __init__(self, v_description, timespan, init_time, pos, size):
        self.__v_description = v_description
        self.timespan: tuple[float, float] = timespan
        self.init_time: float = init_time
        self.pos = pos
        self.__size = size

    def size(self, time):
        start_boost = self.timespan[0]-self.init_time
        if time >= start_boost and time < self.timespan[0]:
            size_boost = SIZE_GROWTH * (self.timespan[0]-time)
        else:
            size_boost = 0

        return self.__size+size_boost


    def vertices(self, time):
        mat = np.hstack([self.b.reshape(-1, 1) * self.size(time), -self.A])
        self.cdd_mat = cdd.matrix_from_array(mat, rep_type=cdd.RepType.INEQUALITY)
        self.poly    = cdd.polyhedron_from_matrix(self.cdd_mat)

        generators_lib = cdd.copy_generators(self.poly)
        generators     = np.array(generators_lib.array)
        linearities    = np.array(list(generators_lib.lin_set)) # It tells which rays are allowed to be also negative (https://people.inf.ethz.ch/fukudak/cdd_home/cddlibman2021.pdf) pp. 4
        
        if not len(generators) :
            raise ValueError("The polytope is empty. No vertices or rays found.")
        vertices_indices = generators[:,0] == 1. # vertices are only the generators with 1 in the first column (https://people.inf.ethz.ch/fukudak/cdd_home/cddlibman2021.pdf) pp. 4
        
        vertices      = generators[vertices_indices, 1:]
        vertices[:,0] += self.pos[0]
        vertices[:,1] += self.pos[1]
        
        return vertices

class Square(Polytope):
    def __init__(self, size, pos, timespan, init_time):
        self.A = np.array([
            [1,0],
            [-1,0],
            [0,1],
            [0,-1]])
        
        self.b = np.array([[1, 1, 1, 1]]).T

        super().__init__(self.A, timespan, init_time, pos, size)

        mat = np.hstack([self.b.reshape(-1, 1) * size, -self.A])
        self.cdd_mat = cdd.matrix_from_array(mat, rep_type=cdd.RepType.INEQUALITY)
        self.poly    = cdd.polyhedron_from_matrix(self.cdd_mat)

def poly_lines(vertices):
    return flatten([[[vertices[i-1][0], vertices[i][0]],[vertices[i-1][1], vertices[i][1]]] for i in range(len(vertices))])

def poly_lines_ax(vertices):
    return flatten([[[vertices[i-1][0], vertices[i][0]],[vertices[i-1][1], vertices[i][1]]] for i in range(len(vertices))])

def boundarie_constraint(squares, t, x, x_list, b_pos, c_list):
    active_square = []
    for square in squares:
        if square.timespan[0]-square.init_time <= t < square.timespan[1]:
            active_square.append(square)

    const = []
    for square in active_square:
        A = square.A
        b = square.b * square.size(t)

        c = np.array([[square.pos[0]], [square.pos[1]]])
        c_old = np.array(square.pos)

        x_list.append(x-c)
        b_pos.append(b)
        c_list.append(c)

        const += [A @ (x - c_old) <= b]

    return const


def optimize_path(horizon, squares):
    u = cp.Variable((2, horizon-1))
    x = cp.Variable((4, horizon))

    x0 = np.array([0, 0, 0, 0])

    A = np.array([
        [0, 0, 1, 0],
        [0, 0, 0, 1],
        [0, 0, 0, 0],
        [0, 0, 0, 0]])
    
    B = np.array([
        [0, 0],
        [0, 0],
        [1, 0],
        [0, 1]])
    
    dt = 0.1

    system_discrete = cont2discrete((A, B, np.eye(4), np.zeros((4, 2))), dt)
    A_d, B_d, _, _, _ = system_discrete

    x_list = []
    b_pos = []
    c_list = []

    num_const = 0
    constraints = [x[:,0] == x0]
    constraints += [x[:,1:] == A_d @ x[:,:-1] + B_d @ u]

    for k in range(horizon-1):
        t = k*dt 
        for square in squares:
            if square.timespan[0]-square.init_time <= t < square.timespan[1]:
                num_const += 1
        #constraints += [x[:,k+1] == A_d @ x[:,k] + B_d @ u[:,k]]
        constraint = boundarie_constraint(squares, k*dt, x[:2,k], x_list, b_pos, c_list)
        if METHOD == 1:
            constraints += constraint

    C = np.zeros((horizon, num_const))

    c = 0
    for k in range(horizon-1):
        t = k*dt 
        for square in squares:
            if square.timespan[0]-square.init_time <= t < square.timespan[1]:
                C[k][c] = 1
                c += 1
    
    if METHOD == 0:
        A_s = squares[0].A
        b_stack = cp.hstack(b_pos)
        c_stack = cp.hstack(c_list)
        
        constraints += [A_s @ (x[:2]@C-c_stack) <= b_stack]

    objective = cp.Minimize(cp.norm(u))
    prob = cp.Problem(objective, constraints)

    start_time = time.perf_counter()

    result = prob.solve(solver=cp.CLARABEL)

    print("compute time: {}s".format(time.perf_counter()-start_time))

    return np.array(x.value)[:2].T
    


def main():
    squares = [
        Square(5, (4,0), (0, 1.5), 0),
        Square(5, (12,10), (2, 3.5), 2),
        Square(5, (0,20), (4.5, 5.5), 2),
        Square(5, (-20, 30), (8, 10), 3)]
    
    fig, ax = plt.subplots()
    active_square = []
    start_time = time.perf_counter()
    
    lines = []

    HORIZON = 100
    pos_vals = optimize_path(HORIZON, squares)

    def init():
        #ax.clear()
        ax.set_xlim(-40, 40)  # Set as per your data range
        ax.set_ylim(-10, 60)
        ax.plot(lines)
        global scatter_point
        scatter_point = ax.scatter([], [], color='blue', s=50)
        return lines + [scatter_point]

    def update(frame):
        current_time = time.perf_counter()
        actual_time = current_time - start_time

        actual_time = frame*0.1

        # Clear previous lines
        for line in lines:
            line.remove()
        lines.clear()

        # Determine active squares
        active_square.clear()
        for square in squares:
            if square.timespan[0]-square.init_time <= actual_time < square.timespan[1]:
                active_square.append(square)

        x_vals = []
        y_vals = []
        for square in active_square:
            x_vals.clear()
            y_vals.clear()
            for x, y in square.vertices(actual_time):
                x_vals.append(x)
                y_vals.append(y)
            x_vals.append(x_vals[0])
            y_vals.append(y_vals[0])

            line, = ax.plot(x_vals, y_vals, color='red')
            lines.append(line)

        scatter_point.set_offsets([pos_vals[frame%(HORIZON-1)]])


        return lines + [scatter_point]
        

    ani = animation.FuncAnimation(
        fig, update, frames=HORIZON, init_func=init, blit=True, interval=200, repeat=False
    )
    #ani.save(filename="MPC_demo2.gif", writer="ffmpeg")
    plt.show()

    
if __name__ == "__main__":
    main()