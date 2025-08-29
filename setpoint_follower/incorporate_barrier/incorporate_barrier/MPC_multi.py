import time
import numpy as np
import cvxpy as cp
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy.signal import cont2discrete

from MPC_test import Square
from MPC_test import boundarie_constraint

HORIZON = 50
METHOD = 0

def boundaries(squares, t, b_pos, c_list):
    active_square = []
    for square in squares:
        if square.timespan[0]-square.init_time <= t < square.timespan[1]:
            active_square.append(square)

    for square in active_square:
        b = square.b * square.size(t)

        c = np.array([[square.pos[0]], [square.pos[1]]])

        b_pos.append(b)
        c_list.append(c)

def optimize_path(horizon, squares, num_agents):
    u = cp.Variable((6, horizon-1))
    x = cp.Variable((12, horizon))

    x0 = np.array([0, 0, 5, 10, 15, 10, 0, 0, 0, 0, 0, 0])
    A = np.array([
        [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])
    
    B = np.array([
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [1, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
        [0, 0, 0, 1, 0, 0],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 1]])
    
    dt = 0.1

    system_discrete = cont2discrete((A, B, np.eye(4), np.zeros((4, 2))), dt)
    A_d, B_d, _, _, _ = system_discrete

    b_pos = []
    c_list = []

    num_const = 0
    constraints = [x[:,0] == x0]
    constraints += [x[:,1:] == A_d @ x[:,:-1] + B_d @ u]

    #t = cp.Variable()
    #constraints += [t>=3]
#
    #for k in range(HORIZON):
    #    constraints += [cp.sum_squares(x[0:2, k] - x[2:4, k]) >= 9.0]
    #    constraints += [cp.sum_squares(x[0:2, k] - x[4:6, k]) >= 9.0]
    #    constraints += [cp.sum_squares(x[2:4, k] - x[4:6, k]) >= 9.0]
    collision_cost = 0
    A_s = squares[0].A
    b = squares[0].b*20
    print(b)
    for k in range(HORIZON):
        collision_cost += 5-cp.norm(x[0:2, k] - x[2:4, k], 1)+0.01
        #constraints += [x[0, k] - x[2, k] = 4]
        
        #constraints += [A_s @ (x[0:2, k] - x[2:4, k]) <= b]
        #constraints += [cp.norm(x[0:2, k] - x[2:4, k]) >= 5]

    for k in range(horizon-1):
        t = k*dt 
        for square in squares:
            if square.timespan[0]-square.init_time <= t < square.timespan[1]:
                num_const += 1

        
        
        boundaries(squares, k*dt, b_pos, c_list)

    C = np.zeros((horizon, num_const))

    c = 0
    for k in range(horizon-1):
        t = k*dt 
        for square in squares:
            if square.timespan[0]-square.init_time <= t < square.timespan[1]:
                C[k][c] = 1
                c += 1

    A_s = squares[0].A
    b_stack = cp.hstack(b_pos)
    c_stack = cp.hstack(c_list)

    x_select = []
    x_n = x @ C
    for i in range(num_const):
        m = i%3
        x_select.append(x_n[2*m:2*(m+1),i])

    x_select = cp.vstack(x_select).T
        
    constraints += [A_s @ (x_select-c_stack) <= b_stack]

    objective = cp.Minimize(cp.norm(u)-collision_cost)
    prob = cp.Problem(objective, constraints)

    start_time = time.perf_counter()

    result = prob.solve(solver=cp.CLARABEL)

    print(result)
    print("compute time: {}s".format(time.perf_counter()-start_time))

    return x.value[:6].T

def optimize_new(num_agents):
    A = np.zeros((num_agents*4, num_agents*4))
    for i in range(num_agents*2):
        A[i,i+num_agents*2] = 1
    B = np.zeros((num_agents*4, num_agents*2))
    for i in range(num_agents*2):
        B[i+num_agents*2,i] = 1


    print(B)

def anim_square(length, squares, pos_vals):
    fig, ax = plt.subplots()
    active_square = []
    
    lines = []
    
    start_time = time.perf_counter()

    def init():
        #ax.clear()
        ax.set_xlim(-10, 30)  # Set as per your data range
        ax.set_ylim(-20, 20)
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
        
        offsets = []
        for i in range(3):
            idx = frame%(length-1)
            offsets.append([pos_vals[idx][2*i], pos_vals[idx][2*i+1]])
        scatter_point.set_offsets(offsets)

        return lines + [scatter_point]

    ani = animation.FuncAnimation(
        fig, update, frames=length, init_func=init, blit=True, interval=200, repeat=False
    )
    ani.save(filename="MPC_multi_demo.gif", writer="ffmpeg")
    #plt.show()

def main():
    squares = [
        Square(5, (0,0), (0, 1.5), 0),
        Square(5, (5,10), (0, 1.5), 0),
        Square(5, (15,10), (0, 1.5), 0),

        Square(5, (15,0), (3, 5), 2),
        Square(5, (10,-5), (3, 5), 2),
        Square(5, (5,-5), (3, 5), 2)]

    x_vals = optimize_path(HORIZON, squares)
    anim_square(HORIZON, squares, x_vals)

def test():
    optimize_new(3)

if __name__ == "__main__":
    #main()
    test()