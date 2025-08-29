import sys
import time
import rclpy
import signal
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import colors as mcolors

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from barrier_msg.srv import BCompSrv
from barrier_msg.msg import BMsg, TMsg

from incorporate_barrier.bMsg import bMsg, HyperCubeHandler
from incorporate_barrier.MPC_barrier import optimize_path
from solve_setpoint.solvers.combined_solve import solve_combined


END = 5.4
STEPS = 100
DT = END / STEPS 


class BarrierDev(Node):
    def __init__(self):
        super().__init__('barrier_dev')

        self.get_logger().info("hello WOrld")

        self.barrier_client = self.create_client(BCompSrv, '/compute_barriers')
        while not self.barrier_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        #keep node running for better cleanup behaviour
        #self.timer = self.create_timer(1, self.dummy_loop)


        self.get_logger().info('start using of service')
        self.req = BCompSrv.Request()
        

        self.task_paths = [[[3, 0], [0, 1], [1, 2]], [[0, 1], [1, 4]], [[4, 5], [5, 6], [6, 7]], [[4, 5]]]
        task_pos = [[20, 10], [10, -10], [15, 0], [10, -10]]
        task_box = [[20, 20, 20, 20], [10, 10, 10, 10], [10, 10, 10, 10], [10, 10, 10, 10]]
        return_mode = "relative"

        succes, self.edge_pos, edge_box = solve_combined(self.task_paths, task_pos, task_box, return_mode)
        self.abs_pos = solve_combined(self.task_paths, task_pos, task_box, "absolute")

        for (edge, pos), (_, box) in zip(self.edge_pos, edge_box):
            tmsg = TMsg()
            #box_transposed = [box[0]+pos[0], -box[1]+pos[0], box[2]+pos[1], -box[3]+pos[1]]
            #self.get_logger().info(f'{box_transposed}')
            tmsg.center.extend(pos)
            tmsg.size.extend(box)
            tmsg.start = 5.
            tmsg.end = 6.
            tmsg.edge_i = int(edge[0])
            tmsg.edge_j = int(edge[1])
            tmsg.type = "always"
            self.req.messages.append(tmsg)

        future = self.barrier_client.call_async(self.req)
        future.add_done_callback(self.my_callback)

    def my_callback(self, future):

        self.msgs = []
        try:
            response = future.result()

            for bmsg in response.messages:
                my_msg = HyperCubeHandler(
                    bmsg.slopes, 
                    bmsg.gamma0, 
                    bmsg.r, 
                    bmsg.slack, 
                    bmsg.b_vector, 
                    bmsg.time_grid, 
                    bmsg.task_id, 
                    bmsg.edge_i, 
                    bmsg.edge_j)
                
                self.msgs.append(my_msg)

            for bmsg in self.msgs:
                candidates = [bmsg2 for bmsg2 in self.msgs if bmsg2.edge_j == bmsg.edge_i]
                if candidates:
                    bmsg.add_neighbour(candidates[0])

        except Exception as e:
            self.get_logger().info(f"Service call failed: {e}", )

    def used_agent(self):
        used_agents = []
        for task_path in self.task_paths:
            for vals in task_path:
                for i in range(2):
                    if vals[i] not in used_agents:
                        used_agents.append(vals[i])

        return used_agents

        

    def dummy_loop(self):
        pass

    def abs_from_rel(self, edge_rel, hypercubes):
        edges = [edge for edge, _ in edge_rel]
        idx1 = [idx[0] for idx in edges]
        idx2 = [idx[1] for idx in edges]
        start_idx = [idx for idx in idx1 if idx not in idx2]

        paths = [start_idx.copy()]

        def traverse(vals: list):
            tot_vals = []
            for a_val in vals:
                new_idx = [i for i, val in enumerate(idx1) if val == a_val]
                new_vals = [val for i, val in enumerate(idx2) if i in new_idx]
                tot_vals.extend(new_vals)

                for i, val in enumerate(new_vals):
                    #self.get_logger().info(f"{paths}")
                    if i < 1:
                        #set current path in first iteration
                        current_path = [path for path in paths if path[-1] == a_val][0]
                        current_path.append(val)
                    else:
                        paths.append(current_path.copy())
                        paths[-1][-1] = val
                        

            return tot_vals

        new_vals = start_idx

        for i in range(len(edge_rel)):  
            new_vals = traverse(new_vals)
            if not new_vals:
                break
        
        positions = {}
        rel_pos = np.array([rel for _, rel in edge_rel])
        for path in paths:
            for i in range(len(path)):
                my_path = path[:i+1]
                indices = [i for i, val in enumerate(idx2) if val in my_path]
                sum = np.sum(rel_pos[indices], axis=0)
                positions[my_path[-1]] = sum

        return positions

def flatten(x):
    return [item for e in x for item in e]

def poly_lines(vertices):
    return flatten([[[vertices[i-1][0], vertices[i][0]],[vertices[i-1][1], vertices[i][1]]] for i in range(len(vertices))])

def get_vertices(hypercube):
    return np.array([[hypercube[0], hypercube[2]], [hypercube[0], -hypercube[3]], [-hypercube[1], -hypercube[3]], [-hypercube[1], hypercube[2]]])

def signal_handler(sig, frame):
    print("Shutdown signal received, cleaning up...")
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(0)

#chatGPT animation
def animate_msgs(node, pos, timesteps):
    fig, ax = plt.subplots()

    x_indices = [i * 2 for i in range(len(pos[0]) // 2)]
    y_indices = [i * 2 + 1 for i in range(len(pos[0]) // 2)]

    agents = node.used_agent()

    ax.set_xlim(-20, 30)
    ax.set_ylim(-20, 20)


    #scatter = ax.scatter(pos[0][x_indices], pos[0][y_indices])
    scatter = ax.scatter([], [])
    plotted_lines = []

    def init():
        return []

    def update(k):
        t = timesteps[k]

        # Remove previously drawn lines
        while plotted_lines:
            plotted_lines.pop().remove()

        # Draw all messages at timestep t
        for (edge, _), msg in zip(node.edge_pos, node.msgs):
            idx = agents.index(msg.edge_i)
            cube = msg.rel_offset_trans(t, pos[k,idx*2:(idx*2)+2])
            lines = poly_lines(get_vertices(cube))
            plotted_lines.extend(ax.plot(*lines, color="blue"))

        indices = [[0, 1], [1, 2], [2, 3], [2, 4], [4, 5], [5, 6], [6, 7]]
        idx_choice = [x_indices, y_indices]
        pos_list = [[pos[k][idx_choice[i]][idx[0]], pos[k][idx_choice[i]][idx[1]]] for idx in indices for i in range(2)]
        plotted_lines.extend(ax.plot(*pos_list, color="orange"))

        scatter.set_offsets(np.column_stack((pos[k][x_indices], pos[k][y_indices])))
        return plotted_lines + [scatter]

    anim = animation.FuncAnimation(fig, update, frames=len(timesteps), init_func=init, blit=True)
    #anim.save("barrier_service_rel.mp4")
    plt.show()
    return anim



def main(args=None):
    rclpy.init(args=args)

    node = BarrierDev()

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin_once()

    time.sleep(1)
    executor.spin_once()
    time.sleep(1)

    #hypercubes = node.msgs[0].c
    #animate_msgs(node, node.abs_from_rel(node.edge_pos, None))
    agents = node.used_agent()
    
    timespan = np.linspace(0, END, STEPS)

    pos = optimize_path(STEPS, node.msgs, agents, dt=DT)
    print(pos[0].shape)

    x_indices = [i*2 for i in range(len(agents))]
    y_indices = [i*2+1 for i in range(len(agents))]

    print(x_indices)
    
    #plt.scatter(pos[1][x_indices], pos[2][y_indices])
    #plt.scatter(pos[2][x_indices], pos[2][y_indices])
    

    animate_msgs(node, pos, timespan)

    # for k, t in enumerate(timespan):
    #     plt.scatter(pos[k][x_indices], pos[k][y_indices])
    #     for msg in node.msgs:
    #         cube = msg.get_offset(t)
    #         lines = poly_lines(get_vertices(cube))
    #         plt.plot(*lines)
    #     plt.show()


    # plt.scatter(node.abs_pos[:,0], node.abs_pos[:,1])
    # indices = [[0, 1], [1, 2], [2, 3], [2, 4], [4, 5]]
    # pos_list = [[node.abs_pos[idx[0],i], node.abs_pos[idx[1],i]] for idx in indices for i in range(2)]
    # plt.plot(*pos_list)
    # plt.show()

    # pos = node.abs_from_rel(node.edge_pos)
    # for (edge, _), msg in zip(node.edge_pos, node.msgs):
    #     cube = msg.compute_offset_vector(2.49)
    #     cube_trans = [cube[0] + pos[edge[0]][0], cube[1] + pos[edge[0]][0], cube[2] + pos[edge[0]][1], cube[3] + pos[edge[0]][1]]
    #     lines = poly_lines(get_vertices(cube_trans))
    #     plt.plot(*lines, color='blue')
        
    # plt.scatter(node.abs_pos[:,0], node.abs_pos[:,1])
    # plt.show()

    sys.exit(0)

if __name__ == "__main__":
    main()
