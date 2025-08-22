import sys
import time
import rclpy
import signal
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from barrier_msg.srv import BCompSrv
from barrier_msg.msg import BMsg, TMsg

from incorporate_barrier.bMsg import bMsg
from solve_setpoint.solvers.combined_solve import solve_combined

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
        

        task_paths = [[[3, 0], [0, 1], [1, 2]], [[0, 1], [1, 4]], [[4, 5]]]
        task_pos = [[20, 10], [10, -10], [15, 0]]
        task_box = [[10, 10, 10, 10], [10, 10, 10, 10], [10, 10, 10, 10]]
        return_mode = "relative"

        succes, edge_pos, edge_box = solve_combined(task_paths, task_pos, task_box, return_mode)
        self.abs_pos = solve_combined(task_paths, task_pos, task_box, "absolute")

        self.get_logger().info(f'{edge_pos}')
        self.get_logger().info(f'{edge_box}')

        for (edge, pos), (_, box) in zip(edge_pos, edge_box):
            tmsg = TMsg()
            #box_transposed = [box[0]+pos[0], -box[1]+pos[0], box[2]+pos[1], -box[3]+pos[1]]
            #self.get_logger().info(f'{box_transposed}')
            tmsg.center.extend(pos)
            tmsg.size.extend(box)
            tmsg.start = 2.
            tmsg.end = 3.
            tmsg.edge_i = int(edge[0])
            tmsg.edge_j = int(edge[1])
            tmsg.type = "always"
            self.req.messages.append(tmsg)

        future = self.barrier_client.call_async(self.req)
        future.add_done_callback(self.my_callback)
        self.get_logger().info(f"{future}")

    def my_callback(self, future):

        self.msgs = []
        try:
            response = future.result()

            for bmsg in response.messages:
                my_msg = bMsg(
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

        except Exception as e:
            self.get_logger().info(f"Service call failed: {e}", )

        

    def dummy_loop(self):
        pass

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
def animate_msgs(node):
    fig, ax = plt.subplots()

    # Timesteps only (all msgs will be drawn for each t)
    timesteps = [i / 25 for i in range(50)]

    ax.set_xlim(0, 20)  # Adjust based on your data
    ax.set_ylim(-10, 10)

    plotted_lines = []

    def init():
        """Initialize animation (clear plot)."""
        return plotted_lines

    def update(t):
        """Update plot for timestep t."""
        nonlocal plotted_lines

        # Remove previously drawn lines
        for line in plotted_lines:
            line.remove()
        plotted_lines.clear()

        # Draw all msgs at timestep t
        for msg in node.msgs:
            hypercube = msg.compute_offset_vector(t)
            lines = poly_lines(get_vertices(hypercube))
            plotted_lines += ax.plot(*lines, color='blue')

        return plotted_lines

    ani = animation.FuncAnimation(
        fig, update, frames=timesteps, init_func=init, blit=True, interval=200
    )

    ani.save("barrier_service.mp4")
    plt.show()
    return ani


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

    # for msg in node.msgs:
    #     for t in [i/5 for i in range(10)]:
    #         hypercube = msg.compute_offset_vector(t)
    #         print(hypercube)
    #         lines = poly_lines(get_vertices(hypercube))
    #         plt.plot(*lines)
    #         print("a")

    # plt.show()
    animate_msgs(node)


    plt.scatter(node.abs_pos[:,0], node.abs_pos[:,1])
    indices = [[0, 1], [1, 2], [2, 3], [2, 4], [4, 5]]
    pos_list = [[node.abs_pos[idx[0],i], node.abs_pos[idx[1],i]] for idx in indices for i in range(2)]
    plt.plot(*pos_list)
    plt.show()

    sys.exit(0)
    # try:
    #     executor.spin()
    # except KeyboardInterrupt:
    #     print("Ctrl-C, shutting down...")
    # finally:
    #     executor.remove_node(node)
    #     node.destroy_node()
    #     if rclpy.ok():
    #         rclpy.shutdown()

if __name__ == "__main__":
    main()
