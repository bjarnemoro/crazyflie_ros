import sys
import rclpy
import numpy as np
import signal
from dataclasses import dataclass
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
import tf_transformations
from rclpy.executors import MultiThreadedExecutor
from crazyflie_interfaces.msg import FullState
from incorporate_barrier.MPC_barrier import MPCsolver
from incorporate_barrier.bMsg import bMsg, HyperCubeHandler
from barrier_msg.srv import BCompSrv
from barrier_msg.msg import BMsg, TMsg

@dataclass
class State:
    x = 0
    y = 0
    z = 0
    yaw = 0

class Control(Node):
    def __init__(self):

        super().__init__("control")
        callback = lambda msg, idx=0: self.odom_callback(msg, idx)
        self.odom_subscribers = self.create_subscription(
                PoseStamped, "/cf01" + "/pose", callback, 10)
        
        self.twist_publishers = self.create_publisher(PoseStamped, "/cf01" + "/cmd_vel_legacy", 10)
        self.state_publisher = self.create_publisher(FullState, "/cf01" + "/cmd_full_state", 10)

        self.pos = np.zeros((1, 3))
        self.angles = np.zeros((1, 3))

        #self.timer = self.create_timer(0.02, self.control)
        self.dt = 0.02
        self.timer = self.create_timer(self.dt, self.full_state_control)

        self.state = State()
        self.twist = Twist()
        self.full_state = FullState()

        

        self.k = 0.7
        self.init_q = None
        self.online = False
        self.counter = 0

        self.start_time = self.get_clock().now().nanoseconds
        self.past_time = self.start_time

        

        self.barrier_client = self.create_client(BCompSrv, '/compute_barriers')
        while not self.barrier_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.req = BCompSrv.Request()

        tmsg = TMsg()
        tmsg.center.extend([0.5, 0.5])
        tmsg.size.extend([2, 2, 2, 2])
        tmsg.start = 5.
        tmsg.end = 7.
        tmsg.edge_i = 0
        tmsg.edge_j = 1
        tmsg.type = "always"
        self.req.messages.append(tmsg)

        self.mpc_solver = MPCsolver()

        future = self.barrier_client.call_async(self.req)
        future.add_done_callback(self.my_callback)
        

    def my_callback(self, future):
        self.get_logger().info(f"Called callback!!")

        self.barriers = []
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
                
                self.barriers.append(my_msg)

        except Exception as e:
            self.get_logger().info(f"Service call failed: {e}", )

        self.mpc_solver.initialize_problem(20, 0., self.barriers, [i for i in range(2)], dt=0.1, MAX_INPUT=0.5)
        
        
    def odom_callback(self, msg, idx=0):
        self.online = True
        #self.get_logger().info(f"{[msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]}")

        self.pos[idx, 0] = msg.pose.position.x
        self.pos[idx, 1] = msg.pose.position.y
        self.pos[idx, 2] = msg.pose.position.z

        q = msg.pose.orientation
        
        euler = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

        if self.init_q == None:
            self.init_q = [q.x, q.y, q.z, q.w]
        self.angles[idx, 0] = euler[0]
        self.angles[idx, 1] = euler[1]
        self.angles[idx, 2] = euler[2]
        

    def update_twist(self):
        self.time = self.get_clock().now().nanoseconds - self.start_time
        time_sec = self.time / 1e9

        if time_sec < 5:
            h = 0.9#time_sec/5
            self.twist.linear.z = self.k * (h - self.pos[0, 2])
            self.twist.linear.x = self.k * (0. - self.pos[0, 0])
            self.twist.linear.y = self.k * (0. - self.pos[0, 1])
        elif time_sec >= 5 and time_sec < 10:
            h = 0.9
            self.twist.linear.z = self.k * (h - self.pos[0, 2])
            self.twist.linear.x = self.k * (0.5 - self.pos[0, 0])
            self.twist.linear.y = self.k * (0. - self.pos[0, 1])
        elif time_sec >= 10 and time_sec < 15:
            h = 0.1#max(0, 1-(time_sec-5)/5)
            self.twist.linear.z = self.k * (h - self.pos[0, 2])
            self.twist.linear.x = self.k * (0.5 - self.pos[0, 0])
            self.twist.linear.y = self.k * (0. - self.pos[0, 1])
        else:
            self.twist.linear.z = 0.
            self.twist.linear.x = 0.
            self.twist.linear.y = 0.

        
        self.twist.angular.z = 0.

    def MPC_test(self):
        self.time = self.get_clock().now().nanoseconds - self.start_time
        time_sec = self.time / 1e9
        dt_sec = 0.1

        return_val = "input"

        if time_sec < 3:
            h = 1.#time_sec/5
            self.twist.linear.z = self.k * (h - self.pos[0, 2])
            self.twist.linear.x = self.k * (0. - self.pos[0, 0])
            self.twist.linear.y = self.k * (0. - self.pos[0, 1])
        elif time_sec >= 3 and time_sec < 10:
            x0 = np.array([0., 0., self.pos[0,0], self.pos[0,1]])
            u = self.mpc_solver.recompute(20, x0, time_sec - 3, self.barriers, dt=dt_sec, return_val=return_val)

            self.twist.linear.z = self.k * (1. - self.pos[0, 2])
            self.twist.linear.x = u[2]
            self.twist.linear.y = u[3]

            #self.pos[0, 0] += self.dt * u[2]
            #self.pos[0, 1] += self.dt * u[3]
        elif time_sec >= 10 and time_sec < 15:
            h = 0.1#max(0, 1-(time_sec-5)/5)
            self.twist.linear.z = self.k * (h - self.pos[0, 2])
            self.twist.linear.x = self.k * (0.5 - self.pos[0, 0])
            self.twist.linear.y = self.k * (0.5 - self.pos[0, 1])
        else:
            self.twist.linear.z = 0.
            self.twist.linear.x = 0.
            self.twist.linear.y = 0.

        self.twist.angular.z = 0.
        
            

    def full_state_control(self):
        #if not self.online:
        #    self.get_logger().info(f"not online")
        
        self.MPC_test()

        self.state.x = self.pos[0,0]
        self.state.y = self.pos[0,1]
        self.state.z = self.pos[0,2]
        #self.state.yaw = self.angle_normalize(self.state.yaw + self.twist.angular.z*self.dt)

        #q = tf_transformations.quaternion_from_euler(0, 0, self.state.yaw)

        #self.get_logger().info(f"{[q.x, q.y, q.z, q.w]}")
        #self.get_logger().info(f"{[q[0], q[1], q[2], q[3]]}")

        #self.get_logger().info(f"{[self.twist.linear.x, self.twist.linear.y, self.twist.linear.z]}")
        
        time_sec = self.time / 1e9
        
        if time_sec > 3:
            if time_sec-3 < 6:
                self.counter += 1
                if self.counter == 10:
                    self.counter = 0
                    off = self.barriers[0].compute_offset_vector(time_sec-3)
                    self.get_logger().info(f"{-off[1]} < {self.pos[0,0]} < {off[0]}   {-off[3]} < {self.pos[0,1]} < {off[2]}")

        self.full_state.pose.position.x = self.state.x
        self.full_state.pose.position.y = self.state.y
        self.full_state.pose.position.z = self.state.z
        self.full_state.twist.linear.x = self.twist.linear.x
        self.full_state.twist.linear.y = self.twist.linear.y
        self.full_state.twist.linear.z = self.twist.linear.z
        self.full_state.pose.orientation.x = 0.
        self.full_state.pose.orientation.y = 0.
        self.full_state.pose.orientation.z = 0.
        self.full_state.pose.orientation.w = 1.
        self.full_state.twist.angular.x = 0.
        self.full_state.twist.angular.y = 0.
        self.full_state.twist.angular.z = self.twist.angular.z
        
        self.state_publisher.publish(self.full_state)
    
    def angle_normalize(self, a):
        a = a % (2*np.pi)
        if (a > np.pi):
            a -= 2*np.pi
        return a

    def signal_handler(self, sig, frame):
        print("Shutdown signal received, cleaning up...")

        self.full_state.pose.position.x = self.state.x
        self.full_state.pose.position.y = self.state.y
        self.full_state.pose.position.z = self.state.z
        self.full_state.twist.linear.x = 0.
        self.full_state.twist.linear.y = 0.
        self.full_state.twist.linear.z = 0.
        if self.init_q is not None:
            self.full_state.pose.orientation.x = self.init_q[0]
            self.full_state.pose.orientation.y = self.init_q[1]
            self.full_state.pose.orientation.z = self.init_q[2]
            self.full_state.pose.orientation.w = self.init_q[3]
        self.full_state.twist.angular.x = 0.
        self.full_state.twist.angular.y = 0.
        self.full_state.twist.angular.z = 0.
        
        self.state_publisher.publish(self.full_state)

        rclpy.shutdown()
        sys.exit(0)

def main(args=None):
    rclpy.init(args=args)

    agents = Control()

    signal.signal(signal.SIGINT, agents.signal_handler)
    signal.signal(signal.SIGTERM, agents.signal_handler)

    

    executor = MultiThreadedExecutor()
    executor.add_node(agents)

    executor.spin()

    agents.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()