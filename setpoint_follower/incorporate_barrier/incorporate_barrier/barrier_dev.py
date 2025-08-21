import sys
import rclpy
import signal

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

class BarrierDev(Node):
    def __init__(self):
        super().__init__('barrier_dev')

        self.get_logger().info("hello WOrld")

        #self.timer = self.create_timer(1, self.mainloop)
        #self.time2 = self.create_timer(1, self.mainloop)

    def mainloop(self):
        self.get_logger().info("doing stuff")

    def signal_handler(self, sig, frame):
        self.get_logger().info("Shutdown signal received, cleaning up...")
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(0)

def main(args=None):
    rclpy.init(args=args)

    node = BarrierDev()
    node.mainloop()

    signal.signal(signal.SIGINT, node.signal_handler)
    signal.signal(signal.SIGTERM, node.signal_handler)
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin_once(timeout_sec=0.1)
    node.mainloop()
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
