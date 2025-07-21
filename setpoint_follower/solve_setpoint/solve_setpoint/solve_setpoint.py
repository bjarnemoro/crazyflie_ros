import rclpy
from rclpy.node import Node

class Solver(Node):
    def __init__(self):
        super().__init__('solver')
        self.get_logger().info("\n\nStarting!!\n\n")
        #self.srv = self.create_service(Int, 'give_path', self.solve_cb)

    def solve_cb(self, request, response):
        #find the shortest path

        #solve the positions and the radius


        return response

def main(args=None):
    rclpy.init(args=args)

    solver_service = Solver()

    rclpy.spin(solver_service)

    rclpy.shutdown()

if __name__ == "__main__":
    main()