import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class DepthPressureLogger(Node):
    def __init__(self):
        super().__init__('depth_pressure_logger')
        self.pressure_sub = self.create_subscription(
            Float32,  # Replace with correct message type
            '/model/glider_nautilus/sea_pressure_depth',
            self.depth_callback,
            10
        )

    def depth_callback(self, msg):
        current_depth = msg.data  # Depth data in meters
        self.get_logger().info(f'Current Depth: {current_depth} meters')

def main(args=None):
    rclpy.init(args=args)
    depth_logger = DepthPressureLogger()
    rclpy.spin(depth_logger)
    depth_logger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

