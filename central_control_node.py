import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class CentralControlNode(Node):
    def __init__(self):
        super().__init__('central_control_node')

        # Parameters for depth and pitch
        self.target_depth = self.declare_parameter('target_depth', 1.0).get_parameter_value().double_value
        self.target_pitch = self.declare_parameter('target_pitch', 5.0).get_parameter_value().double_value

        # Publishers for controlling depth and pitch
        self.depth_pub = self.create_publisher(Float32, '/model/glider_nautilus/central_control/target_depth', 10)
        self.pitch_pub = self.create_publisher(Float32, '/model/glider_nautilus/central_control/target_pitch', 10)

        # Set target values for depth and pitch
        self.set_target_depth(self.target_depth)
        self.set_target_pitch(self.target_pitch)

        # Create a timer to send depth and pitch commands periodically
        self.timer = self.create_timer(1.0, self.send_target_commands)

        self.get_logger().info("Central control node started!")

    def set_target_depth(self, depth):
        """Publish target depth."""
        msg = Float32()
        msg.data = depth
        self.depth_pub.publish(msg)
        self.get_logger().info(f"Target depth set to: {depth} meters")

    def set_target_pitch(self, pitch):
        """Publish target pitch."""
        msg = Float32()
        msg.data = pitch
        self.pitch_pub.publish(msg)
        self.get_logger().info(f"Target pitch set to: {pitch} degrees")

    def send_target_commands(self):
        """Periodically send target commands."""
        # Here, you can change the target values dynamically if needed
        self.set_target_depth(self.target_depth)
        self.set_target_pitch(self.target_pitch)

def main(args=None):
    rclpy.init(args=args)

    # Create the central control node
    central_control_node = CentralControlNode()

    try:
        # Keep the node running for any additional tasks
        rclpy.spin(central_control_node)
    except KeyboardInterrupt:
        central_control_node.get_logger().info("Movement node interrupted by user.")
    finally:
        # Clean up and shut down
        central_control_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
