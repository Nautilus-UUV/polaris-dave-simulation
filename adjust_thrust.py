import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import subprocess

glider = "glider_nautilus"

class DepthMonitor(Node):
    def __init__(self):
        super().__init__('depth_monitor')

        # Target depth and Gazebo topic
        self.target_depth = 3.0
        self.thrust_topic = "/model/"+glider+"/joint/bladder_joint/cmd_thrust"
        self.initial_thrust = -150.0

        # Create ROS 2 subscription to depth topic
        self.subscription = self.create_subscription(
            PointStamped,
            "/model/"+glider+"/sea_pressure_depth",
            self.depth_callback,
            10
        )

        # Initialize thrust
        self.set_thrust(self.initial_thrust)
        self.get_logger().info(f"Initial thrust set to {self.initial_thrust}.")

    def set_thrust(self, thrust_value):
        """Send thrust command to Gazebo topic."""
        thrust_command = f"gz topic -t {self.thrust_topic} -m gz.msgs.Double -p 'data: {thrust_value}'"
        subprocess.run(thrust_command, shell=True)
        self.get_logger().info(f"Thrust set to {thrust_value}.")

    def depth_callback(self, msg):
        """Callback function to process depth readings."""
        current_depth = msg.point.z
        self.get_logger().info(f"Current depth: {current_depth:.2f} meters")

        # Stop thrust when the target depth is reached
        if current_depth >= self.target_depth:
            self.get_logger().info(f"Target depth of {self.target_depth} meters reached. Stopping thrust...")
            self.set_thrust(0.0)
            # Shutdown the node gracefully
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    depth_monitor = DepthMonitor()

    try:
        rclpy.spin(depth_monitor)
    except KeyboardInterrupt:
        depth_monitor.get_logger().info("Depth monitoring interrupted by user.")
    finally:
        depth_monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
