import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Vector3
import subprocess

class AngleAttackMonitor(Node):
    def __init__(self):
        super().__init__('angle_attack_monitor')

        # Target depth
        self.target_depth = 3.0
        self.initial_ang_vel = Vector3(x=0.0, y=0.0, z=-300.0)  # Example: downward motion

        # Create ROS 2 subscription to depth topic
        self.subscription = self.create_subscription(
            PointStamped,
            '/model/glider_nautilus/sea_pressure_depth',
            self.depth_callback,
            10
        )

        # Publisher for angular velocity
        self.ang_vel_publisher = self.create_publisher(
            Vector3,
            '/model/glider_nautilus/joint/propeller_joint/ang_vel',
            10
        )

        # Set initial angular velocity
        self.set_ang_vel(self.initial_ang_vel)
        self.get_logger().info(f"Initial angular velocity set to {self.initial_ang_vel}.")

    def set_ang_vel(self, ang_vel):
        """Send angular velocity command to the glider."""
        self.ang_vel_publisher.publish(ang_vel)
        self.get_logger().info(f"Angular velocity set to x: {ang_vel.x}, y: {ang_vel.y}, z: {ang_vel.z}.")

    def depth_callback(self, msg):
        """Callback function to process depth readings."""
        current_depth = msg.point.z
        self.get_logger().info(f"Current depth: {current_depth:.2f} meters")

        # Stop or adjust angular velocity when the target depth is reached
        if current_depth >= self.target_depth:
            self.get_logger().info(f"Target depth of {self.target_depth} meters reached. Stopping motion...")
            self.set_ang_vel(Vector3(x=0.0, y=0.0, z=0.0))  # Stop motion
            # Shutdown the node gracefully
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    angle_attack_monitor = AngleAttackMonitor()

    try:
        rclpy.spin(angle_attack_monitor)
    except KeyboardInterrupt:
        angle_attack_monitor.get_logger().info("Monitoring interrupted by user.")
    finally:
        angle_attack_monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

