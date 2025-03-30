import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import subprocess
import time

glider = "glider_nautilus"

class BladderMonitor(Node):
    def __init__(self):
        super().__init__('bladder_monitor')

        # Parameters
        self.target_depth = 0.5
        self.bladder_topic = f"/model/{glider}/joint/bladder_joint/cmd_thrust"
        self.neutral_thrust = -35.75  # Known neutral thrust
        self.min_thrust = -40  # Minimum thrust
        self.max_thrust = -30  # Maximum thrust
        self.has_reached_target_depth = False
        self.shutdown_triggered = False
        self.callback_frequency = 0.001  # Frequency in seconds

        # PID control parameters
        self.kp = -2.0  # Proportional gain (negative for descending force)
        self.last_depth = 0.0
        self.last_time = 0.0

        # Initialize bladder thrust
        self.current_thrust = self.min_thrust
        self.set_bladder(self.current_thrust)

        # Subscriptions and timers
        self.subscription = self.create_subscription(
            PointStamped,
            f"/model/{glider}/sea_pressure_depth",
            self.depth_callback,
            10
        )
        self.timer = self.create_timer(self.callback_frequency, self.process_depth)

        self.get_logger().info(f"Initial bladder thrust set to {self.neutral_thrust}.")

    def set_bladder(self, bladder_value):
        """Send bladder command to Gazebo topic."""
        bladder_value = max(self.min_thrust, min(bladder_value, self.max_thrust))  # Clamp thrust within limits
        bladder_command = f"gz topic -t {self.bladder_topic} -m gz.msgs.Double -p 'data: {bladder_value}'"
        subprocess.run(bladder_command, shell=True)
        self.get_logger().info(f"Bladder set to {bladder_value:.2f}. Current depth: {self.last_depth:.4f} meters")
        self.current_thrust = bladder_value

    def depth_callback(self, msg):
        """Receive depth information."""
        self.latest_depth_msg = msg.point.z

    def process_depth(self):
        """Process depth and adjust bladder thrust."""
        if not hasattr(self, 'latest_depth_msg') or self.latest_depth_msg is None:
            return

        current_depth = self.latest_depth_msg

        # Check if target depth is overshot
        if current_depth > self.target_depth and self.last_depth < self.target_depth or current_depth < self.target_depth and self.last_depth > self.target_depth:
            self.get_logger().info(f"Target depth overshot: {current_depth:.2f} meters. Setting to neutral thrust.")
            self.set_bladder(self.neutral_thrust)

        current_time = time.time()

        if self.last_depth is not None and self.last_time is not None:
            # Proportional control (depth error)
            depth_error = self.target_depth - current_depth

            # Adjust thrust dynamically
            thrust_adjustment = self.kp * depth_error
            new_thrust = self.current_thrust + thrust_adjustment

            self.set_bladder(new_thrust)

            # Stop bladder adjustment if target depth is reached and maintain neutral thrust
            if abs(depth_error) < 1e-5:
                self.get_logger().info(f"Target depth {self.target_depth} meters reached. Maintaining position.")
                self.set_bladder(self.neutral_thrust)
                self.has_reached_target_depth = True
                self.shutdown_triggered = True
                rclpy.shutdown()

        # Update the previous depth and time for next iteration
        self.last_depth = current_depth
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)

    depth_monitor = BladderMonitor()

    try:
        rclpy.spin(depth_monitor)
    except KeyboardInterrupt:
        depth_monitor.get_logger().info("Bladder monitoring interrupted by user.")
    finally:
        if not depth_monitor.shutdown_triggered:
            depth_monitor.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
