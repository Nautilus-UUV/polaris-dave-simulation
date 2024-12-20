import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import subprocess

glider = "glider_nautilus"

class BladderMonitor(Node):
    def __init__(self):
        super().__init__('bladder_monitor')

        # Inputs
        self.target_depth = 1
        self.isStrokeDownward = True

        # Parameters
        self.bladder_topic = f"/model/{glider}/joint/bladder_joint/cmd_thrust"
        self.neutral_thrust = -35.75  # Known neutral thrust
        self.down_thrust = -10  # Minimum thrust differential (downwards)
        self.up_thrust = +10  # Maximum thrust differential (upwards)
        self.has_reached_target_depth = False
        self.shutdown_triggered = False
        self.callback_frequency = 1e-5  # Frequency in seconds

        # Overshooting counter
        self.overshoot_counter = 1

        # Initialize bladder thrust
        if self.isStrokeDownward: self.current_thrust = self.neutral_thrust + self.down_thrust
        else: self.current_thrust = self.neutral_thrust + self.up_thrust
        self.set_bladder(self.current_thrust)

        # Subscriptions and timers
        self.subscription = self.create_subscription(
            PointStamped,
            f"/model/{glider}/sea_pressure_depth",
            self.depth_callback,
            10
        )
        self.timer = self.create_timer(self.callback_frequency, self.process_depth)

        self.get_logger().info(f"Initial bladder thrust set to {self.current_thrust}.")

    def set_bladder(self, bladder_value):
        """Send bladder command to Gazebo topic."""
        bladder_value = max(self.neutral_thrust + self.down_thrust, min(bladder_value, self.neutral_thrust + self.up_thrust))  # Clamp thrust within limits
        bladder_command = f"gz topic -t {self.bladder_topic} -m gz.msgs.Double -p 'data: {bladder_value}'"
        subprocess.run(bladder_command, shell=True)
        self.get_logger().info(f"Bladder set to {bladder_value:.2f}.")
        self.current_thrust = bladder_value

    def depth_callback(self, msg):
        """Receive depth information."""
        self.latest_depth_msg = msg.point.z

    def process_depth(self):
        """Process depth and adjust bladder thrust."""
        if not hasattr(self, 'latest_depth_msg') or self.latest_depth_msg is None:
            return

        current_depth = self.latest_depth_msg

        if current_depth > self.target_depth and self.last_depth < self.target_depth:
            self.get_logger().info(f"Target depth overshot. Slown down factor: {self.overshoot_counter}.")
            self.set_bladder(self.neutral_thrust + self.up_thrust/(2**self.overshoot_counter))
            self.overshoot_counter += 1
        elif current_depth < self.target_depth and self.last_depth > self.target_depth:
            self.get_logger().info(f"Target depth overshot. Slown down factor: {self.overshoot_counter}.")
            self.set_bladder(self.neutral_thrust + self.down_thrust/(2**self.overshoot_counter))
            self.overshoot_counter += 1

        if self.last_depth is not None and self.last_time is not None:
            # Equilibrium reached?
            depth_error = self.target_depth - current_depth

            # Stop bladder adjustment if target depth is reached and maintain neutral thrust
            if abs(depth_error) < 1e-3:
                self.get_logger().info(f"Target depth {self.target_depth} meters reached. Maintaining position.")
                self.set_bladder(self.neutral_thrust)
                self.has_reached_target_depth = True
                self.shutdown_triggered = True
                rclpy.shutdown()

        # Depth logger
        self.get_logger().info(f"Current depth: {current_depth:.4f} meters")

        # Update the previous depth and time for next iteration
        self.last_depth = current_depth

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
