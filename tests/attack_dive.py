import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped
import subprocess
import time

glider = "glider_nautilus"

class WeightAndBladderMonitor(Node):
    def __init__(self, target_weight_position, target_depth):
        super().__init__('weight_and_bladder_monitor')

        # Parameters
        self.target_weight_position = target_weight_position
        self.target_depth = target_depth

        # Weight-related parameters
        self.weight_topic = f"/model/{glider}/joint/weight_joint/cmd_vel"
        self.joint_state_topic = f"/world/oceans_waves/model/{glider}/joint_state"
        self.initial_weight_velocity = 0.1
        self.weight_target_reached = False

        # Bladder-related parameters
        self.bladder_topic = f"/model/{glider}/joint/bladder_joint/cmd_thrust"
        self.neutral_thrust = -35.75
        self.min_thrust = -38.0
        self.max_thrust = -33.0
        self.kp = -2.0  # Proportional gain
        self.bladder_target_reached = False

        # Frequency control
        self.callback_frequency = 0.01  # secs
        self.last_callback_time = 0.0

        # Store latest messages
        self.latest_weight_position = None
        self.latest_depth = None

        # Initialize weight movement
        self.set_weight_velocity(self.initial_weight_velocity)
        self.get_logger().info(f"Initial weight velocity set to {self.initial_weight_velocity}.")

        # Initialize bladder thrust
        self.current_bladder_thrust = self.neutral_thrust
        self.set_bladder_thrust(self.current_bladder_thrust)
        self.get_logger().info(f"Initial bladder thrust set to {self.neutral_thrust}.")

        # Subscriptions
        self.create_subscription(
            JointState,
            self.joint_state_topic,
            self.weight_position_callback,
            10
        )
        self.create_subscription(
            PointStamped,
            f"/model/{glider}/sea_pressure_depth",
            self.depth_callback,
            10
        )

        # Timer for control
        self.timer = self.create_timer(self.callback_frequency, self.process_monitoring)

    def set_weight_velocity(self, velocity):
        """Send weight velocity command to Gazebo topic."""
        command = f"gz topic -t {self.weight_topic} -m gz.msgs.Double -p 'data: {velocity}'"
        subprocess.run(command, shell=True)
        self.get_logger().info(f"Weight velocity set to {velocity}.")

    def set_bladder_thrust(self, thrust):
        """Send bladder thrust command to Gazebo topic."""
        thrust = max(self.min_thrust, min(thrust, self.max_thrust))  # Clamp thrust within limits
        command = f"gz topic -t {self.bladder_topic} -m gz.msgs.Double -p 'data: {thrust}'"
        subprocess.run(command, shell=True)
        self.get_logger().info(f"Bladder thrust set to {thrust:.2f}.")

    def weight_position_callback(self, msg):
        """Callback function for weight position updates."""
        if "weight_joint" in msg.name:
            idx = msg.name.index("weight_joint")
            self.latest_weight_position = msg.position[idx]

    def depth_callback(self, msg):
        """Callback function for depth updates."""
        self.latest_depth = msg.point.z

    def process_monitoring(self):
        """Monitor both weight position and depth."""
        if self.latest_weight_position is not None and not self.weight_target_reached:
            self.check_weight_position()

        if self.latest_depth is not None and not self.bladder_target_reached:
            self.adjust_bladder_thrust()

        if self.weight_target_reached and self.bladder_target_reached:
            self.get_logger().info("Both weight position and depth targets achieved. Shutting down...")
            rclpy.shutdown()

    def check_weight_position(self):
        """Check and adjust weight position."""
        current_position = self.latest_weight_position
        if current_position >= self.target_weight_position - 1e-3:
            self.get_logger().info(f"Target weight position {self.target_weight_position} meters reached. Stopping movement...")
            self.set_weight_velocity(0.0)
            self.weight_target_reached = True

    def adjust_bladder_thrust(self):
        """Adjust bladder thrust to maintain target depth."""
        current_depth = self.latest_depth

        depth_error = self.target_depth - current_depth
        thrust_adjustment = self.kp * depth_error
        new_thrust = self.current_bladder_thrust + thrust_adjustment

        self.set_bladder_thrust(new_thrust)

        if abs(depth_error) < 1e-3:
            self.get_logger().info(f"Target depth {self.target_depth} meters reached. Maintaining position.")
            self.set_bladder_thrust(self.neutral_thrust)
            self.bladder_target_reached = True

def main(args=None):
    rclpy.init(args=args)

    target_weight_position = float(input("Enter the target weight position (in meters): "))
    target_depth = float(input("Enter the target depth (in meters): "))

    monitor = WeightAndBladderMonitor(target_weight_position, target_depth)

    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        monitor.get_logger().info("Monitoring interrupted by user.")
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

