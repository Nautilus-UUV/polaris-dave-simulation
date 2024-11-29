import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import subprocess
import time  # For throttling logs

glider = "glider_nautilus"

class WeightMonitor(Node):
    def __init__(self):
        super().__init__('weight_monitor')

        # Weight parameters
        self.weight_topic = f"/model/{glider}/joint/weight_joint/cmd_vel"
        self.joint_state_topic = f"/world/oceans_waves/model/{glider}/joint_state"
        self.initial_velocity = 0.1
        self.weight_target_position = 0.5

        # Threshold for surface re-entry
        self.has_reached_target_position = False
        self.shutdown_triggered = False

        # Frequency control
        self.callback_frequency = 0.01  # secs
        self.last_callback_time = 0.0

        # Logging throttle
        self.log_frequency = 0.1  # secs
        self.last_log_time = time.time()

        # Store position information
        self.latest_position_msg = None
        self.last_logged_position = None
        self.eq_counter = 0

        # Subscribe to the joint_state topic
        self.create_subscription(
            JointState,
            self.joint_state_topic,
            self.position_callback,
            10  # QoS depth
        )

        # Timer to control processing frequency
        self.timer = self.create_timer(self.callback_frequency, self.process_position)

        # Initialize weight movement
        self.set_weight_velocity(self.initial_velocity)
        self.get_logger().info(f"Initial weight velocity set to {self.initial_velocity}.")

    def set_weight_velocity(self, velocity):
        """Send weight velocity command to Gazebo topic."""
        command = f"gz topic -t {self.weight_topic} -m gz.msgs.Double -p 'data: {velocity}'"
        subprocess.run(command, shell=True)
        self.get_logger().info(f"Weight velocity set to {velocity}.")

    def position_callback(self, msg):
        """Callback function to process the joint state data."""
        # Extract the position of weight_joint from the JointState message
        if "weight_joint" in msg.name:
            idx = msg.name.index("weight_joint")
            self.latest_position_msg = msg.position[idx]

    def process_position(self):
        """Process the latest position message at a controlled frequency."""
        if not self.latest_position_msg:
            return
        
        current_position = self.latest_position_msg

        # Log the position at throttled intervals
        current_time = time.time()
        if current_time - self.last_log_time >= self.log_frequency:
            self.get_logger().info(f"Weight position: {current_position:.3f} meters")
            self.last_log_time = current_time
            self.last_logged_position = current_position

        # Check for equilibrium and shutdown if stable
        self.check_position(current_position)
        self.check_equilibrium(current_position)

    def check_position(self, current_position):
        """Check if the target position is reached and stop the weight."""
        if current_position >= self.weight_target_position - 1e-3 and not self.has_reached_target_position:
            self.get_logger().info(f"Target weight position {self.weight_target_position} meters reached. Stopping movement...")
            self.set_weight_velocity(0.0)
            self.has_reached_target_position = True

    def check_equilibrium(self, current_position):
        """Check if equilibrium is reached and shut down the node."""
        if self.last_logged_position is not None:
            # Compare current position with the last logged position
            if abs(current_position - self.last_logged_position) < 1e-3:
                self.eq_counter += 1
            else:
                self.eq_counter = 0

            # If the position has stabilized over multiple cycles, shut down
            if self.eq_counter >= int(1/self.log_frequency):
                self.get_logger().info("Weight has reached equilibrium. Shutting down gracefully...")
                self.shutdown_triggered = True
                rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    weight_monitor = WeightMonitor()

    try:
        rclpy.spin(weight_monitor)
    except KeyboardInterrupt:
        weight_monitor.get_logger().info("Weight monitoring interrupted by user.")
    finally:
        # Check if shutdown was already triggered
        if not weight_monitor.shutdown_triggered:
            weight_monitor.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
