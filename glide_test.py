import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import subprocess

glider = "glider_nautilus"

class DepthMonitor(Node):
    def __init__(self):
        super().__init__('depth_monitor')

        # Target depth and Gazebo topics
        self.target_depth = 3.0
        self.thrust_topic = "/model/" + glider + "/joint/bladder_joint/cmd_thrust"
        self.sliding_mass_topic = "/model/" + glider + "/joint/sliding_mass_joint/cmd_vel"
        self.joint_state_topic = "/world/oceans_waves/model/" + glider + "/joint_state"

        self.initial_thrust = -150.0
        self.sliding_mass_velocity = 0.1
        self.current_sliding_mass_position = 0.0
        self.sliding_mass_target_position = 0.5
        self.sliding_mass_moving = False

        # Shutdown flag to ensure rclpy.shutdown() is called only once
        self.shutdown_triggered = False

        # Create ROS 2 subscription to depth topic
        self.subscription = self.create_subscription(
            PointStamped,
            "/model/" + glider + "/sea_pressure_depth",
            self.depth_callback,
            10
        )

        # Create a timer to monitor joint state
        self.timer = self.create_timer(0.1, self.monitor_sliding_mass_pos)

        # Initialize thrust
        self.set_thrust(self.initial_thrust)
        self.get_logger().info(f"Initial thrust set to {self.initial_thrust}.")

    def set_thrust(self, thrust):
        """Send thrust command to bladder_joint."""
        thrust_command = f"gz topic -t {self.thrust_topic} -m gz.msgs.Double -p 'data: {thrust}'"
        subprocess.run(thrust_command, shell=True)
        self.get_logger().info(f"Thrust set to {thrust}.")

    def set_sliding_mass_vel(self, vel):
        """Send velocity command to sliding_mass_joint."""
        sliding_mass_command = f"gz topic -t {self.sliding_mass_topic} -m gz.msgs.Double -p 'data: {vel}'"
        subprocess.run(sliding_mass_command, shell=True)
        self.get_logger().info(f"Sliding mass velocity set to {vel}.")

    def monitor_sliding_mass_pos(self):
            """Monitor the position of the sliding mass."""
            if self.sliding_mass_moving:
                try:
                    # Check the joint state to get the position of the sliding mass
                    joint_state_output = subprocess.check_output(
                        f"gz topic -e {self.joint_state_topic}", shell=True, text=True
                    )

                    # Extract position of sliding_mass_joint from joint state
                    if "sliding_mass_joint" in joint_state_output:
                        lines = joint_state_output.splitlines()
                        for line in lines:
                            if "sliding_mass_joint" in line:
                                # Extract position from the pose field
                                position_str = line.split("pose")[1]
                                position_value = position_str.split("{")[1].split("}")[0].split(":")[1].strip()

                                # The position could be a nested structure, extract the actual numeric value
                                position = float(position_value)

                                # Update current sliding mass position
                                self.current_sliding_mass_position = position
                                self.get_logger().info(f"Sliding mass position: {self.current_sliding_mass_position:.2f} meters")

                                break

                    # Stop sliding mass when target position is reached
                    if abs(self.current_sliding_mass_position - self.sliding_mass_target_position) <= 0.01:
                        self.get_logger().info(f"Target sliding mass position {self.sliding_mass_target_position} meters reached. Stopping movement...")
                        self.set_sliding_mass_vel(0.0)
                        self.sliding_mass_moving = False

                except Exception as e:
                    self.get_logger().error(f"Error monitoring sliding mass position: {e}")

    def depth_callback(self, msg):
        """Callback function to process depth readings."""
        current_depth = msg.point.z
        self.get_logger().info(f"Current depth: {current_depth:.2f} meters")

        # Stop thrust when the target depth is reached
        if current_depth >= self.target_depth:
            self.get_logger().info(f"Target depth of {self.target_depth} meters reached. Stopping thrust...")
            self.set_thrust(0.0)

            # Start moving the sliding mass
            if not self.sliding_mass_moving:
                self.get_logger().info("Starting sliding mass movement...")
                self.set_sliding_mass_vel(self.sliding_mass_velocity)
                self.sliding_mass_moving = True

            # Shutdown the node gracefully
            if not self.shutdown_triggered:
                self.shutdown_triggered = True
                rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    depth_monitor = DepthMonitor()

    try:
        rclpy.spin(depth_monitor)
    except KeyboardInterrupt:
        depth_monitor.get_logger().info("Depth monitoring interrupted by user.")
    finally:
        # Check if shutdown was already triggered
        if not depth_monitor.shutdown_triggered:
            depth_monitor.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
