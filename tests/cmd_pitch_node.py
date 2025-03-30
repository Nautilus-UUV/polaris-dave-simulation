import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import subprocess
import time  # For throttling logs

glider = "glider_nautilus"

class WeightMonitor(Node):
    def __init__(self):
        super().__init__('weight_monitor')

        # Topics
        self.weight_topic = f"/model/{glider}/joint/weight_joint/cmd_vel"
        self.joint_state_topic = f"/world/oceans_waves/model/{glider}/joint_state"
        self.bladder_ang_vel_topic = f"/model/{glider}/joint/bladder_joint/ang_vel"

        # Weight control parameters
        self.weight_velocity = 0.1
        self.weight_target_position = 0.5

        self.has_reached_target_position = False
        self.shutdown_triggered = False

        self.callback_frequency = 1e-2  # secs

        self.weight_position = None

        # Bladder control parameters
        self.bladder_ang_vel = 100.0 # rad/sec
        self.bladder_target_pitch = None

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
        self.set_weight_velocity(self.weight_velocity)
        self.get_logger().info(f"Initial weight velocity set to {self.weight_velocity}.")

    def set_weight_velocity(self, velocity):
        """Send weight velocity command to Gazebo topic."""
        command = f"gz topic -t {self.weight_topic} -m gz.msgs.Double -p 'data: {velocity}'"
        subprocess.run(command, shell=True)
        self.get_logger().info(f"Weight velocity set to {velocity}.")

    def position_callback(self, msg):
        """Callback function to process the joint state data."""
        # Extract the positions of weight_joint from the JointState message
        if "weight_joint" in msg.name:
            idx = msg.name.index("weight_joint")
            self.weight_position = msg.position[idx]

        # Extract the position of bladder_joint from the JointState message
        if "bladder_joint" in msg.name:
            idx = msg.name.index("bladder_joint")
            self.bladder_position = msg.position[idx]  # This is the angular position (pitch)

        # Extract the position of the glider (base_link) if needed
        if "base_link" in msg.name:
            idx = msg.name.index("base_link")
            self.glider_position = msg.position[idx]  # This is the angular position of the base link

    def set_bladder_ang_vel(self, velocity):
        """Send bladder angular velocity command to Gazebo topic."""
        command = f"gz topic -t /model/{glider}/joint/bladder_joint/cmd_vel -m gz.msgs.Double -p 'data: {velocity}'"
        subprocess.run(command, shell=True)
        self.get_logger().info(f"Bladder angular velocity set to {velocity}.")

    def process_bladder_ang_vel(self):
        """Move bladder joint to target pitch."""
        self.bladder_target_pitch = -self.glider_position
        set_bladder_ang_vel(self.bladder_ang_vel) #############################!!!!!!!!!!!!!!!!!!!!

    def process_position(self):
        """Process the latest position message at a controlled frequency."""
        if not hasattr(self, 'weight_position') or self.latest_depth_msg is None:
            return
        
        # Check for target position and shutdown if stable
        weight_error = self.weight_target_position - self.weight_position
        if abs(weight_error) < 1e-3 and not self.has_reached_target_position:
            self.get_logger().info(f"Target weight position {self.weight_target_position} meters reached. Stopping movement...")
            self.set_weight_velocity(0.0)
            self.has_reached_target_position = True
            self.shutdown_triggered = True
            rclpy.shutdown()

        self.get_logger().info(f"Weight position: {self.weight_position:.3f} meters")
        self.get_logger().info(f"Thruster position: {self.bladder_position:.3f} radians")

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
