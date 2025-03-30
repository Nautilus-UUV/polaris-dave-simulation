from rclpy.node import Node
from rclpy import shutdown, init, spin 
from sensor_msgs.msg import FluidPressure
import subprocess

glider = "glider_nautilus"

# CONSTANTS

# p=p0+k*z
__std_pressure__ = 101.325  # kPa
__k__ = 9.80638 # kPa/m

class BadderControlUnit(Node):
    def __init__(self):
        super().__init__('bladder_monitor')

        # Inputs
        self.target_depth = 4
        self.is_stroke_downward = True

        # Bladder Parameters
        self.bladder_topic = f"/model/{glider}/joint/bladder_joint/cmd_thrust"
        self.neutral_thrust = -35.75  # Known neutral thrust
        self.down_thrust = -25  # Minimum thrust differential (downwards)
        self.up_thrust = +25  # Maximum thrust differential (upwards)

        # Code Parameters
        self.reached_target_depth = False
        self.shutdown_triggered = False
        self.callback_frequency = 1e-5  # Frequency in seconds

        # Overshooting counterFalse
        self.overshoot_counter = 1

        self.current_depth = None

        # Subscription to Pressure
        self.subscription = self.create_subscription(
            FluidPressure,
            f"/model/{glider}/sea_pressure",
            self.pressure_callback,
            10
        )

        self.timer = self.create_timer(self.callback_frequency, self.process_depth)

        # Initialize bladder thrust
        if self.is_stroke_downward: self.current_thrust = self.neutral_thrust + self.down_thrust
        else: self.current_thrust = self.neutral_thrust + self.up_thrust
        self.set_bladder(self.current_thrust)
        self.get_logger().info(f"Initial bladder set to {self.current_thrust}.")

    def set_bladder(self, bladder_value):
        """Send bladder command to Gazebo topic."""
        bladder_value = max(self.neutral_thrust + self.down_thrust, min(bladder_value, self.neutral_thrust + self.up_thrust))  # Clamp thrust within limits
        bladder_command = f"gz topic -t {self.bladder_topic} -m gz.msgs.Double -p 'data: {bladder_value}'"
        subprocess.run(bladder_command, shell=True)
        self.get_logger().info(f"Bladder set to {bladder_value:.2f}.")
        self.current_thrust = bladder_value

    def pressure_callback(self, msg):
        """Store the latest depth message."""
        # Access the fluid pressure from the message
        pressure = msg.fluid_pressure  # This is the pressure value in Pascals
        # Convert pressure from Pascals to kPa
        # Calculate depth from pressure
        self.current_depth = (pressure - __std_pressure__) / __k__

    def direct_thrust_control(self):
        if self.current_depth > self.target_depth:
            self.get_logger().info(f"Target depth overshot. Slown down factor: {self.overshoot_counter}.")
            self.set_bladder(self.neutral_thrust + self.up_thrust/(2**self.overshoot_counter))
            self.overshoot_counter += 1
        else:
            self.get_logger().info(f"Target depth overshot. Slown down factor: {self.overshoot_counter}.")
            self.set_bladder(self.neutral_thrust + self.down_thrust/(2**self.overshoot_counter))
            self.overshoot_counter += 1

    def PID_loop(self):
        return

    def stop(self):
        # Equilibrium reached?
        depth_error = self.target_depth - self.current_depth

        # Stop bladder adjustment if target depth is reached and maintain neutral thrust
        if abs(depth_error) < 1e-3:
            self.get_logger().info(f"Target depth {self.target_depth} meters reached. Maintaining position.")
            self.set_bladder(self.neutral_thrust)
            self.shutdown_triggered = True
            shutdown()


    def process_depth(self):
        """Process depth and adjust bladder thrust."""
        if not hasattr(self, 'current_depth') or self.current_depth is None:
            return

        # Has reached target depth?
        if self.is_stroke_downward:
            if not self.reached_target_depth and self.current_depth > self.target_depth:
                self.reached_target_depth = True
        else:
            if not self.reached_target_depth and self.current_depth < self.target_depth:
                self.reached_target_depth = True

        if self.reached_target_depth:
            self.direct_thrust_control()
            self.stop()

        # Depth logger
        self.get_logger().info(f"Current depth: {self.current_depth:.4f} meters")

def main(args=None):
    init(args=args)

    bladder = BadderControlUnit()

    try:
        spin(bladder)
    except KeyboardInterrupt:
        bladder.get_logger().info("Bladder monitoring interrupted by user.")
    finally:
        if not bladder.shutdown_triggered:
            bladder.destroy_node()
            shutdown()

if __name__ == '__main__':
    main()
