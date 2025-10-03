from rclpy.node import Node
from rclpy import shutdown, init, spin 
from std_msgs.msg import Float64
from sensor_msgs.msg import FluidPressure
import subprocess

glider = "glider_nautilus"

# CONSTANTS

# p=p0+k*z
__std_pressure__ = 101.325  # kPa
__k__ = 9.80638 # kPa/m

# Glider parameters
__bladder_period__ = 5  # Time to inflate or deflate bladder
__bladder_thrust_differential__ = 5  # Maximum thrust differential
__neutral_thrust__ = -35.75  # Neutral thrust

class BadderControlUnit(Node):
    def __init__(self):
        super().__init__('maintain_dive')

        # Inputs
        self.target_depth = 0.6
        self.is_stroke_downward = True

        # Bladder Parameters
        self.bladder_topic = f"/model/{glider}/joint/bladder_joint/cmd_thrust"
        self.neutral_thrust = __neutral_thrust__  # Known neutral thrust
        self.down_thrust = -1*__bladder_thrust_differential__  # Minimum thrust differential (downwards)
        self.up_thrust = __bladder_thrust_differential__  # Maximum thrust differential (upwards)

        # Initialize bladder thrust
        if self.is_stroke_downward: self.current_thrust = self.neutral_thrust + self.down_thrust
        else: self.current_thrust = self.neutral_thrust + self.up_thrust
        self.set_bladder(self.current_thrust)
        self.get_logger().info(f"Initial bladder set to {self.current_thrust}.")

        # Global time tracking
        self.current_time = 0.0
        self.last_update_time = 0.0  # Store last update time for gradual adjustment
        self.start_thrust = None
        self.target_thrust = None
        self.transition_duration = 0  # Time over which to change thrust

        # Time subscription
        self.create_subscription(
            Float64, 
            '/global_time', 
            self.time_callback, 
            10
        )

        # PID Parameters
        self.Kp = 10.0   # Proportional gain
        self.Ki = 1.0    # Integral gain
        self.Kd = 5.0    # Derivative gain

        self.prev_error = 0.0
        self.integral = 0.0
        self.last_pid_time = 0.0  # Last time the PID was updated

        # Code Parameters
        self.reached_target_depth = False
        self.shutdown_triggered = False
        self.callback_frequency = 1e-1  # Frequency in seconds

        # Overshooting counterFalse
        self.overshoot_counter = 1

        self.current_depth = None

        # Subscription to Pressure
        self.create_subscription(
            FluidPressure,
            f"/model/{glider}/sea_pressure",
            self.pressure_callback,
            10
        )

        self.timer = self.create_timer(self.callback_frequency, self.motion_control)

    ###########################################################
    ### Gradual bladder
    ###########################################################

    def time_callback(self, msg):
        """Update the global time."""
        self.current_time = msg.data

    def start_gradual_thrust_adjustment(self, mode, duration=__bladder_period__):
        """Change bladder state: 'inflating', 'stationary', or 'deflating'."""
        if mode == "inflating":
            self.target_thrust = self.neutral_thrust + self.up_thrust
        elif mode == "deflating":
            self.target_thrust = self.neutral_thrust + self.down_thrust
        elif mode == "stationary":
            self.target_thrust = self.current_thrust  # Hold current thrust

        self.start_thrust = self.current_thrust
        self.transition_duration = duration
        self.last_update_time = self.current_time  # Mark the start time

    def update_thrust(self):
        """Gradually adjust the thrust based on elapsed time since the last update."""
        if self.start_thrust is None or self.target_thrust is None:
            self.get_logger().warn("⚠️ No transition in progress.")
            return  # No transition in progress

        elapsed_time = self.current_time - self.last_update_time

        # Debug: Print the state of the thrust transition
        self.get_logger().info(f"🕒 Elapsed Time: {elapsed_time:.4f}s / {self.transition_duration}s")
        self.get_logger().info(f"🔄 Transitioning from {self.start_thrust:.2f} -> {self.target_thrust:.2f}")

        if elapsed_time >= self.transition_duration:
            self.set_bladder(self.target_thrust)
            self.start_thrust = None  # Stop adjusting
            self.target_thrust = None
            self.get_logger().info(f"✅ Thrust transition completed. New thrust: {self.current_thrust:.2f}")
        else:
            # Linear interpolation between start and target thrust
            new_thrust = self.start_thrust + (self.target_thrust - self.start_thrust) * (elapsed_time / self.transition_duration)
            self.set_bladder(new_thrust)
            self.get_logger().info(f"➡️ Gradual adjustment: {new_thrust:.2f}")


    def PID_loop(self):
        """PID controller to decide whether to inflate, deflate, or stay stationary."""
        if self.current_depth is None or self.current_time == 0.0:
            self.get_logger().warn("⚠️ No valid depth reading yet.")
            return  # No valid depth reading yet

        # Time step calculation
        delta_time = self.current_time - self.last_pid_time
        if delta_time <= 0:
            self.get_logger().warn("⚠️ Invalid delta_time, skipping PID update.")
            return  # Prevent division by zero

        # Compute error
        error = self.target_depth - self.current_depth
        self.integral += error * delta_time  # Accumulate integral
        derivative = (error - self.prev_error) / delta_time  # Compute derivative

        # Compute PID output (decision)
        pid_output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)

        # Debug: Log all PID values
        self.get_logger().info(f"📊 PID DATA -> Error: {error:.4f}, Integral: {self.integral:.4f}, Derivative: {derivative:.4f}, PID Output: {pid_output:.2f}")

        # Update last error and time
        self.prev_error = error
        self.last_pid_time = self.current_time

        # Determine bladder action
        if pid_output < -1:
            self.get_logger().info("🔺 PID Decision: INFLATING")
            self.start_gradual_thrust_adjustment("inflating")
        elif pid_output > 1:
            self.get_logger().info("🔻 PID Decision: DEFLATING")
            self.start_gradual_thrust_adjustment("deflating")
        else:
            self.get_logger().info("⏸️ PID Decision: STATIONARY")
            self.start_gradual_thrust_adjustment("stationary")


    ###########################################################
    ### Abrupt bladder
    ###########################################################

    def set_bladder(self, bladder_value):
        """Send bladder command to Gazebo topic."""
        bladder_value = max(self.neutral_thrust + self.down_thrust, min(bladder_value, self.neutral_thrust + self.up_thrust))  # Clamp thrust within limits
        bladder_command = f"gz topic -t {self.bladder_topic} -m gz.msgs.Double -p 'data: {bladder_value}'"
        subprocess.run(bladder_command, shell=True)
        self.get_logger().info(f"Bladder set to {bladder_value:.2f}.")
        self.current_thrust = bladder_value

    def direct_thrust_control(self):
        if self.current_depth > self.target_depth:
            self.get_logger().info(f"Target depth overshot. Slown down factor: {self.overshoot_counter}.")
            self.set_bladder(self.neutral_thrust + self.up_thrust/(2**self.overshoot_counter))
            self.overshoot_counter += 1
        else:
            self.get_logger().info(f"Target depth overshot. Slown down factor: {self.overshoot_counter}.")
            self.set_bladder(self.neutral_thrust + self.down_thrust/(2**self.overshoot_counter))
            self.overshoot_counter += 1

    def stop(self):
        # Equilibrium reached?
        depth_error = self.target_depth - self.current_depth

        # Stop bladder adjustment if target depth is reached and maintain neutral thrust
        if abs(depth_error) < 1e-3:
            self.get_logger().info(f"Target depth {self.target_depth} meters reached. Maintaining position.")
            self.set_bladder(self.neutral_thrust)
            self.shutdown_triggered = True
            shutdown()

    ###########################################################
    ### Pressure
    ###########################################################

    def pressure_callback(self, msg):
        """Store the latest depth message."""
        # Access the fluid pressure from the message
        pressure = msg.fluid_pressure  # This is the pressure value in Pascals
        # Convert pressure from Pascals to kPa
        # Calculate depth from pressure
        self.current_depth = (pressure - __std_pressure__) / __k__

    ###########################################################
    ### Motion
    ###########################################################

    def motion_control(self):
        """Motion control loop."""
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
            self.get_logger().info("🎯 Target depth reached. Running PID loop.")
            self.PID_loop()
            self.update_thrust()

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
