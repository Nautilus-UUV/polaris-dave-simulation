from rclpy.node import Node
from rclpy import init, spin, shutdown
from std_msgs.msg import Float64
from sensor_msgs.msg import FluidPressure
import subprocess
from json import load

with open('config.json', 'r') as f:
    config = load(f)

glider = config['glider_name']

class MCU(Node):
    def __init__(self, target_depth, is_stroke_downward):
        super().__init__('mcu')

        # Inputs
        self.target_depth = target_depth
        self.is_stroke_downward = is_stroke_downward
        
        # Bladder Parameters
        self.topic_cmd_thrust = f"/model/{glider}/joint/bladder_joint/cmd_thrust"
        self.neutral = config['neutral_thrust']  # Neutral thrust
        self.down = -1 * config['bladder_thrust_differential']  # Downward thrust limit
        self.up = config['bladder_thrust_differential']  # Upward thrust limit
        self.bladder_period = config['bladder_period']  # Transition period

        # Thrust state
        self.thrust = self.neutral

        self.start_thrust = self.thrust

        self.target_thrust = self.thrust
        self.last_target_thrust = self.target_thrust

        self.current_time = None
        self.start_cycle = None

        # Topics
        self.callback_frequency = 1e-1  # Frequency in seconds

        self.create_subscription(Float64, '/global_time', self.time_callback, 10)
        self.create_subscription(FluidPressure, f"/model/{glider}/sea_pressure", self.pressure_callback, 10)

        self.target_thrust_publisher = self.create_publisher(Float64, '/target_thrust', 10)
        self.create_timer(self.callback_frequency, self.publish_target_thrust) 

        # PID Parameters
        self.Kp = 20.0   # Proportional gain
        self.Ki = 1.0    # Integral gain
        self.Kd = 10.0    # Derivative gain

        self.prev_error = 0.0
        self.integral = 0.0
        self.last_pid_time = None  # Last time the PID was updated

        # Code Parameters
        self.reached_target_depth = False
        self.shutdown_triggered = False

        # Overshooting counterFalse
        self.overshoot_counter = 1

        self.current_depth = None

        # Motion Control Loop
        self.create_timer(self.callback_frequency, self.motion_control)

    ###########################################################
    ### Topics
    ###########################################################

    def time_callback(self, msg): self.current_time = msg.data

    def pressure_callback(self, msg):
        pressure = msg.fluid_pressure
        self.current_depth = (pressure - config["std_pressure"]) / config["k"]

    def publish_target_thrust(self):
        msg = Float64()
        msg.data = self.target_thrust
        self.target_thrust_publisher.publish(msg)

    ###########################################################
    ### PID loop
    ###########################################################

    def PID_loop(self):
        """PID controller to """
        if self.last_pid_time is None or self.prev_error is None:
            self.prev_error = self.target_depth - self.current_depth
            self.last_pid_time = self.current_time
            return
        
        # Time step calculation
        delta_time = self.current_time - self.last_pid_time

        # Compute error
        error = self.target_depth - self.current_depth
        self.integral += error * delta_time  # Accumulate integral
        derivative = (error - self.prev_error) / delta_time  # Compute derivative

        self.get_logger().info(f"Error: {error:.4f}")
        self.get_logger().info(f"Integral: {self.integral:.4f}")
        self.get_logger().info(f"Derivative: {derivative:.4f}")


        # PID output
        pid_output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)

        # Update last error and time
        self.prev_error = error
        self.last_pid_time = self.current_time

        # Determine Target Thrust
        if self.is_stroke_downward:
            self.target_thrust = self.neutral - pid_output
        else:
            self.target_thrust = self.neutral + pid_output


    ###########################################################
    ### Stop
    ###########################################################

    def stop(self):
        # Equilibrium reached?
        depth_error = self.target_depth - self.current_depth

        # Stop bladder adjustment if target depth is reached and maintain neutral thrust
        if abs(depth_error) < 1e-3:
            self.get_logger().info(f"Target depth {self.target_depth} meters reached. Stopping.")
            self.shutdown_triggered = True


    ###########################################################
    ### Motion
    ###########################################################

    def motion_control(self):
        """Motion control loop."""

        self.PID_loop()
        #self.stop()

        # Depth logger
        self.get_logger().info(f"Current depth: {self.current_depth:.4f} meters")

def main(args=None):
    init(args=args)

    bladder = MCU(target_depth=1, is_stroke_downward=True)

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