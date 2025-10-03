from rclpy.node import Node
from rclpy import init, spin, shutdown
from std_msgs.msg import Float64
import subprocess
from json import load

with open('config.json', 'r') as f:
    config = load(f)

glider = config['glider_name']

class SimulationBCU(Node):
    def __init__(self):
        super().__init__('node_simulation_bcu')
        
        # Bladder Parameters
        self.topic_cmd_thrust = f"/model/{glider}/joint/bladder_joint/cmd_thrust"
        self.neutral = config['neutral_thrust']  # Neutral thrust
        self.max_delta_thrust = config['bladder_thrust_differential']
        self.bladder_period = config['bladder_period']  # Transition period

        # Thrust state
        self.thrust = self.neutral

        self.start_thrust = self.thrust

        self.target_thrust = self.thrust
        self.last_target_thrust = self.target_thrust

        self.current_time = 0.0
        self.start_cycle = None

        # Subscriptions
        self.create_subscription(Float64, '/global_time', self.time_callback, 10)
        self.create_subscription(Float64, '/target_thrust', self.target_thrust_callback, 10)

        # Timer to handle gradual thrust adjustment
        self.timer = self.create_timer(0.01, self.update_thrust)

    def time_callback(self, msg):
        """Update the current simulation time."""
        self.current_time = msg.data

    def target_thrust_callback(self, msg):
        """Receive target thrust and initiate transition."""

        # Limit the target thrust to the bladder limits
        if msg.data < self.neutral - self.max_delta_thrust:
            self.target_thrust = self.neutral - self.max_delta_thrust
        elif msg.data > self.neutral + self.max_delta_thrust:
            self.target_thrust = self.neutral + self.max_delta_thrust
        else:
            self.target_thrust = msg.data

        # Reset the timer if the target thrust changes
        if abs(self.last_target_thrust - self.target_thrust) > 1e-3: 
            print("Resetting timer")
            self.start_thrust = self.thrust
            self.start_cycle = self.current_time

        self.get_logger().info(f"Last target thrust: {self.last_target_thrust:.2f}")
        self.get_logger().info(f"New target thrust: {self.target_thrust:.2f}")
        self.last_target_thrust = self.target_thrust
    
    def update_thrust(self):
        """Gradually adjust the thrust over time."""
        if self.start_cycle is None or (abs(self.target_thrust - self.neutral) < 1e-3 and abs(self.thrust - self.neutral) < 1e-3):
            return # No need to adjust thrust if thrust and target thrust is neutral
        
        elapsed_time = self.current_time - self.start_cycle
        if elapsed_time >= self.bladder_period:
            self.set_bladder(self.target_thrust)
            self.start_thrust = self.neutral
            self.target_thrust = self.neutral
        else:
            new_thrust = self.start_thrust + (self.target_thrust - self.start_thrust) * (elapsed_time / self.bladder_period)
            self.set_bladder(new_thrust)

    def set_bladder(self, thrust):
        """Send bladder command to Gazebo topic."""
        self.thrust = thrust
        subprocess.run(f"gz topic -t {self.topic_cmd_thrust} -m gz.msgs.Double -p 'data: {thrust}'", shell=True)
        self.get_logger().info(f"Bladder thrust set to {thrust:.2f}")



def main():
    init()
    node = SimulationBCU()
    try:
        spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("BCU simulation shutting down.")
    finally:
        node.destroy_node()
        shutdown()

if __name__ == '__main__':
    main()
