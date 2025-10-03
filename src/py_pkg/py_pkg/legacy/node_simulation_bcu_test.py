import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import time


class TestBCU(Node):
    def __init__(self):
        super().__init__('node_simulation_bcu_test')

        self.create_subscription(Float64, '/global_time', self.time_callback, 10)
        self.thrust_pub = self.create_publisher(Float64, '/target_thrust', 10)

        self.sim_time = 0.0

    def time_callback(self, msg):
        """Update the current simulation time."""
        self.current_time = msg.data

    def send_thrust(self, thrust):
        """Send a new target thrust command."""
        msg = Float64()
        msg.data = thrust
        self.thrust_pub.publish(msg)
        self.get_logger().info(f"Sent thrust command: {thrust:.2f}")



def main():
    rclpy.init()
    tester = TestBCU()
    
    try:
        time.sleep(1)  # Allow node to initialize

        # Gradually test thrust adjustments
        tester.send_thrust(5.0)
        time.sleep(3)

        tester.send_thrust(-3.0)
        time.sleep(3)

        tester.send_thrust(0.0)  # Return to neutral
        time.sleep(3)

    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
