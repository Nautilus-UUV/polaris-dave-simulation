import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class SlidingMassSubscriber(Node):
    def __init__(self):
        super().__init__('sliding_mass_subscriber')

        # Define the joint_state topic to subscribe to
        self.joint_state_topic = "/world/oceans_waves/model/glider_nautilus/joint_state"
        
        # Create a subscription to the joint_state topic
        self.subscription = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self.joint_state_callback,
            10  # Queue size
        )
        self.subscription  # prevent unused variable warning

    def joint_state_callback(self, msg):
        """Callback function to handle the received JointState message."""
        # Find the position of sliding_mass_joint
        try:
            # Assuming sliding_mass_joint is one of the joints
            # The position field contains all joint positions, so we'll search for sliding_mass_joint
            joint_names = msg.name  # List of joint names in the JointState message
            joint_positions = msg.position  # List of joint positions corresponding to joint_names
            
            # Find the index of sliding_mass_joint
            if 'sliding_mass_joint' in joint_names:
                index = joint_names.index('sliding_mass_joint')
                position = joint_positions[index]
                self.get_logger().info(f"Sliding mass position: {position:.2f} meters")
            else:
                self.get_logger().warn("sliding_mass_joint not found in the joint state.")
        
        except Exception as e:
            self.get_logger().error(f"Error processing joint state: {e}")

def main(args=None):
    rclpy.init(args=args)

    sliding_mass_subscriber = SlidingMassSubscriber()

    try:
        rclpy.spin(sliding_mass_subscriber)
    except KeyboardInterrupt:
        sliding_mass_subscriber.get_logger().info("Subscription interrupted by user.")
    finally:
        sliding_mass_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
