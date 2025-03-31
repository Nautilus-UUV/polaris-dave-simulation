from rclpy import shutdown, init, spin 
from rclpy.node import Node
from std_msgs.msg import Float64
from time import time

class TopicTime(Node):
    def __init__(self):
        super().__init__('topic_time')
        self.publisher = self.create_publisher(Float64, '/global_time', 10)
        self.start_time = time()
        self.timer = self.create_timer(0.1, self.publish_time)  # Publish every 0.1 seconds

    def publish_time(self):
        elapsed_time = Float64()
        elapsed_time.data = time() - self.start_time
        self.publisher.publish(elapsed_time)
        self.get_logger().info(f'Global Time: {elapsed_time.data:.2f} seconds')

def main():
    init()
    node = TopicTime()
    try:
        spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Time topic shutting down.")
    finally:
        node.destroy_node()
        shutdown()

if __name__ == '__main__':
    main()