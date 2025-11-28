import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32  # Change to any message type you need

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher_ = self.create_publisher(Int32, 'counter', 10)
        self.get_logger().info("Publisher node started")

        # Timer for publishing (every 1 second)
        self.timer_ = self.create_timer(1.0, self.publish_callback)
        self.counter = 0

    def publish_callback(self):
        msg = Int32()
        msg.data = self.counter  # Replace with dynamic data if needed
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.counter += 1

def main():
    rclpy.init()
    node = PublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
