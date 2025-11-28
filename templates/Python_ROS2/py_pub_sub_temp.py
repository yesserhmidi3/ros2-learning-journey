import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32  # Change to any message type you need

class PubSubNode(Node):
    def __init__(self):
        super().__init__('pubsub_node')

        # Publisher
        self.publisher_ = self.create_publisher(Int32, 'counter', 10)
        self.timer_ = self.create_timer(1.0, self.publish_callback)
        self.counter = 0

        # Subscriber
        self.subscription = self.create_subscription(
            Int32,
            'counter',
            self.listener_callback,
            10
        )
        self.subscription
        self.get_logger().info("PubSub node started")

    def publish_callback(self):
        msg = Int32()
        msg.data = self.counter
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.counter += 1

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')

def main():
    rclpy.init()
    node = PubSubNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
