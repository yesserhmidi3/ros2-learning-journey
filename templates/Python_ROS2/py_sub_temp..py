import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32  # Change to the message type you expect

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(
            Int32, 
            'counter', 
            self.listener_callback, 
            10
        )
        self.subscription  # Avoid unused variable warning
        self.get_logger().info("Subscriber node started")

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')
        # You can add logic here to process incoming data

def main():
    rclpy.init()
    node = SubscriberNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
