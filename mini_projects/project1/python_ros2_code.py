import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class CounterSubscriber(Node):
    def __init__(self):
        super().__init__('esp32_counter_receiver')

        # Subscribe to the "counter" topic (published by ESP32)
        self.subscription = self.create_subscription(
            Int32,
            'counter',
            self.callback,
            10
        )

    # Runs every time a new message arrives
    def callback(self, msg):
        self.get_logger().info(f'Received counter value: {msg.data}')


def main():
    rclpy.init()
    node = CounterSubscriber()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

