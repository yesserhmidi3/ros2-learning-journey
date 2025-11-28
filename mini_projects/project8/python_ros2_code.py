import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Bool 


class DistanceToLED(Node):
    def __init__(self):
        super().__init__('DistanceToLED')

        # Publisher
        self.publisher_ = self.create_publisher(Bool, 'led', 10)

        # Subscriber
        self.subscription = self.create_subscription(
            Float32,
            'distance',
            self.listener_callback,
            10
        )
        self.subscription
        self.get_logger().info("PubSub node started")
        self.led_state = False


    def publish_callback(self):
        msg = Bool()
        msg.data = self.led_state
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published LED state: {msg.data}')

    def listener_callback(self, msg):
        new_state = msg.data < 25
        if new_state != self.led_state:
            self.led_state = new_state
            self.publish_callback()
        
        
    

def main():
    rclpy.init()
    node = DistanceToLED()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
