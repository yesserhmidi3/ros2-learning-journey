import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from std_msgs.msg import Float32

from visualization_msgs.msg import Marker #For visualization in RViz


class multi_sensor_sub(Node):
    def __init__(self):
        super().__init__('multi_sensor_sub')
        self.pot_subscription = self.create_subscription(
            Int32, 
            'pot_value', 
            self.pot_callback, 
            10
        )
        self.button_subscription = self.create_subscription(
            Bool, 
            'button_state', 
            self.button_callback, 
            10
        )
        self.distance_subscription = self.create_subscription(
            Float32, 
            'distance_cm', 
            self.distance_callback, 
            10
        )
        self.get_logger().info("Subscriber node started")

        #publisher for RViz Marker
        self.pot_marker_pub = self.create_publisher(Marker, 'pot_marker', 10)
        self.button_marker_pub = self.create_publisher(Marker, 'button_marker', 10)
        self.distance_marker_pub = self.create_publisher(Marker, 'distance_marker', 10)

        self.get_logger().info("Publishers for RViz Markers created")

    def pot_callback(self, msg):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "potentiometer"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = (msg.data / 255.0) / 2.0  # center at half height
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = msg.data / 255.0  # height proportional to pot
        marker.color.a = 1.0
        marker.color.r = 1.0 # Red color
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.pot_marker_pub.publish(marker)


    def button_callback(self, msg):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "button"
        marker.id = 1
        marker.type = Marker.CUBE
        marker.pose.position.x = 0.3
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.05
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        if msg.data:  # button pressed
            marker.action = Marker.ADD
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        else:         # button released → hide marker
            marker.action = Marker.DELETE

        self.button_marker_pub.publish(marker)

    def distance_callback(self, msg):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "ultrasound"
        marker.id = 2
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = 0.6  # just offset in X so it doesn’t overlap
        marker.pose.position.y = 0.0
        marker.pose.position.z = (msg.data / 100.0) / 2.0  # center at half height
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = msg.data / 100.0  # height proportional to distance
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        self.distance_marker_pub.publish(marker)
    


def main():
    rclpy.init()
    node = multi_sensor_sub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()