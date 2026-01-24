import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class RVizPublisher(Node):
    def __init__(self):
        super().__init__('rviz_publisher_node')

        # Create a publisher to the "visualization_marker" topic
        self.publisher_ = self.create_publisher(Marker, 'visualization_marker', 10)
        self.get_logger().info("RViz2 Marker Publisher Node Started")

        # Publish marker every 0.5 seconds
        self.timer_ = self.create_timer(0.5, self.publish_marker)
        self.counter = 0.0

    def publish_marker(self):
        marker = Marker()

        # === Header ===
        marker.header.frame_id = "map"  # Options: "map", "odom", "base_link", "camera_link", etc.
        marker.header.stamp = self.get_clock().now().to_msg()  # Current time

        # === Namespace and ID ===
        marker.ns = "demo_marker"  # Namespace for grouping markers
        marker.id = 0  # Unique ID per marker (important for updating/removing markers)

        # === Marker type ===
        # Options: SPHERE, CUBE, ARROW, CYLINDER, LINE_STRIP, LINE_LIST, CUBE_LIST, SPHERE_LIST, POINTS, TEXT_VIEW_FACING, MESH_RESOURCE
        marker.type = Marker.SPHERE  

        # === Marker action ===
        # Options: ADD (show/update marker), DELETE (remove marker), DELETEALL (remove all in namespace)
        marker.action = Marker.ADD

        # === Position and orientation ===
        marker.pose.position.x = self.counter  # X coordinate
        marker.pose.position.y = 0.0           # Y coordinate
        marker.pose.position.z = 0.5           # Z coordinate (height)
        marker.pose.orientation.x = 0.0        # Quaternion orientation
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0        # No rotation

        # === Scale ===
        marker.scale.x = 0.2  # Width
        marker.scale.y = 0.2  # Depth
        marker.scale.z = 0.2  # Height
        # Note: For LINE_STRIP, LINE_LIST -> scale.x = line width
        # For POINTS -> scale.x = scale.y = point size

        # === Color ===
        marker.color.r = 1.0  # Red channel (0.0-1.0)
        marker.color.g = 0.0  # Green channel
        marker.color.b = 0.0  # Blue channel
        marker.color.a = 1.0  # Alpha (transparency, 0=transparent, 1=opaque)

        # === Lifetime ===
        # Duration the marker stays visible before auto-removal
        # Use 0 for forever
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0

        # Publish the marker
        self.publisher_.publish(marker)
        self.get_logger().info(f"Published marker at x={self.counter:.2f}")

        # === Example animation ===
        # Move marker along x-axis
        self.counter += 0.1
        if self.counter > 5.0:
            self.counter = 0.0

def main():
    rclpy.init()
    node = RVizPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
