import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('marker_publisher')
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        self.timer = self.create_timer(0.1, self.publish_marker)

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = "camera"  # Change this to the desired frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "test_marker"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # Position the marker in front of the camera
        marker.pose.position.x = 0.5  # Distance in the X-direction (e.g., 0.5 meters)
        marker.pose.position.y = 0.0  # Centered in Y
        marker.pose.position.z = 0.0  # Height (e.g., 1.0 meters above ground)
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # Define the marker's size and color
        marker.scale.x = 0.1  # Sphere diameter
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0  # Alpha (transparency)
        marker.color.r = 1.0  # Red
        marker.color.g = 0.0
        marker.color.b = 0.0

        self.marker_pub.publish(marker)
        self.get_logger().info("Published marker!")

def main(args=None):
    rclpy.init(args=args)
    node = MarkerPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
