import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from gyakuenki_interfaces.msg import ProjectedObjects
from geometry_msgs.msg import Point

class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('marker_publisher')
        self.object_position_sub = self.create_subscription(ProjectedObjects, 'gyakuenki/projected_dnn', self.publish_marker, 10)
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)

    def publish_marker(self, msg: ProjectedObjects):
        for obj in msg.projected_objects:
            if obj.label == "ball":
                marker = Marker()
                marker.header.frame_id = "camera"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "test_marker"
                marker.id = 0
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD

                # Position the marker in front of the camera
                marker.pose.position.x = msg.center.x  # Distance in the X-direction (e.g., 0.5 meters)
                marker.pose.position.y = msg.center.y  # Centered in Y
                marker.pose.position.z = msg.center.z  # Height (e.g., 1.0 meters above ground)
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
