import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math

class FOVVisualizer(Node):
    def __init__(self):
        super().__init__('fov_visualizer')
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        self.timer = self.create_timer(0.5, self.publish_fov)

    def publish_fov(self):
        marker = Marker()
        marker.header.frame_id = "camera" 
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "camera_fov"
        marker.id = 1
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD

        fx = 775.8070010132403
        fy = 775.8950272011443
        img_width = 640
        img_height = 480

        # Calculate FOV angles
        fov_h = 2 * math.atan(img_width / (2 * fx))
        fov_v = 2 * math.atan(img_height / (2 * fy))

        # Frustum corners at 1m distance
        z = 1.0
        x_left = z * math.tan(fov_h / 2)
        x_right = -x_left
        y_top = z * math.tan(fov_v / 2)
        y_bottom = -y_top

        # Define frustum lines
        points = [
            [0.0, 0.0, 0.0], [x_left, y_top, z],    # Top left
            [0.0, 0.0, 0.0], [x_right, y_top, z],  # Top right
            [0.0, 0.0, 0.0], [x_left, y_bottom, z],  # Bottom left
            [0.0, 0.0, 0.0], [x_right, y_bottom, z],  # Bottom right
            [x_left, y_top, z], [x_right, y_top, z],  # Top edge
            [x_left, y_bottom, z], [x_right, y_bottom, z],  # Bottom edge
            [x_left, y_top, z], [x_left, y_bottom, z],  # Left edge
            [x_right, y_top, z], [x_right, y_bottom, z]  # Right edge
        ]

        for p in points:
            marker.points.append(Point(x=p[0], y=p[1], z=p[2]))

        marker.scale.x = 0.01  # Line width
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        self.marker_pub.publish(marker)
        self.get_logger().info("Published FOV marker!")

def main(args=None):
    rclpy.init(args=args)
    node = FOVVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
