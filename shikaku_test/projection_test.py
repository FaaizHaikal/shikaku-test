import rclpy
import numpy as np
from rclpy.node import Node
import tf2_ros as tf2
from geometry_msgs.msg import Quaternion
from gyakuenki_interfaces.msg import ProjectedObjects, ProjectedObject
from ninshiki_interfaces.msg import DetectedObjects
from scipy.spatial.transform import Rotation

class ProjectionTest(Node):
  def __init__(self):
    super().__init__('projection_test')
    
    self.camera_frame = 'camera'
    self.base_footprint_frame = 'base_footprint'
    
    self.K = np.array([[232.73994455, 0, 148.08704113], [0, 232.29722415, 116.9780848], [0, 0, 1]])
    
    self.subscription = self.create_subscription(DetectedObjects, 'ninshiki_cpp/dnn_detection', self.detected_objects_callback, 10)
    
    self.publisher = self.create_publisher(ProjectedObjects, 'gyakuenki/projected_dnn', 10)
    
    self.tf_buffer = tf2.Buffer()
    self.tf_listener = tf2.TransformListener(self.tf_buffer, self)
    
    self.get_logger().info('Projection Test Node has been initialized')
  
  def get_latest_common_time(self, target_frame: str, source_frame: str) -> rclpy.time.Time:
    try:
      return self.tf_buffer.get_latest_common_time(target_frame, source_frame)
    except tf2.LookupException as e:
      raise ValueError(f"Error while getting latest common time: {e}, source_frame: {source_frame}, target_frame: {target_frame}")

  def quaternion_to_rotation_matrix(self, quaternion: Quaternion) -> np.ndarray:
    # Convert quaternion to [x, y, z, w] array
    q = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    q = q / np.linalg.norm(q)

    # Create a rotation object and return the rotation matrix
    return Rotation.from_quat(q).as_matrix()
    
  def detected_objects_callback(self, msg: DetectedObjects):
    projected_objects = ProjectedObjects()
    
    for detected_object in msg.detected_objects:
      try:
        latest_time = self.get_latest_common_time(self.base_footprint_frame, self.camera_frame)
        
        transform = self.tf_buffer.lookup_transform(self.base_footprint_frame, self.camera_frame, latest_time)
        
        # Extract R and T
        R = self.quaternion_to_rotation_matrix(transform.transform.rotation)
        T = np.array([transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z])
        
        fx = self.K[0, 0]
        fy = self.K[1, 1]
        cx = self.K[0, 2]
        cy = self.K[1, 2]
      
        object_center_x = detected_object.left + detected_object.right / 2
        object_center_y = detected_object.top + detected_object.bottom / 2

        x = (object_center_x - cx) / fx
        y = (object_center_y - cy) / fy
        
        object_diameter = 0.135 # Ball diameter in meters
        height_offset = object_diameter / 2
        
        denominator = R[2, 0] * x + R[2, 1] * y + R[2, 2]
        if np.isclose(denominator, 0.0):
          raise ValueError("Denominator in Z_c computation is zero or too small.")
          
        Z_c = (height_offset - T[2]) / (R[2, 0] * x + R[2, 1] * y + R[2, 2])
        
        P_c = np.array([x * Z_c, y * Z_c, Z_c])
        
        P_b = np.dot(R, P_c) + T
        
        projected_object = ProjectedObject()
        projected_object.center.x = P_b[0]
        projected_object.center.y = P_b[1]
        projected_object.center.z = P_b[2]
        
        projected_object.label = detected_object.label
    
        projected_objects.projected_objects.append(projected_object)
        
      except Exception as e:
        self.get_logger().error(f"{e}")
        return
    
    self.publisher.publish(projected_objects)
    
def main(args=None):
  rclpy.init(args=args)
  node = ProjectionTest()
  rclpy.spin(node)
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
        