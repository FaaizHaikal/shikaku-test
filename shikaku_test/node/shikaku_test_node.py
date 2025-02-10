import rclpy
import tf2_ros
from tachimawari_interfaces.msg import CurrentJoints
from tachimawari_interfaces.msg import SetJoints
from kansei_interfaces.msg import Status
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped

from shikaku_test.utils.utils import joint_names, fixed_joints, joint_directions

class ShikakuTestNode:
  def __init__(self, node):
    self.node = node
    self.joints_msg = None
    self.tf_msg = None

    self.current_joints_subscription = self.node.create_subscription(
      SetJoints, "joint/set_joints", 
      lambda msg: [self.update_joints(msg.joints)], 10)
    
    # self.kansei_status_subscription = self.node.create_subscription(
    #   Status, "measurement/status",
    #   lambda msg: [self.update_imu_tf(msg.orientation)], 10)
    
    self.joint_state_publisher = self.node.create_publisher(JointState, "joint_states", 10)
    
    # self.tf_buffer = tf2_ros.Buffer()
    # self.tf_broadcaster = tf2_ros.TransformBroadcaster(self.node, self.tf_buffer)
    
    self.node_timer = self.node.create_timer(0.008, self.update)
    
  def angle_to_radian(self, angle):
    return angle * 3.14159 / 180
  
  def update_joints(self, joints):
    self.joints_msg = JointState()
 
    for joint in joints:
      self.joints_msg.name.append(joint_names[joint.id])
      self.joints_msg.position.append(self.angle_to_radian(joint.position) * joint_directions[joint.id])
    
    for joint in fixed_joints:
      self.joints_msg.name.append(joint)
      self.joints_msg.position.append(0)

    self.joints_msg.header.stamp = self.node.get_clock().now().to_msg()
  
  def update_imu_tf(self, imu_data):
    # t = TransformStamped()
    # t.header.stamp = self.node.get_clock().now().to_msg()
    # t.header.frame_id = "odom"
    # t.child_frame_id = "body"
    # t.transform.translation.x = 0
    # t.transform.translation.y = 0
    # t.transform.translation.z = 0
    # t.transform.rotation.x = imu_data.pitch
    # t.transform.rotation.y = imu_data.roll
    # t.transform.rotation.z = imu_data.yaw
    # t.transform.rotation.w = 0.0  # Assuming w is 0 if not provided
    
    # self.tf_msg = [t]
    return
    
  def update(self):
    if (self.joints_msg):
      self.joint_state_publisher.publish(self.joints_msg)
    # self.tf_broadcaster.sendTransform(self.tf_msg)