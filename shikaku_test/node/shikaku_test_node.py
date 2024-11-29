import rclpy
from tachimawari_interfaces.msg import CurrentJoints
# from kansei_interfaces.msg import Status
from sensor_msgs.msg import JointState

class ShikakuTestNode:
  def __init__(self, node):
    self.node = node
    
    self.joint_names = {
      1: "right_shoulder_pitch",
      2: "left_shoulder_pitch",
      3: "right_shoulder_roll",
      4: "left_shoulder_roll",
      5: "right_elbow",
      6: "left_elbow",
      7: "right_hip_yaw",
      8: "left_hip_yaw",
      9: "right_hip_roll",
      10: "left_hip_roll",
      11: "right_hip_pitch",
      12: "left_hip_pitch",
      13: "right_knee",
      14: "left_knee",
      15: "right_ankle_pitch",
      16: "left_ankle_pitch",
      17: "right_ankle_roll",
      18: "left_ankle_roll",
      19: "neck_yaw",
      20: "neck_pitch"
    }
    
    self.fixed_joints = [
      "odom_joint",
      "camera_joint",
      "left_foot_joint",
      "right_foot_joint",
    ]
    
    self.joint_directions = {
      1: 1,
      2: 1,
      3: -1,
      4: -1,
      5: 1,
      6: 1,
      7: -1,
      8: -1,
      9: 1,
      10: 1,
      11: 1,
      12: 1,
      13: 1,
      14: 1,
      15: 1,
      16: 1,
      17: 1,
      18: 1,
      19: 1,
      20: 1
    }

    self.current_joints_subscription = self.node.create_subscription(
      CurrentJoints, "joint/current_joints", 
      lambda msg: [self.publish_joint_state(msg.joints)], 10)
    
    self.joint_state_publisher = self.node.create_publisher(JointState, "joint_states", 10)
    
  def angle_to_radian(self, angle):
    return angle * 3.14159 / 180

  def publish_joint_state(self, joints):
    print(joints)

    msg = JointState()
    for joint in joints:
      msg.name.append(self.joint_names[joint.id])
      msg.position.append(self.angle_to_radian(joint.position) * self.joint_directions[joint.id])
    
    # add fixed joints
    for joint in self.fixed_joints:
      msg.name.append(joint)
      msg.position.append(0)
      
    msg.header.stamp = self.node.get_clock().now().to_msg()
    
    self.joint_state_publisher.publish(msg)
    