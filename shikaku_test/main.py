import rclpy

from shikaku_test.node.shikaku_test_node import ShikakuTestNode

def main():
  rclpy.init()
  node = rclpy.create_node("shikaku_test_node")
  shikaku_test_node = ShikakuTestNode(node)
  rclpy.spin(shikaku_test_node.node)
  
  node.destroy_node()
  rclpy.shutdown()

if __name__ == "__main__":
  main()