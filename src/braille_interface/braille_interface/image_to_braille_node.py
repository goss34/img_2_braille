import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import UInt8MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np

from braille_interface.display import camera_to_edges

from rclpy.qos import qos_profile_sensor_data

class ImageToBrailleNode(Node):
    def __init__(self):
        super().__init__('image_to_braille_node')

        self.subscription = self.create_subscription(
            Image,
            '/camera0/color/image_raw',
            self.image_callback,
            qos_profile_sensor_data)

        self.publisher = self.create_publisher(UInt8MultiArray, '/braille_image', 10)
        self.debug_publisher = self.create_publisher(Image, '/braille_image_debug', 10)

        self.bridge = CvBridge()
        self.get_logger().info("ImageToBrailleNode started.")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CV Bridge conversion failed: {e}")
            return

        # Grayscale, resize to board resolution, blur, Canny edges
        binary, debug_stack = camera_to_edges(cv_image)
        msg_out = UInt8MultiArray()
        msg_out.data = binary.flatten().tolist()
        self.publisher.publish(msg_out)

        # --- Composite Debug Image ---
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_stack, encoding='mono8')
            self.debug_publisher.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f"Debug image generation failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageToBrailleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
