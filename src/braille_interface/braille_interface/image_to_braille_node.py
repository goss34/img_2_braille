import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import UInt8MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np

DOT_ROWS = 36  # 9 rows × 4 dots
DOT_COLS = 80  # 40 cols × 2 dots

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

        # Step 1: Convert to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Step 2: Resize to Braille display resolution
        resized = cv2.resize(gray, (DOT_COLS, DOT_ROWS))

        # Step 3: Gaussian blur
        blurred = cv2.GaussianBlur(resized, (5, 5), 0)

        # Step 4: Canny edge detection
        edges = cv2.Canny(blurred, 50, 150)

        # Step 5: Binary format for Braille
        binary = (edges > 0).astype(np.uint8) * 255
        msg_out = UInt8MultiArray()
        msg_out.data = binary.flatten().tolist()
        self.publisher.publish(msg_out)

        # --- Composite Debug Image ---
        try:
            # Resize original gray to match debug layout
            gray_resized = cv2.resize(gray, (DOT_COLS, DOT_ROWS))

            # Stack images horizontally
            debug_stack = np.hstack((gray_resized, resized, blurred, binary))
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
