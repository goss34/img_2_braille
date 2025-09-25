import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image
from std_msgs.msg import UInt8MultiArray
from cv_bridge import CvBridge

import cv2
import numpy as np

DOT_ROWS, DOT_COLS = 36, 80


class BrailleMapPublisher(Node):
    def __init__(self):
        super().__init__('publish_braille_map')

        self.declare_parameter("use_dithering", False)
        self.declare_parameter("show_debug_image", True)

        self.use_dithering = self.get_parameter("use_dithering").get_parameter_value().bool_value
        self.show_debug_image = self.get_parameter("show_debug_image").get_parameter_value().bool_value

        self.add_on_set_parameters_callback(self.param_callback)

        self.subscription = self.create_subscription(
            Image, '/braille/heightmap', self.image_callback, 10)
        self.braille_pub = self.create_publisher(UInt8MultiArray, '/braille_map', 10)
        self.debug_pub = self.create_publisher(Image, '/braille_publish_map_node/debug_image', 10)

        self.bridge = CvBridge()
        self.get_logger().info("BrailleMapPublisher initialized.")

        # cv2.startWindowThread()

    def param_callback(self, params):
        for param in params:
            if param.name == 'use_dithering':
                self.use_dithering = param.value
                self.get_logger().info(f"Dithering: {self.use_dithering}")
            if param.name == 'show_debug_image':
                self.show_debug_image = param.value
                self.get_logger().info(f"Show debug: {self.show_debug_image}")
        return SetParametersResult(successful=True)

    def image_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        if img.shape != (DOT_ROWS, DOT_COLS):
            img = cv2.resize(img, (DOT_COLS, DOT_ROWS), interpolation=cv2.INTER_LINEAR)

        bin_img = floyd_steinberg_dither(img) if self.use_dithering else (img > 127).astype(np.uint8)

        msg_out = UInt8MultiArray()
        msg_out.data = bin_img.flatten().astype(np.uint8).tolist()
        self.braille_pub.publish(msg_out)

        if self.show_debug_image:
            try:
                debug = cv2.resize((bin_img * 255).astype(np.uint8),
                                   (DOT_COLS * 8, DOT_ROWS * 8),
                                   interpolation=cv2.INTER_NEAREST)
                # cv2.imshow("Braille Dithered", debug)
                # cv2.waitKey(1)

                debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding="mono8")
                debug_msg.header = msg.header
                self.debug_pub.publish(debug_msg)
            except Exception as e:
                self.get_logger().warn(f"Debug image display error: {e}")


def floyd_steinberg_dither(img: np.ndarray) -> np.ndarray:
    img = img.astype(np.float32) / 255.0
    h, w = img.shape
    out = np.copy(img)

    for y in range(h):
        for x in range(w):
            old = out[y, x]
            new = round(old)
            out[y, x] = new
            err = old - new
            if x + 1 < w: out[y, x + 1] += err * 7 / 16
            if x > 0 and y + 1 < h: out[y + 1, x - 1] += err * 3 / 16
            if y + 1 < h: out[y + 1, x] += err * 5 / 16
            if x + 1 < w and y + 1 < h: out[y + 1, x + 1] += err * 1 / 16

    return (out > 0.5).astype(np.uint8)


def main(args=None):
    rclpy.init(args=args)
    node = BrailleMapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
