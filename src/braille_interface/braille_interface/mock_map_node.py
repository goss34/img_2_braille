import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
import numpy as np
import time

DOT_ROWS = 36  # 9 rows * 4
DOT_COLS = 80  # 40 chars * 2

class MockBraillePublisher(Node):
    def __init__(self):
        super().__init__('mock_braille_publisher')

        self.map_pub = self.create_publisher(UInt8MultiArray, '/braille_map', 10)
        self.img_pub = self.create_publisher(UInt8MultiArray, '/braille_image', 10)

        self.timer = self.create_timer(2.0, self.timer_callback)
        self.toggle = True
        self.get_logger().info("MockBraillePublisher ready.")

    def timer_callback(self):
        occ_map = np.zeros((DOT_ROWS, DOT_COLS), dtype=np.uint8)
        img_map = np.zeros((DOT_ROWS, DOT_COLS), dtype=np.uint8)

        if self.toggle:
            occ_map[:, ::4] = 255  # vertical stripes
            img_map[::4, :] = 255  # horizontal stripes
        else:
            cv2 = __import__('cv2')
            occ_map = cv2.circle(occ_map, (40, 18), 10, 255, -1)
            img_map = cv2.rectangle(img_map, (20, 10), (60, 26), 255, -1)

        self.toggle = not self.toggle

        occ_msg = UInt8MultiArray()
        occ_msg.data = occ_map.flatten().tolist()
        self.map_pub.publish(occ_msg)

        img_msg = UInt8MultiArray()
        img_msg.data = img_map.flatten().tolist()
        self.img_pub.publish(img_msg)

        self.get_logger().info("Published mock occupancy and image maps.")

def main(args=None):
    rclpy.init(args=args)
    node = MockBraillePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
