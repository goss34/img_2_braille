import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import numpy as np
import cv2
import time
import serial
import asyncio
import threading

from braille_interface.util.simple_hdlc import HDLC
from braille_interface.util.comms_codes import CMD_SEND_LINE, CMD_SEND_BUTTONS, CMD_WARMUP

SERIAL_PORT = '/dev/ttyACM0'
BAUDRATE = 57600
ROWS, CHARS = 9, 40
DOT_ROWS, DOT_COLS = ROWS * 4, CHARS * 2

BUTTON_MAP_X = {
    0: "ROW_0", 1: "ROW_1", 2: "ROW_2", 3: "ROW_3",
    4: "ROW_4", 5: "ROW_5", 6: "ROW_6", 7: "ROW_7"
}
BUTTON_MAP_Y = {
    3: "LEFT", 4: "CENTER", 5: "RIGHT"
}

class BraillePipelineNode(Node):
    def __init__(self):
        super().__init__('braille_pipeline_node')

        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.2)
        except serial.SerialException as e:
            self.get_logger().fatal(f"Serial connection failed: {e}")
            raise

        self.hdlc = HDLC(self.ser)
        self.occupancy_map = None
        self.image_map = None
        self.prev_buttons = set()
        self.display_mode = 'split'

        self.subscription = self.create_subscription(
            UInt8MultiArray, '/braille_map', self.map_callback, 10)

        self.image_sub = self.create_subscription(
            UInt8MultiArray, '/braille_image', self.image_callback, 10)

        self.srv = self.create_service(
            Trigger, '/braille_display/update', self.handle_trigger)

        self.trigger_client = self.create_client(Trigger, '/braille_display/update')
        while not self.trigger_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('/braille_display/update service not available...')

        self.debug_pub = self.create_publisher(Image, '/braille_pipeline/debug_viz', 10)
        self.bridge = CvBridge()

        self.latest_processed = None
        self.latest_braille_lines = None
        self.debug_timer = self.create_timer(1.0, self.publish_debug_image)

        self.get_logger().info("BraillePipelineNode ready.")

        self.button_thread = threading.Thread(
            target=self.start_button_polling_loop, daemon=True)
        self.button_thread.start()

    def map_callback(self, msg):
        if len(msg.data) != DOT_ROWS * DOT_COLS:
            self.get_logger().warn(f"Expected {DOT_ROWS * DOT_COLS}, got {len(msg.data)}")
            return
        self.occupancy_map = np.array(msg.data, dtype=np.uint8).reshape((DOT_ROWS, DOT_COLS))

    def image_callback(self, msg):
        if len(msg.data) != DOT_ROWS * DOT_COLS:
            self.get_logger().warn(f"Expected {DOT_ROWS * DOT_COLS}, got {len(msg.data)}")
            return
        self.image_map = np.array(msg.data, dtype=np.uint8).reshape((DOT_ROWS, DOT_COLS))

    def handle_trigger(self, request, response):
        map1 = self.occupancy_map
        map2 = self.image_map

        if self.display_mode == 'split':
            if map1 is None or map2 is None:
                self.get_logger().warn("Split view requested, but one map is missing.")
                fallback_map = map1 if map1 is not None else map2
                if fallback_map is None:
                    response.success = False
                    response.message = "No maps available for fallback."
                    return response
                processed = self.preprocess_and_pad(fallback_map, (DOT_ROWS, DOT_COLS))
            else:
                left = self.preprocess_and_pad(map1, (DOT_ROWS, DOT_COLS // 2 - 1))
                right = self.preprocess_and_pad(map2, (DOT_ROWS, DOT_COLS // 2 - 1))
                divider = 255 * np.ones((DOT_ROWS, 2), dtype=np.uint8)
                processed = np.hstack((left, divider, right))
        elif self.display_mode == 'occupancy':
            if map1 is None:
                response.success = False
                response.message = "No occupancy map available."
                return response
            processed = self.preprocess_and_pad(map1, (DOT_ROWS, DOT_COLS))
        elif self.display_mode == 'image':
            if map2 is None:
                response.success = False
                response.message = "No image map available."
                return response
            processed = self.preprocess_and_pad(map2, (DOT_ROWS, DOT_COLS))

        braille_lines = self.occupancy_to_braille(processed)
        for row, line in enumerate(braille_lines):
            self.send_line(row, line)
            time.sleep(0.05)

        self.latest_processed = processed
        self.latest_braille_lines = braille_lines
        self.publish_debug_image()

        response.success = True
        response.message = f"Braille display updated in mode: {self.display_mode}"
        return response

    def occupancy_to_braille(self, grid):
        lines = []
        for r in range(0, DOT_ROWS, 4):
            line = ''
            for c in range(0, DOT_COLS, 2):
                cell = 0
                for dy, dx in [(i, j) for i in range(4) for j in range(2)]:
                    if grid[r+dy, c+dx]:
                        bit = {(0, 0): 0, (1, 0): 1, (2, 0): 2, (3, 0): 6,
                               (0, 1): 3, (1, 1): 4, (2, 1): 5, (3, 1): 7}[(dy, dx)]
                        cell |= (1 << bit)
                line += chr(0x2800 + cell)
            lines.append(line)
        return lines

    def send_line(self, row, text):
        payload = bytes([ord(c) & 0x3F for c in text.ljust(CHARS)])
        packet = bytes([CMD_SEND_LINE, row]) + payload
        self.hdlc.sendFrame(packet)
        self.get_logger().info(f"Row {row} sent: {text}")

    def preprocess_and_pad(self, input_img, target_shape=(DOT_ROWS, DOT_COLS)):
        h, w = input_img.shape
        target_h, target_w = target_shape

        if h == 0 or w == 0:
            self.get_logger().warn("Empty input image received.")
            return np.zeros(target_shape, dtype=np.uint8)

        input_aspect = w / h
        target_aspect = target_w / target_h

        if input_aspect > target_aspect:
            new_w = target_w
            new_h = int(target_w / input_aspect)
        else:
            new_h = target_h
            new_w = int(target_h * input_aspect)

        resized = cv2.resize(input_img, (new_w, new_h), interpolation=cv2.INTER_NEAREST)
        padded = np.zeros(target_shape, dtype=np.uint8)
        y_offset = (target_h - new_h) // 2
        x_offset = (target_w - new_w) // 2
        padded[y_offset:y_offset+new_h, x_offset:x_offset+new_w] = resized

        return padded

    def render_split_debug_image(self, occupancy_map):
        scale = 5
        vis = cv2.resize(
            occupancy_map,
            (DOT_COLS * scale, DOT_ROWS * scale),
            interpolation=cv2.INTER_NEAREST
        )
        vis = cv2.cvtColor(vis, cv2.COLOR_GRAY2BGR)

        border = 20
        vis_with_border = cv2.copyMakeBorder(
            vis, top=border, bottom=border, left=border, right=border,
            borderType=cv2.BORDER_CONSTANT, value=(255, 255, 255)
        )

        line_x = (DOT_COLS // 2) * scale
        cv2.line(vis_with_border, (line_x, 0), (line_x, vis_with_border.shape[0]), (0, 0, 255), 1)

        cv2.putText(vis_with_border,
                    f"Mode: {self.display_mode} (Left: Occ, Right: Img)",
                    (10, 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)

        return vis_with_border

    def publish_debug_image(self):
        if self.latest_processed is not None:
            debug_image = self.render_split_debug_image(self.latest_processed)
            msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
            self.debug_pub.publish(msg)

    def start_button_polling_loop(self):
        asyncio.run(self.poll_buttons())

    async def poll_buttons(self):
        while rclpy.ok():
            self.hdlc.sendFrame(bytes([CMD_SEND_BUTTONS]))
            await asyncio.sleep(0.1)
            try:
                frame = self.hdlc.readFrame(timeout=0.5)
                if not isinstance(frame, (bytes, bytearray)):
                    raise TypeError(f"Expected bytearray, got {type(frame)}")
                self.handle_button_frame(frame)
            except RuntimeError:
                self.get_logger().warn("[Timeout] No button response.")
            except ValueError:
                self.get_logger().warn("[CRC FAIL] Invalid frame.")
            except Exception as e:
                self.get_logger().error(f"[Unexpected Error] {e}")

    def handle_button_frame(self, frame):
        if len(frame) < 3:
            self.get_logger().warn(f"[Invalid Frame] Too short: {list(frame)}")
            return

        x_byte = frame[1]
        y_byte = frame[2]
        active = set()

        for bit, name in BUTTON_MAP_X.items():
            if x_byte & (1 << bit):
                active.add(name)

        for bit, name in BUTTON_MAP_Y.items():
            if y_byte & (1 << bit):
                active.add(name)

        new_presses = active - self.prev_buttons
        if new_presses:
            self.get_logger().info(f"[Button Pressed] {', '.join(sorted(new_presses))}")

            if "CENTER" in new_presses:
                self.get_logger().info("CENTER button → triggering update")
                self.call_braille_update_service()

            if "ROW_0" in new_presses:
                self.get_logger().info("ROW_0 button → resetting display")
                self.reset_display()

            if "LEFT" in new_presses:
                self.get_logger().info("LEFT button → warming up display")
                self.warmup_display()

            if "RIGHT" in new_presses:
                modes = ['split', 'occupancy', 'image']
                idx = (modes.index(self.display_mode) + 1) % len(modes)
                self.display_mode = modes[idx]
                self.get_logger().info(f"[Mode Change] → {self.display_mode}")
                self.call_braille_update_service()

        self.prev_buttons = active

    def call_braille_update_service(self):
        request = Trigger.Request()
        future = self.trigger_client.call_async(request)

        def callback(fut):
            try:
                result = fut.result()
                if result.success:
                    self.get_logger().info(f"Braille update: {result.message}")
                else:
                    self.get_logger().warn(f"Braille update failed: {result.message}")
            except Exception as e:
                self.get_logger().error(f"Service call failed: {e}")

        future.add_done_callback(callback)

    def reset_display(self):
        blank_line = chr(0x2800) * CHARS
        for row in range(ROWS):
            self.send_line(row, blank_line)
            time.sleep(0.05)

        self.occupancy_map = None
        self.image_map = None
        self.latest_processed = None
        self.latest_braille_lines = None
        self.get_logger().info("Braille display and buffers reset.")

    def warmup_display(self):
        self.get_logger().info("Sending CMD_WARMUP to Braille display...")
        try:
            self.hdlc.sendFrame(bytes([CMD_WARMUP]))
            self.get_logger().info("Braille display warm-up command sent.")
        except Exception as e:
            self.get_logger().error(f"Failed to send CMD_WARMUP: {e}")

    def destroy_node(self):
        super().destroy_node()
        if self.ser and self.ser.is_open:
            self.ser.close()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = BraillePipelineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()