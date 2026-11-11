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
from braille_interface.display import (
    ROWS, CELLS as CHARS, DOT_ROWS, DOT_COLS,
    compose_view, decode_buttons, line_payload, next_mode, occupancy_to_braille)

SERIAL_PORT = '/dev/ttyACM0'
BAUDRATE = 57600


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

        # Client for the heightmap node's ego-crop render service.
        # Used when CENTER is pressed in occupancy mode so the heightmap node
        # snapshots the current camera pose and publishes a fresh crop to
        # /braille/heightmap before the normal display update runs.
        self.render_ego_crop_client = self.create_client(
            Trigger, '/braille_heightmap/render_ego_crop')
        # Non-blocking availability check — we warn but don't block startup,
        # since the heightmap node may not always be running.
        if not self.render_ego_crop_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                '/braille_heightmap/render_ego_crop service not available. '
                'Occupancy mode CENTER press will fall back to direct display update.')

        self.debug_pub = self.create_publisher(Image, '/braille_pipeline/debug_viz', 10)
        self.bridge = CvBridge()

        self.latest_processed = None
        self.latest_braille_lines = None
        self.debug_timer = self.create_timer(1.0, self.publish_debug_image)

        self.get_logger().info("BraillePipelineNode ready.")

        self.button_thread = threading.Thread(
            target=self.start_button_polling_loop, daemon=True)
        self.button_thread.start()

    # ------------------------------------------------------------------
    # Map callbacks
    # ------------------------------------------------------------------

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

    # ------------------------------------------------------------------
    # Display update service handler
    # ------------------------------------------------------------------

    def handle_trigger(self, request, response):
        view = compose_view(self.display_mode, self.occupancy_map, self.image_map)
        if view.warning:
            self.get_logger().warn(view.warning)
        if view.image is None:
            response.success = False
            response.message = view.error
            return response
        processed = view.image

        braille_lines = occupancy_to_braille(processed)
        for row, line in enumerate(braille_lines):
            self.send_line(row, line)
            time.sleep(0.05)

        self.latest_processed    = processed
        self.latest_braille_lines = braille_lines
        self.publish_debug_image()

        response.success = True
        response.message = f"Braille display updated in mode: {self.display_mode}"
        return response

    # ------------------------------------------------------------------
    # Button handling
    # ------------------------------------------------------------------

    def handle_button_frame(self, frame):
        if len(frame) < 3:
            self.get_logger().warn(f"[Invalid Frame] Too short: {list(frame)}")
            return

        active = decode_buttons(frame[1], frame[2])

        new_presses = active - self.prev_buttons
        if new_presses:
            self.get_logger().info(f"[Button Pressed] {', '.join(sorted(new_presses))}")

            if "CENTER" in new_presses:
                self.get_logger().info("CENTER button → triggering update")
                self.handle_center_press()

            if "ROW_0" in new_presses:
                self.get_logger().info("ROW_0 button → resetting display")
                self.reset_display()

            if "LEFT" in new_presses:
                self.get_logger().info("LEFT button → warming up display")
                self.warmup_display()

            if "RIGHT" in new_presses:
                self.display_mode = next_mode(self.display_mode)
                self.get_logger().info(f"[Mode Change] → {self.display_mode}")
                self.call_braille_update_service()

        self.prev_buttons = active

    def handle_center_press(self):
        """
        In occupancy mode: first ask the heightmap node to snapshot the
        current camera pose and push a fresh ego-centric crop to
        /braille/heightmap (which publish_map_node will forward to
        /braille_map).  Once that service returns, trigger the normal display
        update so the board renders whatever is now in /braille_map.

        In all other modes: go straight to the normal display update.
        """
        if self.display_mode == 'occupancy' and \
                self.render_ego_crop_client.service_is_ready():
            self.get_logger().info(
                "Occupancy mode — requesting ego-crop render before display update.")
            request = Trigger.Request()
            future  = self.render_ego_crop_client.call_async(request)
            future.add_done_callback(self._on_ego_crop_done)
        else:
            # Non-occupancy modes, or heightmap service unavailable — update directly.
            self.call_braille_update_service()

    def _on_ego_crop_done(self, future):
        """
        Called when /braille_heightmap/render_ego_crop returns.
        Regardless of success/failure we always proceed with the display
        update — worst case the board shows whatever map data is already
        buffered.
        """
        try:
            result = future.result()
            if result.success:
                self.get_logger().info(f"Ego-crop ready: {result.message}")
            else:
                self.get_logger().warn(
                    f"Ego-crop service returned failure: {result.message} "
                    f"— proceeding with display update anyway.")
        except Exception as e:
            self.get_logger().error(
                f"Ego-crop service call raised exception: {e} "
                f"— proceeding with display update anyway.")

        # The heightmap node has now published to /braille/heightmap →
        # publish_map_node will have forwarded it to /braille_map.
        # Small sleep to allow the message to propagate before we read it.
        time.sleep(0.1)
        self.call_braille_update_service()

    # ------------------------------------------------------------------
    # Service call helpers
    # ------------------------------------------------------------------

    def call_braille_update_service(self):
        request = Trigger.Request()
        future  = self.trigger_client.call_async(request)

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

    # ------------------------------------------------------------------
    # Braille encoding / serial send
    # ------------------------------------------------------------------

    def send_line(self, row, text):
        packet  = bytes([CMD_SEND_LINE, row]) + line_payload(text)
        self.hdlc.sendFrame(packet)
        self.get_logger().info(f"Row {row} sent: {text}")

    # ------------------------------------------------------------------
    # Debug image
    # ------------------------------------------------------------------

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
        cv2.line(vis_with_border,
                 (line_x, 0), (line_x, vis_with_border.shape[0]), (0, 0, 255), 1)

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

    # ------------------------------------------------------------------
    # Button polling loop
    # ------------------------------------------------------------------

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

    # ------------------------------------------------------------------
    # Utility commands
    # ------------------------------------------------------------------

    def reset_display(self):
        blank_line = chr(0x2800) * CHARS
        for row in range(ROWS):
            self.send_line(row, blank_line)
            time.sleep(0.05)

        self.occupancy_map        = None
        self.image_map            = None
        self.latest_processed     = None
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