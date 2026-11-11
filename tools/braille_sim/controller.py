"""What the ROS nodes do when buttons are pressed, run on the simulator's world and board.

The drawing steps are the real code (braille_interface.display, shared with the nodes).
The button flow below mirrors braille_server_node.handle_button_frame and
handle_center_press, including their known gaps: split view and the RIGHT button don't
render a fresh map. Keep it in step with the node until that logic is shared too.
"""
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src' / 'braille_interface'))
from braille_interface import display  # noqa: E402

HEIGHTMAP_SIZE = 5.0  # heightmap_generator_node's heightmap_size parameter (metres)


def test_camera_image():
    """A stand-in camera frame (640x480) with a few clear edges, until a real one is loaded."""
    img = np.full((480, 640, 3), 70, np.uint8)
    cv2.rectangle(img, (0, 300), (640, 480), (110, 110, 110), -1)    # floor
    cv2.rectangle(img, (80, 120), (200, 330), (180, 180, 180), -1)   # door
    cv2.rectangle(img, (330, 220), (520, 340), (40, 40, 40), -1)     # table
    cv2.circle(img, (560, 120), 40, (220, 220, 220), -1)             # lamp
    return img


class Controller:
    def __init__(self, world, board, size=HEIGHTMAP_SIZE):
        self.world, self.board, self.size = world, board, size
        self.mode = 'split'            # braille_server_node starts in split view
        self.invert_colors = False     # heightmap_generator_node parameter
        self.use_dithering = False     # publish_map_node parameter
        self.occupancy_map = None      # server's copy of /braille_map (0/1)
        self.image_map = None          # server's copy of /braille_image (0/255)
        self.map_crop = None           # last /braille/heightmap image (0/128/255)
        self.processed = None          # last image sent to the board, before packing
        self.message = 'Ready. Split view.'
        self.set_camera_image(test_camera_image())

    # -- inputs ----------------------------------------------------------------

    def set_camera_image(self, bgr):
        """image_to_braille_node: the camera feed is converted continuously (30 Hz on the Jetson)."""
        self.camera_bgr = bgr
        self.image_map, self.camera_debug = display.camera_to_edges(bgr)

    def render_crop(self):
        """heightmap_generator_node's render service, then publish_map_node."""
        raw = display.occupancy_to_raw(self.world.grid, self.invert_colors)
        self.map_crop = display.render_ego_crop(raw, self.world.meta, self.world.x, self.world.y,
                                                self.world.yaw, self.size)
        self.occupancy_map = display.heightmap_to_dots(self.map_crop, self.use_dithering)

    # -- buttons ---------------------------------------------------------------

    def press(self, button):
        if button == 'CENTER':
            if self.mode == 'occupancy':
                self.render_crop()
            self.update()
        elif button == 'RIGHT':
            self.mode = display.next_mode(self.mode)
            self.message = f'Mode: {self.mode}.'
            self.update()
        elif button == 'ROW_0':
            self.reset()
        elif button == 'LEFT':
            self.message = 'LEFT: warm-up command sent (no effect in the simulator).'
        else:
            self.message = f'{button}: no action in the code.'

    def update(self):
        """braille_server_node.handle_trigger."""
        view = display.compose_view(self.mode, self.occupancy_map, self.image_map)
        if view.image is None:
            self.message = f'Update failed: {view.error}'
            return
        lines = display.occupancy_to_braille(view.image)
        self.board.send_page([list(display.line_payload(line)) for line in lines])
        self.processed = view.image
        note = f' ({view.warning})' if view.warning else ''
        self.message = f'Updating the board in {self.mode} view{note}.'

    def reset(self):
        """braille_server_node.reset_display: blank every line and forget the maps."""
        blank = display.line_payload(chr(0x2800) * display.CELLS)
        self.board.send_page([list(blank)] * display.ROWS)
        self.occupancy_map = self.image_map = self.processed = None
        # On the Jetson the camera topic refills the image map within a frame.
        self.image_map, self.camera_debug = display.camera_to_edges(self.camera_bgr)
        self.message = 'ROW_0: display reset.'
