"""Display logic shared by the braille ROS nodes and the laptop simulator.

Plain Python (numpy and OpenCV, no ROS), so it can be tested and simulated on any
computer. The nodes call these functions; they moved here unchanged from the nodes,
known bugs included, so that fixes can be tested here before they're deployed.
"""
import math
from collections import namedtuple

import cv2
import numpy as np

ROWS, CELLS = 9, 40  # lines on the Canute 360, braille cells per line
# The pipeline draws 4 dot rows per cell, but the Canute's cells have 3:
# line_payload() drops every 4th row (bug #1).
DOT_ROWS, DOT_COLS = ROWS * 4, CELLS * 2

BUTTON_MAP_X = {
    0: "ROW_0", 1: "ROW_1", 2: "ROW_2", 3: "ROW_3",
    4: "ROW_4", 5: "ROW_5", 6: "ROW_6", 7: "ROW_7"
}
BUTTON_MAP_Y = {
    3: "LEFT", 4: "CENTER", 5: "RIGHT"
}
MODES = ['split', 'occupancy', 'image']

View = namedtuple('View', 'image error warning')


# ---------------------------------------------------------------- occupancy map

def yaw_from_quaternion(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def occupancy_to_raw(occ_data, invert_colors=False):
    """nvblox occupancy values to pixels.

    Default: raised (255) = occupied (100) or unknown (-1), lowered (0) = free (0),
    128 = any other value. invert_colors swaps raised and lowered.
    """
    height, width = occ_data.shape
    raw_map = np.full((height, width), 128, dtype=np.uint8)
    if invert_colors:
        raw_map[occ_data == 0] = 255    # Free     → raised
        raw_map[occ_data == 100] = 0    # Occupied → lowered
        raw_map[occ_data == -1] = 0     # Unknown  → lowered
    else:
        raw_map[occ_data == 0] = 0      # Free     → lowered
        raw_map[occ_data == 100] = 255  # Occupied → raised
        raw_map[occ_data == -1] = 255   # Unknown  → raised
    return raw_map


def render_ego_crop(raw, meta, cam_x, cam_y, yaw, size):
    """Forward-facing, camera-centred window of the stored occupancy map.

    The camera sits at the bottom centre, its forward direction points up, and the
    window covers 0 → size metres ahead and ±size/2 metres to the sides. Each output
    pixel samples the map cell under it. Returns a (DOT_ROWS, DOT_COLS) image.
    """
    res = meta['resolution']
    ox, oy = meta['origin_x'], meta['origin_y']

    half_w = size / 2.0  # lateral half-width (±left/right of camera)
    depth = size         # forward depth       (0 → depth ahead)

    patch_w = max(int(math.ceil(size / res)), 1)   # lateral axis
    patch_h = max(int(math.ceil(depth / res)), 1)  # forward axis
    patch = np.full((patch_h, patch_w), 128, dtype=np.uint8)

    # camera forward in world = (cos_yaw, sin_yaw); camera right = (sin_yaw, -cos_yaw)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)

    for pr in range(patch_h):
        for pc in range(patch_w):
            # pr=0 → full depth ahead; pc=0 → hard left
            fwd_offset = depth - (pr + 0.5) * res
            lat_offset = -half_w + (pc + 0.5) * res

            wx = cam_x + fwd_offset * cos_yaw + lat_offset * sin_yaw
            wy = cam_y + fwd_offset * sin_yaw - lat_offset * cos_yaw

            mx = int((wx - ox) / res)
            my = int((wy - oy) / res)

            if 0 <= mx < meta['width'] and 0 <= my < meta['height']:
                patch[pr, pc] = raw[my, mx]
            # else: leave as 128 (outside known map area)

    return fit_and_pad_map(patch, (DOT_ROWS, DOT_COLS))


def fit_and_pad_map(input_img, target_shape):
    """Scale to fit target_shape (nearest neighbour), pad with 128 and draw a 255 border."""
    h, w = input_img.shape
    target_h, target_w = target_shape
    input_aspect = w / h
    target_aspect = target_w / target_h

    if input_aspect > target_aspect:
        new_w = target_w
        new_h = int(target_w / input_aspect)
    else:
        new_h = target_h
        new_w = int(target_h * input_aspect)

    resized = cv2.resize(input_img, (new_w, new_h), interpolation=cv2.INTER_NEAREST)
    padded = np.full(target_shape, 128, dtype=np.uint8)
    y_offset = (target_h - new_h) // 2
    x_offset = (target_w - new_w) // 2
    padded[y_offset:y_offset + new_h, x_offset:x_offset + new_w] = resized

    cv2.rectangle(
        padded,
        (x_offset, y_offset),
        (x_offset + new_w - 1, y_offset + new_h - 1),
        255, 1
    )
    return padded


def heightmap_to_dots(img, use_dithering=False):
    """The map image to a 0/1 dot grid, as publish_map_node sends it to /braille_map."""
    if img.shape != (DOT_ROWS, DOT_COLS):
        img = cv2.resize(img, (DOT_COLS, DOT_ROWS), interpolation=cv2.INTER_LINEAR)
    return floyd_steinberg_dither(img) if use_dithering else (img > 127).astype(np.uint8)


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


# ---------------------------------------------------------------- camera image

def camera_to_edges(bgr):
    """Camera image to a 0/255 edge grid, plus the debug strip image_to_braille_node publishes."""
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    resized = cv2.resize(gray, (DOT_COLS, DOT_ROWS))
    blurred = cv2.GaussianBlur(resized, (5, 5), 0)
    edges = cv2.Canny(blurred, 50, 150)
    binary = (edges > 0).astype(np.uint8) * 255

    gray_resized = cv2.resize(gray, (DOT_COLS, DOT_ROWS))
    debug_stack = np.hstack((gray_resized, resized, blurred, binary))
    return binary, debug_stack


# ---------------------------------------------------------------- what goes on the board

def compose_view(mode, occupancy_map, image_map):
    """The DOT_ROWS × DOT_COLS image to show for a display mode.

    Returns View(image, error, warning): image is None when there's nothing to show,
    with the reason in error; warning is set when split view falls back to one map.
    """
    if mode == 'split':
        if occupancy_map is None or image_map is None:
            warning = "Split view requested, but one map is missing."
            fallback_map = occupancy_map if occupancy_map is not None else image_map
            if fallback_map is None:
                return View(None, "No maps available for fallback.", warning)
            return View(fit_and_pad(fallback_map, (DOT_ROWS, DOT_COLS)), None, warning)
        left = fit_and_pad(occupancy_map, (DOT_ROWS, DOT_COLS // 2 - 1))
        right = fit_and_pad(image_map, (DOT_ROWS, DOT_COLS // 2 - 1))
        divider = 255 * np.ones((DOT_ROWS, 2), dtype=np.uint8)
        return View(np.hstack((left, divider, right)), None, None)

    if mode == 'occupancy':
        if occupancy_map is None:
            return View(None, "No occupancy map available.", None)
        return View(fit_and_pad(occupancy_map, (DOT_ROWS, DOT_COLS)), None, None)

    if mode == 'image':
        if image_map is None:
            return View(None, "No image map available.", None)
        return View(fit_and_pad(image_map, (DOT_ROWS, DOT_COLS)), None, None)

    raise ValueError(f"unknown display mode: {mode}")


def fit_and_pad(input_img, target_shape=(DOT_ROWS, DOT_COLS)):
    """Scale to fit target_shape (nearest neighbour) and pad with zeros."""
    h, w = input_img.shape
    target_h, target_w = target_shape

    if h == 0 or w == 0:
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
    padded[y_offset:y_offset + new_h, x_offset:x_offset + new_w] = resized

    return padded


def occupancy_to_braille(grid):
    """The dot grid to 9 lines of Unicode braille (8-dot cells; see line_payload)."""
    lines = []
    for r in range(0, DOT_ROWS, 4):
        line = ''
        for c in range(0, DOT_COLS, 2):
            cell = 0
            for dy, dx in [(i, j) for i in range(4) for j in range(2)]:
                if grid[r + dy, c + dx]:
                    bit = {(0, 0): 0, (1, 0): 1, (2, 0): 2, (3, 0): 6,
                           (0, 1): 3, (1, 1): 4, (2, 1): 5, (3, 1): 7}[(dy, dx)]
                    cell |= (1 << bit)
            line += chr(0x2800 + cell)
        lines.append(line)
    return lines


def line_payload(text):
    """One line of braille as the 40 bytes the Canute receives: 6 bits per cell, so dots
    7 and 8 (the 4th dot row of each cell) are dropped."""
    return bytes([ord(c) & 0x3F for c in text.ljust(CELLS)])


# ---------------------------------------------------------------- buttons

def decode_buttons(x_byte, y_byte):
    """Names of the pressed buttons in a SEND_BUTTONS reply."""
    active = set()
    for bit, name in BUTTON_MAP_X.items():
        if x_byte & (1 << bit):
            active.add(name)
    for bit, name in BUTTON_MAP_Y.items():
        if y_byte & (1 << bit):
            active.add(name)
    return active


def next_mode(mode):
    return MODES[(MODES.index(mode) + 1) % len(MODES)]
