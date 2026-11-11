"""Images for the simulator's side panels (numpy/OpenCV only, so they can be tested and saved
without a window). Throughout, dark means raised, as on the board."""
import math

import cv2
import numpy as np

RAISED, LOWERED, MIDDLE, UNKNOWN = 25, 235, 150, 90


def world_view(world, crop_size, px=320):
    """The room from above (+y up), the camera as a red arrow and the ego crop window in blue."""
    shade = np.full(world.grid.shape, UNKNOWN, np.uint8)
    shade[world.grid == 0] = LOWERED
    shade[world.grid == 100] = RAISED
    shade = np.flipud(shade)
    scale = px / max(world.width, world.height)
    img = cv2.resize(shade, (int(world.width * scale), int(world.height * scale)),
                     interpolation=cv2.INTER_NEAREST)
    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    res = world.meta['resolution']

    def to_px(x, y):
        return (int((x - world.origin_x) / res * scale),
                int((world.height - (y - world.origin_y) / res) * scale))

    fx, fy = math.cos(world.yaw), math.sin(world.yaw)   # forward
    rx, ry = math.sin(world.yaw), -math.cos(world.yaw)  # right
    half = crop_size / 2
    corners = [(world.x + f * fx + s * rx, world.y + f * fy + s * ry)
               for f, s in ((0, -half), (crop_size, -half), (crop_size, half), (0, half))]
    cv2.polylines(img, [np.array([to_px(*c) for c in corners], np.int32)], True, (200, 120, 30), 2,
                  cv2.LINE_AA)
    tip = to_px(world.x + 0.6 * fx, world.y + 0.6 * fy)
    cv2.arrowedLine(img, to_px(world.x, world.y), tip, (40, 40, 220), 3, cv2.LINE_AA, tipLength=0.4)
    return img


def map_crop_view(map_crop, scale=5):
    """/braille/heightmap: 255 raised, 0 lowered, 128 grey (which publish_map_node raises)."""
    if map_crop is None:
        return _placeholder('No map rendered yet: CENTER in occupancy view', scale)
    shade = np.full(map_crop.shape, MIDDLE, np.uint8)
    shade[map_crop > 127] = RAISED
    shade[map_crop == 128] = MIDDLE
    shade[map_crop < 127] = LOWERED
    return _scale(cv2.cvtColor(shade, cv2.COLOR_GRAY2BGR), scale)


def sent_view(processed, scale=5):
    """The image sent to the board, before packing. Rows the Canute never shows are tinted red,
    and raised dots on them are drawn red."""
    if processed is None:
        return _placeholder('Nothing sent yet', scale)
    h, w = processed.shape
    img = np.full((h, w, 3), LOWERED, np.uint8)
    img[processed != 0] = RAISED
    for r in range(3, h, 4):
        row = img[r]
        row[(processed[r] == 0)] = (205, 205, 245)
        row[(processed[r] != 0)] = (40, 40, 220)
    return _scale(img, scale)


def camera_view(debug_strip, scale=2):
    """image_to_braille_node's debug strip: camera, resized, blurred, edges."""
    return _scale(cv2.cvtColor(debug_strip, cv2.COLOR_GRAY2BGR), scale)


def _scale(img, scale):
    return cv2.resize(img, (img.shape[1] * scale, img.shape[0] * scale), interpolation=cv2.INTER_NEAREST)


def _placeholder(text, scale):
    img = np.full((36 * scale, 80 * scale, 3), 225, np.uint8)
    cv2.putText(img, text, (8, 18 * scale), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (90, 90, 90), 1, cv2.LINE_AA)
    return img
