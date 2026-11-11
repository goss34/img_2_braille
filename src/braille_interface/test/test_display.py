"""Tests for braille_interface.display. No ROS needed:

    python3 -m unittest discover -s src/braille_interface/test -p 'test_display.py'

Tests marked "bug #N" pin today's known-wrong behaviour so the fix shows up as a deliberate
test change. Update them when the bug is fixed.
"""
import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from braille_interface import display  # noqa: E402
from braille_interface.display import CELLS, DOT_COLS, DOT_ROWS, ROWS  # noqa: E402


def payloads(grid):
    return [display.line_payload(line) for line in display.occupancy_to_braille(grid)]


def room(obstacles, res=0.05, extent=8.0):
    """Free square room centred on the origin, with occupied (x0, y0, x1, y1) rectangles."""
    n = int(extent / res)
    grid = np.zeros((n, n), np.int8)
    origin = -extent / 2
    for x0, y0, x1, y1 in obstacles:
        c0, c1 = int((x0 - origin) / res), int((x1 - origin) / res)
        r0, r1 = int((y0 - origin) / res), int((y1 - origin) / res)
        grid[r0:max(r1, r0 + 1), c0:max(c1, c0 + 1)] = 100
    meta = {'resolution': res, 'origin_x': origin, 'origin_y': origin, 'width': n, 'height': n}
    return display.occupancy_to_raw(grid), meta


def raised_columns(crop, rows=slice(1, DOT_ROWS - 1), cols=slice(23, 57)):
    """Mean column of raised map pixels inside the map square (border excluded)."""
    ys, xs = np.nonzero(crop[rows, cols] == 255)
    return xs.mean() + cols.start if len(xs) else None


class Packing(unittest.TestCase):
    def test_cells_use_unicode_dot_order(self):
        for (dy, dx), bit in {(0, 0): 0, (1, 0): 1, (2, 0): 2, (0, 1): 3, (1, 1): 4, (2, 1): 5}.items():
            grid = np.zeros((DOT_ROWS, DOT_COLS), np.uint8)
            grid[dy, dx] = 1
            self.assertEqual(payloads(grid)[0][0], 1 << bit)

    def test_every_payload_is_one_byte_per_cell(self):
        grid = np.ones((DOT_ROWS, DOT_COLS), np.uint8)
        self.assertEqual([len(p) for p in payloads(grid)], [CELLS] * ROWS)
        self.assertTrue(all(b == 0x3F for p in payloads(grid) for b in p))

    def test_bug1_fourth_dot_row_never_reaches_the_board(self):
        grid = np.zeros((DOT_ROWS, DOT_COLS), np.uint8)
        grid[3::4, :] = 1  # dot rows 3, 7, 11, ... 35
        self.assertTrue(all(b == 0 for p in payloads(grid) for b in p))


class Views(unittest.TestCase):
    def setUp(self):
        self.occ = np.ones((DOT_ROWS, DOT_COLS), np.uint8)
        self.img = np.full((DOT_ROWS, DOT_COLS), 255, np.uint8)

    def test_missing_maps(self):
        self.assertEqual(display.compose_view('occupancy', None, self.img).error, 'No occupancy map available.')
        self.assertEqual(display.compose_view('image', self.occ, None).error, 'No image map available.')
        view = display.compose_view('split', None, None)
        self.assertEqual((view.error, view.warning),
                         ('No maps available for fallback.', 'Split view requested, but one map is missing.'))

    def test_split_falls_back_to_the_map_it_has(self):
        view = display.compose_view('split', None, self.img)
        self.assertIsNotNone(view.warning)
        self.assertTrue(np.array_equal(view.image, self.img))

    def test_split_has_a_two_column_divider(self):
        view = display.compose_view('split', self.occ * 0, self.img * 0)
        self.assertEqual(view.image.shape, (DOT_ROWS, DOT_COLS))
        self.assertTrue((view.image[:, 39:41] == 255).all())


class Buttons(unittest.TestCase):
    def test_decode(self):
        self.assertEqual(display.decode_buttons(0x01, 0x10), {'ROW_0', 'CENTER'})
        self.assertEqual(display.decode_buttons(0x00, 0x07), set())  # unmapped y bits 0-2

    def test_modes_cycle(self):
        self.assertEqual([display.next_mode(m) for m in display.MODES], ['occupancy', 'image', 'split'])


class EgoCrop(unittest.TestCase):
    def test_left_right_and_ahead(self):
        raw, meta = room([(1.4, 0.6, 1.8, 1.0)])  # box ahead and to the left (+y)
        crop = display.render_ego_crop(raw, meta, 0.0, 0.0, 0.0, 5.0)
        self.assertLess(raised_columns(crop), 40)       # left of centre
        ys, _ = np.nonzero(crop[1:35, 23:57] == 255)
        self.assertLess(ys.mean() + 1, 30)              # ahead, so above the camera (bottom row)

    def test_turning_left_moves_things_to_the_right(self):
        raw, meta = room([(1.0, 1.3, 1.4, 1.7)])  # ahead-left when facing +x
        facing_x = display.render_ego_crop(raw, meta, 0.0, 0.0, 0.0, 5.0)
        facing_y = display.render_ego_crop(raw, meta, 0.0, 0.0, math.radians(90), 5.0)  # turned left
        self.assertLess(raised_columns(facing_x), 40)
        self.assertGreater(raised_columns(facing_y), 40)  # now ahead-right

    def test_bug2_padding_and_unknown_become_raised_dots(self):
        crop = np.full((DOT_ROWS, DOT_COLS), 128, np.uint8)
        self.assertTrue((display.heightmap_to_dots(crop) == 1).all())

    def test_bug3_a_thin_wall_can_vanish(self):
        raw, meta = room([(3.0, -3.0, 3.05, 3.0)])  # one-cell wall 3 m ahead
        seen = [(display.render_ego_crop(raw, meta, dx, 0.0, 0.0, 5.0)[1:35, 30:50] == 255).all(axis=1).any()
                for dx in np.arange(0, 0.15, 0.01)]
        self.assertIn(False, seen)


if __name__ == '__main__':
    unittest.main()
