"""Simulator tests (no window needed):

    python3 -m unittest discover -s tools/braille_sim -p 'test_*.py'
"""
import math
import sys
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import panels  # noqa: E402
from board import CELLS, ROWS, Board, render_png  # noqa: E402
from controller import Controller  # noqa: E402
from world import World  # noqa: E402

FULL, BLANK = [0x3F] * CELLS, [0] * CELLS


def finish(board, real_speed=True):
    """Run the board's refresh to completion; return how long it took (simulated seconds)."""
    t = 0.0
    while board.busy:
        board.tick(t, real_speed)
        t += 0.01
    return t


class BoardModel(unittest.TestCase):
    def test_changed_lines_take_line_seconds_unchanged_are_skipped(self):
        board = Board(line_seconds=1.0, skip_seconds=0.1)
        board.send_page([FULL] * 3 + [BLANK] * 6)        # 3 changed lines, 6 unchanged
        self.assertAlmostEqual(board.seconds_left(0.0), 3 * 1.0 + 6 * 0.1, places=6)
        self.assertAlmostEqual(finish(board), 3.6, delta=0.05)
        self.assertEqual(board.cells[:3], [FULL] * 3)

    def test_same_page_again_is_fast(self):
        board = Board(line_seconds=1.0, skip_seconds=0.1)
        board.send_page([FULL] * ROWS)
        finish(board)
        board.send_page([FULL] * ROWS)
        self.assertAlmostEqual(finish(board), 0.9, delta=0.05)

    def test_instant_mode(self):
        board = Board()
        board.send_page([FULL] * ROWS)
        board.tick(0.0, real_speed=False)
        self.assertFalse(board.busy)
        self.assertEqual(board.cells, [FULL] * ROWS)

    def test_only_six_bits_per_cell(self):
        board = Board()
        board.send_line(0, [0xFF] * CELLS)
        board.tick(0.0, real_speed=False)
        self.assertEqual(board.cells[0], FULL)


class Worlds(unittest.TestCase):
    def test_boxes_are_where_the_scene_says(self):
        w = World('Two-box test')
        m = w.meta
        for x, y, expect in ((1.5, 0.5, 100), (2.0, -0.75, 100), (1.0, 0.0, 0), (20.0, 0.0, None)):
            c, r = int((x - m['origin_x']) / m['resolution']), int((y - m['origin_y']) / m['resolution'])
            inside = 0 <= c < m['width'] and 0 <= r < m['height']
            self.assertEqual(w.grid[r, c] if inside else None, expect)

    def test_move_and_turn(self):
        w = World('Empty room')
        w.turn(90)   # left
        w.move(1.0)  # forward is now +y
        self.assertAlmostEqual(w.x, 0.0)
        self.assertAlmostEqual(w.y, 1.0)
        self.assertAlmostEqual(math.degrees(w.yaw), 90.0)


class Buttons(unittest.TestCase):
    """The controller should do what braille_server_node does, known gaps included."""

    def setUp(self):
        self.board = Board()
        self.ctl = Controller(World('Two-box test'), self.board)

    def run_board(self):
        finish(self.board, real_speed=False)
        return self.board.cells

    def test_center_at_startup_shows_the_camera_view(self):
        self.ctl.press('CENTER')  # split view, but no map rendered yet
        self.assertIn('one map is missing', self.ctl.message)
        self.assertTrue(np.array_equal(self.ctl.processed, self.ctl.image_map))

    def test_right_without_a_map_fails_like_the_node(self):
        self.ctl.press('RIGHT')
        self.assertEqual(self.ctl.mode, 'occupancy')
        self.assertEqual(self.ctl.message, 'Update failed: No occupancy map available.')
        self.assertFalse(self.board.busy)

    def test_center_in_occupancy_view_shows_the_boxes(self):
        self.ctl.press('RIGHT')
        self.ctl.press('CENTER')
        cells = np.array(self.run_board())
        # the map square sits in cells 11-28; the left box is in its left half, the right box in its right
        left, right = cells[:, 12:19], cells[:, 21:28]
        rows_with_dots = [i for i in range(1, 8) if cells[i, 12:28].any()]
        self.assertTrue(rows_with_dots)
        self.assertTrue(left[3:8].any() and right[3:8].any())

    def test_bug2_side_padding_is_raised(self):
        self.ctl.press('RIGHT')
        self.ctl.press('CENTER')
        cells = np.array(self.run_board())
        self.assertTrue((cells[:, :10] == 0x3F).all() and (cells[:, 30:] == 0x3F).all())

    def test_row0_resets_and_keeps_the_camera_map(self):
        self.ctl.press('CENTER')
        self.run_board()
        self.ctl.press('ROW_0')
        self.assertEqual(self.run_board(), [BLANK] * ROWS)
        self.assertIsNone(self.ctl.occupancy_map)
        self.assertIsNotNone(self.ctl.image_map)

    def test_other_buttons(self):
        self.ctl.press('LEFT')
        self.assertIn('warm-up', self.ctl.message)
        self.ctl.press('LINE_9')
        self.assertIn('no action', self.ctl.message)
        self.assertFalse(self.board.busy)

    def test_view_cycle(self):
        modes = []
        for _ in range(3):
            self.ctl.press('RIGHT')
            modes.append(self.ctl.mode)
        self.assertEqual(modes, ['occupancy', 'image', 'split'])


class Images(unittest.TestCase):
    def test_panels_and_png(self):
        ctl = Controller(World('Thin wall'), Board())
        ctl.press('RIGHT')
        ctl.press('CENTER')
        self.assertEqual(panels.world_view(ctl.world, 5.0).ndim, 3)
        self.assertEqual(panels.map_crop_view(ctl.map_crop).shape, (180, 400, 3))
        self.assertEqual(panels.sent_view(ctl.processed).shape, (180, 400, 3))
        self.assertEqual(panels.camera_view(ctl.camera_debug).shape, (72, 640, 3))
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / 'board.png'
            finish(ctl.board, real_speed=False)
            render_png(ctl.board.cells, path)
            self.assertIsNotNone(cv2.imread(str(path)))


if __name__ == '__main__':
    unittest.main()
