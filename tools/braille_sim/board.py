"""A model of the Canute 360: what it shows, and how it refreshes.

Cells hold the byte the Canute receives (bits 0-5 = dots 1-6, Unicode braille order).
Refresh behaviour measured on our board (canute_logs, 2026-10-14):
- lines are set one at a time, in the order received
- a changed line takes about 0.6-1.5 s depending on the pattern (1.3 s by default here)
- a line that's already showing the same dots is skipped in about 0.06 s
"""
from collections import deque

import cv2
import numpy as np

ROWS, CELLS = 9, 40
LINE_SECONDS = 1.3
SKIP_SECONDS = 0.06

# Standard braille spacing; replace with caliper measurements of our board when we have them.
DOT_PITCH_MM = 2.5    # between dots inside a cell
CELL_PITCH_MM = 6.0   # cell to cell
LINE_PITCH_MM = 10.0  # line to line

# (dot row, dot column) of each bit
BIT_POSITIONS = {0: (0, 0), 1: (1, 0), 2: (2, 0), 3: (0, 1), 4: (1, 1), 5: (2, 1)}


def dot_position_mm(line, cell, bit):
    """Centre of one dot, in mm from the top-left of the reading area."""
    r, c = BIT_POSITIONS[bit]
    return cell * CELL_PITCH_MM + c * DOT_PITCH_MM, line * LINE_PITCH_MM + r * DOT_PITCH_MM


class Board:
    def __init__(self, line_seconds=LINE_SECONDS, skip_seconds=SKIP_SECONDS):
        self.cells = [[0] * CELLS for _ in range(ROWS)]
        self.line_seconds, self.skip_seconds = line_seconds, skip_seconds
        self.queue = deque()   # (line, cells) waiting
        self.active = None     # (line, cells, finishes_at)

    def send_line(self, line, cells):
        self.queue.append((line, [b & 0x3F for b in cells]))

    def send_page(self, pages):
        for line, cells in enumerate(pages):
            self.send_line(line, cells)

    @property
    def busy(self):
        return self.active is not None or bool(self.queue)

    def refreshing_line(self):
        return self.active[0] if self.active else None

    def tick(self, now, real_speed=True):
        """Advance the refresh to time `now` (seconds). Returns True if any dots changed."""
        changed = False
        while True:
            start = now
            if self.active:
                line, cells, ends = self.active
                if real_speed and now < ends:
                    return changed
                changed |= self.cells[line] != cells
                self.cells[line] = cells
                self.active = None
                if real_speed:
                    start = ends  # the next line starts when this one finished, not at this tick
            if not self.queue:
                return changed
            line, cells = self.queue.popleft()
            duration = self.skip_seconds if self.cells[line] == cells else self.line_seconds
            self.active = (line, cells, start + duration)

    def seconds_left(self, now):
        """Estimated time until the queued lines are done."""
        if not self.busy:
            return 0.0
        t = max(self.active[2] - now, 0.0) if self.active else 0.0
        shown = [list(c) for c in self.cells]
        if self.active:
            shown[self.active[0]] = self.active[1]
        for line, cells in self.queue:
            t += self.skip_seconds if shown[line] == cells else self.line_seconds
            shown[line] = cells
        return t


def render_png(cells, path, px_per_mm=4.0, refreshing=None):
    """Draw the board like the app does: filled dots raised, small rings lowered."""
    margin = 6.0
    w = int((CELLS * CELL_PITCH_MM + margin) * px_per_mm)
    h = int((ROWS * LINE_PITCH_MM + margin) * px_per_mm)
    img = np.full((h, w, 3), 245, np.uint8)
    for line in range(ROWS):
        if line == refreshing:
            y0 = int((line * LINE_PITCH_MM + 1.5) * px_per_mm)
            cv2.rectangle(img, (0, y0), (w, y0 + int(LINE_PITCH_MM * px_per_mm)), (180, 235, 255), -1)
        for cell in range(CELLS):
            for bit in range(6):
                x, y = dot_position_mm(line, cell, bit)
                centre = (int((x + margin) * px_per_mm), int((y + margin) * px_per_mm))
                if cells[line][cell] >> bit & 1:
                    cv2.circle(img, centre, int(0.8 * px_per_mm), (40, 40, 40), -1, cv2.LINE_AA)
                else:
                    cv2.circle(img, centre, max(int(0.3 * px_per_mm), 1), (190, 190, 190), 1, cv2.LINE_AA)
    cv2.imwrite(str(path), img)
