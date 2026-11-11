"""Made-up rooms for the simulator, as nvblox-style occupancy grids, and the camera's pose.

Grid values follow nav_msgs/OccupancyGrid as nvblox publishes it: -1 unknown, 0 free,
100 occupied. Row index = y, column index = x, cell (0, 0) at the origin (lower-left).
The world is flat: unlike nvblox, there's no height band (see docs/simulator.md).
"""
import math

import numpy as np

RESOLUTION = 0.05  # metres per cell, as nvblox's voxel_size on the Jetson
WALL = 0.05        # wall thickness, metres

# Each scene: room bounds (x0, y0, x1, y1), obstacles (x0, y0, x1, y1), camera start (x, y, yaw°).
# The camera looks along +x at yaw 0; +y is to its left.
SCENES = {
    'Two-box test': dict(
        room=(-1.0, -3.0, 6.0, 3.0),
        obstacles=[(1.35, 0.35, 1.65, 0.65),     # 30 cm box, 1.5 m ahead, 0.5 m left
                   (1.80, -0.95, 2.20, -0.55)],  # 40 cm box, 2 m ahead, 0.75 m right
        start=(0.0, 0.0, 0.0)),
    'Thin wall': dict(
        room=(-1.0, -3.0, 6.0, 3.0),
        obstacles=[(3.00, -2.0, 3.05, 2.0),      # 5 cm wall 3 m ahead
                   (2.00, -1.05, 2.10, -0.95)],  # 10 cm pole, 2 m ahead, 1 m right
        start=(0.0, 0.0, 0.0)),
    'Corridor': dict(
        room=(-1.0, -0.6, 12.0, 0.6),
        obstacles=[(4.0, 0.25, 4.4, 0.55)],      # something against the left wall
        start=(0.0, 0.0, 0.0)),
    'Empty room': dict(
        room=(-1.0, -3.0, 6.0, 3.0),
        obstacles=[],
        start=(0.0, 0.0, 0.0)),
}


class World:
    def __init__(self, scene='Two-box test'):
        self.load(scene)

    def load(self, scene):
        spec = SCENES[scene]
        self.scene = scene
        x0, y0, x1, y1 = spec['room']
        # one cell of margin around the room, left unknown
        self.origin_x, self.origin_y = x0 - 2 * RESOLUTION, y0 - 2 * RESOLUTION
        self.width = int(round((x1 - x0) / RESOLUTION)) + 4
        self.height = int(round((y1 - y0) / RESOLUTION)) + 4
        grid = np.full((self.height, self.width), -1, dtype=np.int8)
        self._fill(grid, (x0, y0, x1, y1), 0)
        for wx0, wy0, wx1, wy1 in ((x0, y0, x1, y0 + WALL), (x0, y1 - WALL, x1, y1),
                                   (x0, y0, x0 + WALL, y1), (x1 - WALL, y0, x1, y1)):
            self._fill(grid, (wx0, wy0, wx1, wy1), 100)
        for box in spec['obstacles']:
            self._fill(grid, box, 100)
        self.grid = grid
        x, y, yaw_deg = spec['start']
        self.x, self.y, self.yaw = x, y, math.radians(yaw_deg)

    def _cell(self, x, y):
        return int(round((x - self.origin_x) / RESOLUTION)), int(round((y - self.origin_y) / RESOLUTION))

    def _fill(self, grid, rect, value):
        (c0, r0), (c1, r1) = self._cell(rect[0], rect[1]), self._cell(rect[2], rect[3])
        grid[max(r0, 0):max(r1, r0 + 1), max(c0, 0):max(c1, c0 + 1)] = value

    @property
    def meta(self):
        """The metadata heightmap_generator_node keeps from each OccupancyGrid message."""
        return {'resolution': RESOLUTION, 'origin_x': self.origin_x, 'origin_y': self.origin_y,
                'width': self.width, 'height': self.height}

    def move(self, metres):
        self.x += metres * math.cos(self.yaw)
        self.y += metres * math.sin(self.yaw)

    def turn(self, degrees):
        """Positive turns left (counter-clockwise seen from above), as in ROS."""
        self.yaw = math.atan2(math.sin(self.yaw + math.radians(degrees)),
                              math.cos(self.yaw + math.radians(degrees)))
