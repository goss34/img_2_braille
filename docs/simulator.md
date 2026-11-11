# Braille simulator (Virtual Canute)

Runs the braille display logic on a laptop, with no ROS, Jetson or Canute: a window shows
which pins the board would raise, with clickable buttons, a made-up room and a camera you
can move around in it.

It uses the same drawing code the Jetson runs (`braille_interface/display.py`), so a change
there shows up in the simulator before it's deployed.

## Running it

Needs Python 3 with numpy, OpenCV and Tk. On a Mac with Homebrew, `python3` (3.15) has
numpy and OpenCV, and Tk comes from `brew install python-tk@3.15`.

```bash
python3 tools/braille_sim
```

Options: `--instant` (refresh the board at once instead of at real speed),
`--line-seconds 1.3` (time per changed line), `--scene "Thin wall"`.

## The window

- **Board.** 9 lines of 40 cells, 6 dots each, at standard braille spacing (2.5 mm
  between dots, 6 mm cell to cell, 10 mm line to line). Filled dots are raised. The line
  being refreshed is highlighted.
- **Line buttons** (left of each line) and **LEFT / CENTER / RIGHT** (below), doing what
  the code does: CENTER updates the board, RIGHT switches view (split → occupancy →
  image), ROW_0 resets, LEFT sends a warm-up (no effect here). Which physical button is
  `ROW_0` isn't confirmed yet, and the 9th line button isn't mapped in the code.
- **Status line:** the current view, the refresh progress, and what the last button did.
- **Room from above:** the camera (red arrow) and the 5 m window the occupancy view shows
  (blue).
- **Map crop:** what `heightmap_generator_node` publishes. Grey is the "middle" value
  128, which later becomes raised dots.
- **Sent to the board:** the image the server packs into braille. Rows tinted red are the
  4th dot row of each cell, which the Canute never shows; raised dots there are drawn red.
- **Camera → edges:** the image view's input. It starts with a test picture; load a
  photo to try a real one. Like on the Jetson, a new camera image reaches the board at the
  next update.
- **Controls:** scene, load camera photo, real-speed refresh (on by default), seconds per
  changed line, and save the board as a PNG.

| Key | Does |
| --- | --- |
| W / S | Move the camera 25 cm forward / back |
| A / D | Turn the camera 15° left / right |
| J / K / L | LEFT / CENTER / RIGHT |
| 1–9 | Line buttons (1 = `ROW_0`) |

**To see the occupancy map:** press L (RIGHT) to switch to occupancy view, then K (CENTER).
At startup in split view, CENTER shows the camera view alone, because no map has been
rendered yet. That's how the real pipeline behaves too (a known gap).

## How faithful it is

**The same as on the Jetson:**
- Everything from the occupancy grid to the bytes sent to the board: the ego-centric crop,
  the conversion to dots, the view layout and the braille packing. This is the shared
  `display.py`, known bugs included.
- The board receiving 6 dots per cell, so the 4th dot row is dropped (bug #1).
- The refresh timing measured on our board: changed lines one at a time (about 0.6–1.5 s
  each; 1.3 s by default here), and unchanged lines skipped almost instantly.

**Mirrored, not shared:** the button logic in `tools/braille_sim/controller.py` copies
`braille_server_node`, including its known gaps (split view and RIGHT don't render a new
map). Keep the two in step until that logic moves into shared code too.

**Not modelled:**
- **Height:** the world is flat. On the Jetson, nvblox only maps obstacles between 0.1 m
  below and 0.3 m above the camera's starting height.
- **Exploration:** the simulator knows the whole room. nvblox only maps what the camera
  has seen, and everything else is "unknown" (raised).
- **Sensor noise, SLAM drift, a moving camera picture**, and the server node freezing
  while it sends lines to the board.

## Tests

No window needed:

```bash
python3 -m unittest discover -s src/braille_interface/test -p 'test_display.py'
python3 -m unittest discover -s tools/braille_sim -p 'test_*.py'
```

Tests named `bug1`, `bug2` and `bug3` pin today's known-wrong behaviour. When a bug is
fixed, its test changes on purpose.

## Using it to work on the display

1. Change `src/braille_interface/braille_interface/display.py` on `dev`.
2. Run the tests, then look at the result in the simulator.
3. Commit, push, and deploy to the Jetson (`tools/deploy_braille.sh dev`).

## Ideas

- Load real occupancy grids recorded on the Jetson (`jetson_snapshot.sh --live` saves one).
- A mode where the real `braille_server_node` drives the simulator over the Canute's serial
  protocol, for testing the node itself without the board.
