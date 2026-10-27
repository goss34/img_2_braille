# Tools

Standalone helpers for checking the Jetson setup and the Canute 360, and for deploying
the package to the Jetson. None are part of the `braille_interface` package. Only
`deploy_braille.sh` changes anything, and it backs up first.

| Tool | What it does | Needs |
|---|---|---|
| `deploy_braille.sh` | Copies the package from a branch or commit into the Isaac ROS workspace, with a backup, rollback and a record of what's deployed | bash, git, rsync (Jetson host) |
| `jetson_snapshot.sh` | Read-only snapshot of the Jetson host or container: code versions, local changes, Docker and ROS setup, devices | bash |
| `snapshot_ros.py` | Called by the snapshot: Python module versions and, with `--live`, one message from each pipeline topic | ROS 2 (container) |
| `canute_test.py` | Sends test patterns to the Canute, logs every frame, times refreshes, logs buttons | Python 3, `pyserial`, `crcmod` |

## Getting the tools onto the Jetson without touching your working copy

The snapshot should record the Jetson exactly as it is, so don't switch branches or pull
first. Copy the tools into the workspace instead, where both the host and the container
can see them:

```bash
cd /path/to/img_2_braille          # the repo clone on the Jetson
git fetch origin dev               # only updates origin/dev; your files are untouched
mkdir -p "$ISAAC_ROS_WS/braille_tools"
git archive origin/dev tools | tar -x -C "$ISAAC_ROS_WS/braille_tools" --strip-components=1
```

A USB stick works too: copy this `tools` folder to `$ISAAC_ROS_WS/braille_tools`.

## 1. Snapshot

On the Jetson host:

```bash
bash "$ISAAC_ROS_WS/braille_tools/jetson_snapshot.sh"
```

If it can't find your repo clone by itself, point it there with `-r /path/to/img_2_braille`.

Then inside the container (`run_dev.sh`):

```bash
bash /workspaces/isaac_ros-dev/braille_tools/jetson_snapshot.sh
```

Optionally, run it once more with the pipeline running (`ros2 launch braille.launch.py`
in another terminal) and with the camera pointed at something:

```bash
bash /workspaces/isaac_ros-dev/braille_tools/jetson_snapshot.sh --live
```

Each run writes a folder and a `.tar.gz` to `$ISAAC_ROS_WS/braille_snapshots/`. Skim
`report.txt`, then bring back the `.tar.gz` files. Values that look like API keys, tokens
or passwords are masked. `--live` also saves one camera frame (`camera_color.png`), so
check it before sharing.

## 2. Canute test

Stop the braille pipeline first: two programs on one serial port corrupt each other.
Inside the container, source the workspace so the tool can find the HDLC code:

```bash
cd /workspaces/isaac_ros-dev && source install/setup.bash
T=/workspaces/isaac_ros-dev/braille_tools/canute_test.py

python3 $T ports                       # confirm the board's port and USB IDs
python3 $T probe                       # size queries and button state; raw replies logged
python3 $T pattern rows                # line order and left/right orientation
python3 $T pattern dots                # dot order within a cell
python3 $T pattern border              # full 27 x 80 frame
python3 $T pattern legacy-border       # the same frame as the pipeline draws it today

python3 $T pattern blank               # clear the board
python3 $T pattern full                # time a full-page refresh (all 9 lines change)
python3 $T pattern full                # same page again: if this is instant, the board skips unchanged lines
python3 $T pattern blank
python3 $T pattern full --rows 0       # time a one-line refresh

python3 $T buttons                     # press each button once
python3 $T probe --extended            # adds VERSION and POLL queries (never sent by the pipeline)
```

After each pattern, press Enter when the board stops moving and the tool records how
long the refresh took. Compare the board with the Unicode braille the tool prints. Logs
go to `canute_logs/` in the current folder.

The board is a USB serial device, so this also works from a laptop if the Canute is
plugged in there (`pip install pyserial crcmod`, then `ports` to find it, e.g.
`--port /dev/cu.usbmodem1101` on macOS).

### What to note for each pattern

- **rows**: which line has 1 cell (should be the top), and whether the cells start at the left.
- **dots**: each group of six cells should match the terminal. If they differ, write down
  which physical dot rose for each cell.
- **border** vs **legacy-border**: legacy should be missing its bottom edge.
- **full**: with calipers, measure the dot spacing inside a cell, the distance from a
  cell to the next one, and the distance from one line to the next.
- **buttons**: the raw bytes for every physical button, including any shown as `unmapped`.

## 3. Deploying code to the Jetson

Run this on the Jetson host, not in the container, from the repo clone that lives outside
the workspace. It deploys only `src/braille_interface`, into
`$ISAAC_ROS_WS/src/isaac_ros_common/braille_interface`.

```bash
cd ~/img_2_braille && git pull                             # get the latest tools
tools/deploy_braille.sh --status                           # what's deployed, and any hand edits since
tools/deploy_braille.sh --dry-run jetson-snapshot-2026-10  # see what would change
tools/deploy_braille.sh jetson-snapshot-2026-10            # deploy (asks first)
```

- **A branch name deploys GitHub's version** (`origin/<branch>`, fetched first), so push
  before deploying. Tags and commit hashes work too. The clone's own checkout never
  changes, so people can deploy different branches without switching branches.
- **Every deploy is backed up** to `$ISAAC_ROS_WS/braille_deploy_backups/`.
  `--rollback` undoes the most recent deploy or rollback; `--rollback FILE` restores a
  specific backup; `--list` shows the history.
- **It tells you when to rebuild.** Edits to existing `.py` files take effect the next time
  you launch the pipeline. Changes to `setup.py`, `package.xml`, `resource/` or the set of
  launch files need one build of this package inside the container:
  `colcon build --symlink-install --packages-select braille_interface`.
- **Make changes in the repo, not in the workspace.** `--status` lists any files edited by
  hand since the last deploy. A deploy replaces them, though the backup keeps a copy.
- The first deploy of `jetson-snapshot-2026-10` changes no code. It only removes
  `braille_interface/bak/` and the stray `launch/build`, `launch/install` and `launch/log`
  folders, and those stay in the backup.
