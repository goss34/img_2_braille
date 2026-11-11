# Isaac ROS container

How the Jetson's Docker container is built and started, and how to change it safely.

## How it's put together

`run_dev.sh` (NVIDIA's, in `$ISAAC_ROS_WS/src/isaac_ros_common/scripts/`) builds the image
in layers, chosen by `CONFIG_IMAGE_KEY` in `scripts/.isaac_ros_common-config` next to it.
Ours adds one layer on top of NVIDIA's ROS 2 and RealSense layers:

| In this repo | On the Jetson | What it does |
| --- | --- | --- |
| `src/Dockerfile.braille` | `$ISAAC_ROS_WS/src/isaac_ros_common/docker/Dockerfile.braille` | Adds tools (`usbreset`, `pyserial`, `crcmod`, …) and copies in the startup script |
| `src/docker/scripts/isaac_ros_workspace_setup.user.sh` | `$ISAAC_ROS_WS/src/isaac_ros_common/docker/scripts/` | The startup script, run as `admin` every time the container starts |

The image key is `CONFIG_IMAGE_KEY=ros2_humble.realsense.braille`. Its last part names
our Dockerfile: if you rename the Dockerfile, change the key to match, or `run_dev.sh`
fails with `Could not resolve Dockerfiles for target image ids`.

## What happens at startup

- `run_dev.sh` rebuilds the image every time it starts a container. Unchanged layers come
  from Docker's cache, so this is quick, and edits to the Dockerfile or the startup script
  are picked up automatically at the next start.
- Running `run_dev.sh` while a container is already running opens another shell in it.
- The container gets `DISPLAY` and the host's devices only when it starts. If the Canute
  or camera is unplugged and plugged back in, restart the container.
- The startup script fixes the Canute's permissions and adds the aliases below. It does
  **not** build the workspace (since 2026-11-10).

| Alias | What it does |
| --- | --- |
| `launch_braille` | Starts the pipeline (`ros2 launch braille.launch.py`); add `run_foxglove:=True` for [Foxglove](foxglove.md) |
| `build_braille` | Builds only `braille_interface`: the usual build after a deploy says one is needed |
| `build_ws` | Builds the whole workspace, including nvblox. Slow, and it replaces what the pipeline runs, so back up `install/` first (e.g. `cp -a install install.bak` on the host) |
| `source_ws` | Re-sources the workspace in the current shell |

## Changing the Dockerfile or the startup script

`tools/deploy_braille.sh` only deploys the braille package, so these files are copied by hand.

1. Edit the file on `dev`, commit and push.
2. On the Jetson host, update the repo clone and copy the file into place:

   ```bash
   cd ~/img_2_braille && git pull
   cp src/docker/scripts/isaac_ros_workspace_setup.user.sh "$ISAAC_ROS_WS/src/isaac_ros_common/docker/scripts/"
   ```

3. Exit every shell in the container (that stops it), then start it again with `run_dev.sh`.
   Only the layers from the changed file onward are rebuilt.

**If something goes wrong:**

- `run_dev.sh -b` starts a container without rebuilding the image. Use it if a rebuild
  fails: `run_dev.sh` stops on a failed rebuild instead of falling back to the old image.
- To go back to the last image known to work:

  ```bash
  docker tag isaac_ros_dev-aarch64:known-good isaac_ros_dev-aarch64:latest
  ```

  then start with `run_dev.sh -b`.
- The October 2026 versions of these files are on the `jetson-snapshot-2026-10` branch,
  e.g. `git show jetson-snapshot-2026-10:src/docker/scripts/isaac_ros_workspace_setup.user.sh`.

Saved image tags: `known-good` (the image as of October 2026, before the startup script
stopped building) and `pre-hook-2025` (the build before the startup script was added).

## Known issues

- **Never build as root** (e.g. `sudo colcon build`). It leaves root-owned files in
  `build/`, `install/` and `log/`, and later builds then fail at the install step with
  `cannot set permissions … Operation not permitted`. To fix it, give those files back to
  the normal user on the Jetson host (`-h` changes symlinks themselves, since many point
  to container paths that don't exist on the host):

  ```bash
  sudo find "$ISAAC_ROS_WS"/build "$ISAAC_ROS_WS"/install "$ISAAC_ROS_WS"/log -user root -exec chown -h mercer:mercer {} +
  ```

- **Don't let the image's early layers rebuild.** Our Dockerfile's `apt-get update` would
  fail today because of changes in the package repositories. Those layers come from
  Docker's cache, so don't prune Docker images or its build cache.
- **The Dockerfile's `ISAAC_ROS_WS` is wrong but harmless.** It sets
  `/workspaces/isaac_ros-img`, but `run_dev.sh` overrides it with
  `/workspaces/isaac_ros-dev`. The nvblox fork it clones is only used to install
  dependencies into the image.
- **`build_braille` prints `ERROR setuptools_scm … listing git files failed`.** Harmless:
  the build still finishes.

## History

- **2026-10:** state recorded on the `jetson-snapshot-2026-10` branch. The startup script
  ran `rosdep` and a full `colcon build` on every start; the build always failed.
- **2026-11-10:** the startup script stops building; `build_braille` alias added. On the
  Jetson, only the last two image steps rebuilt and the container started without
  building. `rosdep check` confirmed nothing depended on the old startup step: its
  `rosdep install` could never have worked, because the container user has no rosdep
  cache until it runs `rosdep update` itself. The Dockerfile was renamed from
  `Dockerfile.yoon` to `Dockerfile.braille` and the image key changed to match.
- **2026-11-10:** the 575 root-owned files left in `build/`, `install/` and `log/` by an
  earlier root build were given back to `mercer` (count now 0), so full builds should no
  longer fail at the install step. `build_ws` hasn't been run since.
