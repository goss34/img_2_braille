# Jetson snapshot (October 2026)

This branch records the braille code and container setup as they actually ran on the
team's Jetson (`mercer-jetson`). The files were collected on 2026-10-14 (snapshot tool)
and 2026-10-21 (archive of the workspace copy). Compare it with `main` to see what the
Jetson runs that GitHub didn't have:

```bash
git diff main..jetson-snapshot-2026-10
```

## Where each file lives on the Jetson

`$WS` is `/mnt/nova_ssd/mnt/nova_ssd/workspaces/isaac_ros-dev` on the host, mounted as
`/workspaces/isaac_ros-dev` in the container.

| In this repo | On the Jetson |
|---|---|
| `src/braille_interface/` | `$WS/src/isaac_ros_common/braille_interface/` (only the two node files differ from `main`) |
| `src/Dockerfile.braille` | `$WS/src/isaac_ros_common/docker/Dockerfile.yoon` |
| `src/docker/scripts/isaac_ros_workspace_setup.user.sh` | `$WS/src/isaac_ros_common/docker/scripts/` (the image copies it into `/usr/local/bin/scripts/entrypoint_additions/`) |
| `src/udev/99-braille-board.rules` | `/etc/udev/rules.d/99-braille-board.rules` (host, not the container) |

On the Jetson the Dockerfile is named `Dockerfile.yoon` and selected by
`CONFIG_IMAGE_KEY=ros2_humble.realsense.yoon` in
`$WS/src/isaac_ros_common/scripts/.isaac_ros_common-config`. The repo uses the `.braille`
name going forward.

## Left out on purpose

- `braille_interface/bak/`: older copies of the nodes.
- `braille_interface/launch/build`, `install`, `log`: leftovers from running `colcon build`
  inside `launch/`.
- `~/Documents/braille_interface`: an older copy whose only change sets the serial port
  to `/dev/braille`, a name no installed udev rule creates.
- `$WS/src/isaac_ros_common/docker/udev_rules/99-canute-braille.rules`: not used. The
  Dockerfile never copies it, and the host rule above is the live one.

## Not captured yet

- `$WS/src/scripts/reset_realsense.sh`: may differ from `src/scripts/reset_realsense.sh`
  (the setup guide says to edit the camera's USB ID in it).
