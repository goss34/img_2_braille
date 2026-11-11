# img_2_braille

Turns what a camera sees into tactile maps on a braille display. An Intel RealSense D455
feeds NVIDIA Isaac ROS (visual SLAM and nvblox mapping) on a Jetson Orin Nano, and the
`braille_interface` ROS 2 package draws an occupancy map or a camera edge image on a
Canute 360 braille display.

## Documentation

Setting up a Jetson, in order:
1. [Jetson first steps](docs/jetson_setup.md): Docker, swap, power mode
2. [nvblox setup](docs/nvblox_setup.md): Isaac ROS workspace, RealSense and nvblox
3. [Braille package setup](docs/braille_setup.md): installing and running `braille_interface`

Working with the Jetson:
- [Foxglove](docs/foxglove.md): watching the pipeline from your own computer (ongoing page)
- [X11 forwarding with VS Code](docs/x11_forwarding_vscode.md): Jetson windows on a Mac
- [Tools](docs/tools.md): deploying code, Jetson snapshots, Canute board tests
- [Jetson snapshot, October 2026](docs/jetson_snapshot.md): what the Jetson ran before this
  repo was used to deploy it

## Repository layout

| Path | What's there |
|---|---|
| `src/braille_interface/` | The ROS 2 package |
| `src/Dockerfile.braille`, `src/docker/scripts/` | The Isaac ROS container's extra layer and its startup script |
| `src/scripts/` | `reset_realsense.sh` |
| `src/udev/` | udev rule for the Canute 360 (installed on the Jetson host) |
| `tools/` | Deploy, snapshot and Canute test scripts |
| `docs/` | Documentation |

## Branches

- `dev`: day-to-day work and experiments, and what's deployed on the Jetson.
- `jetson-snapshot-2026-10`: frozen record of the Jetson before deploys came from this repo.
- `main`: the production branch.

To get a change onto the Jetson: commit it to `dev`, push, then on the Jetson host run
`tools/deploy_braille.sh dev` from the repo clone. See [Tools](docs/tools.md#3-deploying-code-to-the-jetson).
