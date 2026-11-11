# Foxglove

*Ongoing page: setup, what works, and what we've learned. Add to the [log](#log) as you
troubleshoot and experiment.*

[Foxglove](https://foxglove.dev) shows the pipeline's live ROS data (the occupancy map,
the camera's pose, debug images) on your own computer. The Jetson only sends the data and
your computer draws it, so it works where `rviz2` over X11 doesn't (see
[X11 forwarding](x11_forwarding_vscode.md)).

## Quick start

1. Install the [Foxglove app](https://foxglove.dev/download) (or use
   [app.foxglove.dev](https://app.foxglove.dev) in Chrome) and sign in. The Free plan
   covers 3 users; the Academic plan is $0 with a university email address.
2. Make sure your computer is on the same Tailscale network as the Jetson.
3. In the container, start the pipeline with the bridge:

   ```bash
   launch_braille run_foxglove:=True
   ```

   The bridge's startup log should show the Jetson's Tailscale address:
   `[WS] WebSocket server listening at ws://100.x.y.z:8765`.
4. In Foxglove: **Open connection → Foxglove WebSocket** → `ws://mercer-jetson:8765`.
5. Set up the panels below.

## How the bridge is set up

`run_foxglove:=True` makes `braille.launch.py` also start `foxglove_bridge` (version 0.8.5
in our container). It's off by default.

- **View-only.** It runs with `capabilities:=[connectionGraph,assets]`, so Foxglove can't
  publish messages, change parameters or call services. Without this, anyone connected
  could, for example, call `/braille_display/update` and move the board.
- **Tailscale only.** It listens on the Jetson's Tailscale address, found when the launch
  starts, so only devices on our Tailscale network can connect. If there's no Tailscale
  address it falls back to `127.0.0.1` (local connections only). To override:
  `foxglove_address:=<ip>`.

By default the bridge would listen on every network, with no password and full
capabilities. Keep these settings if you ever start it another way. The manual command
we tested with is:

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml address:=<tailscale ip> capabilities:='[connectionGraph,assets]'
```

## Panels

**3D panel**

- Set **Display frame** to `odom`. Leave **Fixed frame** blank.
- Under **Topics** in the panel settings, click the eye icon next to
  `/nvblox_node/static_occupancy_grid` and `/visual_slam/tracking/vo_pose`.

**Image panels.** Pick the topic in the panel's settings (gear icon).

| Topic | What it shows |
| --- | --- |
| `/braille_image_debug` | The camera image at board resolution, blurred, and its edges, side by side. Small and cheap. |
| `/braille/heightmap_generator_debug_viz` | The ego-centric map crop. Only updates when CENTER renders a new map in occupancy mode. |
| `/braille_publish_map_node/debug_image` | That crop after it's converted to raised/lowered dots. |
| `/braille_pipeline/debug_viz` | What was last sent to the board, republished once a second. Until bug #1 is fixed, the board doesn't show every 4th row of this image. |
| `/camera0/color/image_raw` | The raw camera. **Avoid:** see the first problem below. |

## Known problems

**"Server is dropping messages to the client…"**
The raw camera feed (`/camera0/color/image_raw`) is uncompressed: about 28 MB/s at
640×480 and 30 frames per second, which is more than our connection carries. The bridge's
send buffer fills and it drops messages, which can include map and pose updates. Close
that Image panel or switch it to a debug topic. Raising the send buffer limit, as the
warning suggests, doesn't help: the overload is steady, so a bigger buffer only fills up
too, adding lag and using the Jetson's memory. This doesn't affect the pipeline itself.

**Panels stay empty**

- The pipeline isn't running: the **Topics** tab then lists only a few system topics.
- **Display frame** isn't set (it shows a red warning), or the topics aren't turned on in
  the 3D panel's **Topics** section.
- The bridge log prints `Subscribing to topic …` for each topic a panel asks for. If a
  topic is subscribed but its panel stays empty, that's worth recording here.

**The nvblox 3D mesh doesn't display.** That needs NVIDIA's separate Foxglove extension,
which we haven't set up. The occupancy grid displays without it.

**Reported elsewhere, not seen here:** on NVIDIA's forum, someone running Isaac ROS got
only `/tf` through the bridge.

## Ideas to try

- **Camera view:** check the **Topics** tab for `/camera0/color/image_raw/compressed`
  (the compression plugin is installed). JPEG compression costs Jetson CPU while you watch.
- **Shared layout:** export a Foxglove layout with the panels above and commit it to
  `docs/`, so everyone starts from the same view.
- **Calibration aid:** compare the occupancy grid in Foxglove with what the board shows,
  e.g. to check that short objects appear at all. The map only includes obstacles
  between 0.1 m below and 0.3 m above the camera's starting height.
- **Recordings:** record the pipeline's topics with `ros2 bag` and open them in Foxglove
  later, for offline testing without the Jetson.
- **nvblox mesh:** set up NVIDIA's Foxglove extension.

## Log

- **2026-11-03:** First connection. The installed bridge (0.8.5) works with the current
  Foxglove app over Tailscale. Panels were blank until the pipeline was running and the
  Display frame was set to `odom`. Viewing the raw camera triggered "Server is dropping
  messages". `rviz2` over X11 failed the same day, which is why we use Foxglove for 3D.
- **2026-11-10:** `run_foxglove:=True` added to `braille.launch.py` (commit `56ae290`).
  Not yet tested on the Jetson.

## Sources

- [The Foxglove Bridge and Tailscale VPN](https://foxglove.dev/blog/the-foxglove-bridge-and-tailscale-vpn)
- [foxglove_bridge README](https://github.com/foxglove/foxglove-sdk/blob/main/ros/src/foxglove_bridge/README.md)
- [Foxglove getting started guide](https://docs.foxglove.dev/docs/getting-started-guide)
- [Foxglove pricing](https://foxglove.dev/pricing)
- [Isaac ROS: Foxglove bridge only sends /tf (NVIDIA forum)](https://forums.developer.nvidia.com/t/isaac-ros-foxglove-bridge-only-sends-tf-message/324584)
