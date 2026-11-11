# X11 forwarding with VS Code (macOS)

Show windows from programs on the Jetson on your Mac, through the same SSH connection VS
Code uses. Good for simple GUI tools. For 3D tools like `rviz2`, use
[Foxglove](foxglove.md) instead.

| Tool | Over X11 | Tested |
| --- | --- | --- |
| Simple GUI tools (Qt, Tk, matplotlib), `xeyes` | Works | `xeyes` works (2026-11-03) |
| `rqt_image_view` | Should work; fine for small debug images, slow for the full camera | Not yet |
| `rviz2` | **Doesn't work**: OpenGL fails through XQuartz | Failed (2026-11-03) |
| `realsense-viewer` | Reported to fail (needs OpenGL 3.0) | Not tried |

## Mac setup

1. Install [XQuartz](https://www.xquartz.org), then log out and back in so it can set
   `DISPLAY`.
2. Add the Jetson to `~/.ssh/config`:

   ```text
   Host mercer-jetson
     HostName mercer-jetson
     User mercer
     ForwardX11 yes
     ForwardX11Trusted yes
     XAuthLocation /opt/X11/bin/xauth
   ```

   `ForwardX11Trusted yes` gives programs on the Jetson full access to your XQuartz
   session, so keep it limited to this host.
3. In XQuartz → Settings → Security, leave **Allow connections from network clients**
   off. SSH forwarding doesn't need it.

VS Code's Remote - SSH extension uses this SSH config, and its
`remote.SSH.enableX11Forwarding` setting is on by default, so VS Code needs no extra
settings.

## Test

1. **Plain SSH first.** From a Mac terminal, run `ssh mercer-jetson`, then on the Jetson:

   ```bash
   echo $DISPLAY; which xauth; grep -i '^X11' /etc/ssh/sshd_config
   ```

   Expect something like `localhost:10.0`, a path to `xauth`, and `X11Forwarding yes`.
   Then run `xeyes`; a window should open on your Mac.
2. **VS Code.** Reconnect the remote window, open a *new* terminal (not a restored one), and
   repeat `echo $DISPLAY` and `xeyes`. If `DISPLAY` is empty, run **Remote-SSH: Kill VS
   Code Server on Host** from the Command Palette and reconnect.

## Using it inside the Isaac ROS container

`run_dev.sh` passes `DISPLAY` and `~/.Xauthority` into the container only when the container
**starts**. So:

- Start the container from a terminal where `echo $DISPLAY` works (step 2 above).
- A forwarded display belongs to one SSH connection. If VS Code reconnects and windows stop
  opening, restart the container from the new terminal.
- A container started from the NoMachine desktop or another terminal opens its windows
  there, or fails.

## What we tried for rviz2 (2026-11-03)

`rviz2` in the container failed with `libGL error: No matching fbConfigs or visuals found`
and `failed to load driver: swrast`, then OGRE errors (`Window with name 'OgreWindow(0)'
already exists`) until `Unable to create the rendering window after 100 tries`. The OGRE
errors are a side effect of retrying; the real failure is OpenGL. Neither of these helped:

- turning on indirect OpenGL in XQuartz:
  `defaults write org.xquartz.X11 enable_iglx -bool true`, then restarting XQuartz
- `LIBGL_ALWAYS_INDIRECT=1 rviz2`

For 3D views, use [Foxglove](foxglove.md), or NoMachine, which draws on the Jetson's GPU
and streams the picture.

## Sources

- [VS Code Remote-SSH X11 forwarding issue](https://github.com/microsoft/vscode-remote-release/issues/267)
- [XQuartz indirect OpenGL bug report](https://bugs.freedesktop.org/show_bug.cgi?id=99146)
- [rviz and Gazebo over X forwarding (ROS Answers)](https://answers.ros.org/question/313576/today-in-2019-will-rviz-and-gazebo-work-with-x-forwarding-over-ssh)
- [realsense-viewer over ssh -X](https://gachiemchiep.github.io/cheatsheet/opengl_over_ssh)
