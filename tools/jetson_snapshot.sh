#!/usr/bin/env bash
# Read-only snapshot of the Jetson setup behind the braille pipeline.
#
# Run it twice, from a terminal:
#   1. on the Jetson host:                 bash jetson_snapshot.sh
#   2. inside the Isaac ROS container:     bash jetson_snapshot.sh
#      or, with the pipeline running:      bash jetson_snapshot.sh --live
#
# It never changes anything on the system. The only things it writes are its output
# folder and a .tar.gz of it, by default in <workspace>/braille_snapshots/, which the
# host and the container can both see. Skim report.txt before sharing it.
#
# Options:
#   -r DIR       the img_2_braille repo clone (default: the repo holding this script,
#                otherwise found by searching for braille_interface copies)
#   -w DIR       Isaac ROS workspace (default: $ISAAC_ROS_WS on the host,
#                /workspaces/isaac_ros-dev in the container)
#   -o DIR       output folder (default: <workspace>/braille_snapshots)
#   --mode MODE  host or container (default: auto-detected)
#   --live       container only: also sample live topics and node parameters

MODE=""; REPO=""; WS=""; OUT_BASE=""; LIVE=0
while [ $# -gt 0 ]; do
  case "$1" in
    -r|-w|-o|--mode)
      [ $# -ge 2 ] || { echo "$1 needs a value" >&2; exit 2; }
      case "$1" in -r) REPO="$2" ;; -w) WS="$2" ;; -o) OUT_BASE="$2" ;; --mode) MODE="$2" ;; esac
      shift 2 ;;
    --live) LIVE=1; shift ;;
    -h|--help) sed -n '2,21p' "$0" | cut -c3-; exit 0 ;;
    *) echo "Unknown option: $1 (see --help)" >&2; exit 2 ;;
  esac
done

if [ -z "$MODE" ]; then
  if [ -f /.dockerenv ]; then MODE=container; else MODE=host; fi
fi
case "$MODE" in host|container) ;; *) echo "--mode must be host or container" >&2; exit 2 ;; esac

if [ -z "$WS" ]; then
  if [ "$MODE" = container ]; then WS=/workspaces/isaac_ros-dev; else WS="${ISAAC_ROS_WS:-}"; fi
fi
WS="${WS%/}"; REPO="${REPO%/}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -z "$REPO" ]; then
  top="$(git -C "$SCRIPT_DIR" rev-parse --show-toplevel 2>/dev/null)"
  [ -n "$top" ] && [ -f "$top/src/braille_interface/package.xml" ] && REPO="$top"
fi

if [ -z "$OUT_BASE" ]; then
  if [ -n "$WS" ] && [ -d "$WS" ]; then OUT_BASE="$WS/braille_snapshots"; else OUT_BASE="$PWD/braille_snapshots"; fi
fi
OUT="$OUT_BASE/${MODE}_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT/files" "$OUT/patches" || exit 1
R="$OUT/report.txt"

# macOS has no timeout(1); this only matters when testing the script off the Jetson.
if ! command -v timeout >/dev/null 2>&1; then
  timeout() { shift; "$@"; }
  export -f timeout
fi

# ---------------------------------------------------------------- helpers

section() { printf '\n\n==================== %s ====================\n' "$1" >>"$R"; echo "  - $1"; }
note() { printf '%s\n' "$*" >>"$R"; }

# cap 'shell command': append the command, its output and any non-zero exit code to the report.
cap() {
  printf '\n$ %s\n' "$1" >>"$R"
  bash -c "$1" >>"$R" 2>&1 </dev/null
  local rc=$?
  [ $rc -ne 0 ] && printf '[exit %d]\n' "$rc" >>"$R"
  return 0
}

# slug PATH: a readable file name for a path, e.g. ws__src__isaac_ros_common__docker__Dockerfile.yoon
slug() {
  local p="$1"
  if [ -n "$WS" ] && [ "${p#"$WS"/}" != "$p" ]; then p="ws/${p#"$WS"/}"
  elif [ "${p#"$HOME"/}" != "$p" ]; then p="home/${p#"$HOME"/}"; fi
  echo "$p" | sed 's#^/##; s#[/ ]#__#g'
}

# keep FILE [NAME]: copy a config file into files/.
keep() {
  local src="$1" name="${2:-$(slug "$1")}"
  if [ -f "$src" ]; then
    cp "$src" "$OUT/files/$name" && note "kept: $src -> files/$name"
  else
    note "missing: $src"
  fi
}

rp() { readlink -f "$1" 2>/dev/null || (cd "$1" 2>/dev/null && pwd -P) || echo "$1"; }

pyhashes() {
  (cd "$1" && find . -name '*.py' -not -path '*/__pycache__/*' | LC_ALL=C sort | while IFS= read -r f; do
    printf '%s  %s\n' "$(md5sum <"$f" | cut -c1-32)" "$f"
  done)
}

# git_state DIR LABEL COPY_UNTRACKED: branch, commit, remotes (credentials stripped), local
# changes and stashes. Uncommitted diffs go to patches/; untracked files optionally to files/.
git_state() {
  local d="$1" label="$2" g="git -c 'safe.directory=*' -C '$1'" untracked=normal
  [ "$3" = 1 ] && untracked=all
  cap "$g rev-parse --abbrev-ref HEAD; $g log -1 --format='%h %ad %s' --date=short"
  cap "$g remote -v | sed -E 's#://[^/@]*@#://<redacted>@#'"
  note "(ahead / behind upstream, as of the last fetch; this script does not fetch)"
  cap "$g rev-list --left-right --count 'HEAD...@{upstream}'"
  cap "$g status --porcelain --untracked-files=$untracked -- . ':(exclude)*.pyc' | head -200"
  cap "$g stash list"
  local p="$OUT/patches/${label}_uncommitted.diff"
  git -c 'safe.directory=*' -C "$d" diff HEAD -- . ':(exclude)*.pyc' >"$p" 2>/dev/null
  if [ -s "$p" ]; then note "uncommitted changes -> patches/$(basename "$p")"; else rm -f "$p"; fi
  [ "$3" = 1 ] || return 0
  local dest="$OUT/files/${label}_untracked"
  git -c 'safe.directory=*' -C "$d" ls-files --others --exclude-standard -- . ':(exclude)*.pyc' 2>/dev/null |
  while IFS= read -r f; do
    [ -f "$d/$f" ] || continue
    if [ "$(wc -c <"$d/$f" | tr -d ' ')" -gt 2000000 ]; then note "skipped large untracked file: $f"; continue; fi
    mkdir -p "$dest/$(dirname "$f")" && cp -p "$d/$f" "$dest/$f"
  done
  [ -d "$dest" ] && note "untracked files copied -> files/$(basename "$dest")/"
  return 0
}

env_section() {
  cap "env | grep -E '^(ROS_|RMW_|CYCLONEDDS|FASTRTPS|ISAAC|AMENT_PREFIX_PATH|COLCON|PYTHONPATH|LD_LIBRARY_PATH|CUDA|NVIDIA_VISIBLE|NVIDIA_DRIVER|USERNAME|HOME=)' | sort"
  local uri="${CYCLONEDDS_URI#file://}"
  [ -n "$uri" ] && [ -f "$uri" ] && keep "$uri" cyclonedds_config.xml
  return 0
}

# ---------------------------------------------------------------- sections

copies_section() {
  section "braille_interface copies on this machine"
  local roots="$WS/src" c top n=0
  if [ "$MODE" = host ]; then
    roots="$roots $HOME /mnt/nova_ssd"
    [ -n "$REPO" ] && roots="$roots $(dirname "$REPO")"
  fi
  note "searched: $roots"
  for root in $roots; do
    [ -d "$root" ] || continue
    timeout 120 find "$root" -maxdepth 9 \( -name .git -o -name build -o -name install -o -name log \
      -o -name node_modules -o -name isaac_ros_assets -o -name '.cache' -o -name docker -o -name snap \
      -o -name braille_snapshots -o -name Library \) -prune \
      -o -type f -path '*/braille_interface/package.xml' -print 2>/dev/null
  done | while IFS= read -r p; do rp "$(dirname "$p")"; done | LC_ALL=C sort -u >"$OUT/.copies"

  while IFS= read -r c; do
    n=$((n + 1))
    note ""; note "----- copy $n: $c"
    top="$(git -c 'safe.directory=*' -C "$c" rev-parse --show-toplevel 2>/dev/null)"
    note "inside git repo: ${top:-no}"
    if [ -z "$REPO" ] && [ -n "$top" ] && [ "$(rp "$top/src/braille_interface")" = "$c" ]; then
      REPO="$top"; note "-> treating this as the img_2_braille repo clone"
    fi
    { echo "md5 of .py files:"; pyhashes "$c"; } >>"$R"
  done <"$OUT/.copies"
  [ $n -eq 0 ] && note "no copies found"
  note ""
  note "expected workspace location:"
  cap "ls -ld '$WS/src/isaac_ros_common/braille_interface'"
  return 0
}

compare_section() {
  if [ -z "$REPO" ]; then
    section "Comparison with the repo clone"
    note "repo clone not found; rerun with -r /path/to/img_2_braille to compare"
    return 0
  fi
  section "Repo clone: $REPO"
  git_state "$REPO" repo 1

  section "braille_interface copies vs the repo clone"
  note "(in the diffs, '-' lines are the repo clone and '+' lines are the copy)"
  local ref c label
  ref="$(rp "$REPO/src/braille_interface")"
  while IFS= read -r c; do
    [ "$c" = "$ref" ] && continue
    label="copy_$(slug "$c")"
    note ""; note "----- $c"
    if diff -rq -x __pycache__ -x '*.pyc' -x '*.egg-info' "$ref" "$c" >>"$R" 2>&1; then
      note "(identical to the repo)"
    else
      diff -ruN -x __pycache__ -x '*.pyc' -x '*.egg-info' "$ref" "$c" >"$OUT/patches/$label.diff" 2>&1
      note "full diff -> patches/$label.diff"
    fi
  done <"$OUT/.copies"

  [ -n "$WS" ] || return 0
  section "Other deployed files vs the repo clone"
  local pair a b
  for pair in "src/scripts/reset_realsense.sh|$WS/src/scripts/reset_realsense.sh" \
              "src/Dockerfile.braille|$WS/src/isaac_ros_common/docker/Dockerfile.braille" \
              "src/Dockerfile.braille|$WS/src/isaac_ros_common/docker/Dockerfile.yoon"; do
    a="$REPO/${pair%%|*}"; b="${pair#*|}"
    if [ -f "$b" ]; then cap "if diff -u '$a' '$b'; then echo '(identical)'; fi"; else note "not present: $b"; fi
  done
}

workspace_section() {
  section "Workspace layout"
  if [ -z "$WS" ] || [ ! -d "$WS" ]; then note "workspace not found: '$WS' (pass -w DIR)"; return 0; fi
  cap "ls -la '$WS' '$WS/src'"
  cap "ls -la '$WS/src/isaac_ros_common'"
  cap "ls -la '$WS/.isaac_ros_built' '$HOME/.isaac_ros_built'"

  section "Which braille_interface source colcon built"
  cap "ls -la '$WS/build/braille_interface'"
  cap "find '$WS/install/braille_interface' -maxdepth 6 \( -name '*.egg-link' -o -name '*.pth' \) -print -exec cat {} \;"

  section "Build files not owned by uid $(id -u) (root-owned files break colcon builds)"
  local n
  n="$(find "$WS/build" "$WS/install" "$WS/log" ! -user "$(id -u)" 2>/dev/null | wc -l | tr -d ' ')"
  note "count: $n"
  find "$WS/build" "$WS/install" "$WS/log" ! -user "$(id -u)" 2>/dev/null | head -20 >>"$R"

  section "Last colcon build"
  cap "ls -la '$WS/log/latest_build'"
  cap "grep -c 'JobEnded' '$WS/log/latest_build/events.log'"
  note "(packages that did not end with rc 0:)"
  cap "grep -h 'JobEnded' '$WS/log/latest_build/events.log' | grep -v \"'rc': 0\" | head -40"

  section "Git repositories in the workspace"
  local g d
  for g in "$WS"/src/*/.git "$WS"/src/*/*/.git; do
    [ -e "$g" ] || continue
    d="$(dirname "$g")"
    note ""; note "----- $d"
    git_state "$d" "ws_$(echo "${d#"$WS/src/"}" | tr '/ ' '__')" 0
  done
}

host_sections() {
  section "System"
  cap "tr -d '\0' </proc/device-tree/model; echo"
  cap "uname -a"
  cap "cat /etc/nv_tegra_release"
  cap "grep -E '^(PRETTY_NAME|VERSION_ID)=' /etc/os-release"
  cap "dpkg-query -W -f='\${Package} \${Version}\n' nvidia-jetpack nvidia-l4t-core"
  cap "nvpmodel -q 2>&1 | head -5"
  cap "free -m; swapon --show"
  cap "df -h / '$WS'"

  section "Docker"
  cap "docker --version"
  cap "docker info 2>/dev/null | grep -Ei 'server version|runtimes|default runtime|storage driver|docker root dir'"
  cap "docker images --format 'table {{.Repository}}:{{.Tag}}\t{{.ID}}\t{{.CreatedSince}}\t{{.Size}}'"
  cap "docker ps -a --format 'table {{.Names}}\t{{.Image}}\t{{.Status}}'"
  keep /etc/docker/daemon.json
  local img
  for img in $(docker images --format '{{.Repository}}:{{.Tag}}' 2>/dev/null | grep -i 'isaac_ros_dev' | head -3); do
    docker history --no-trunc --format '{{.CreatedSince}}\t{{.CreatedBy}}' "$img" \
      >"$OUT/files/docker_history_$(echo "$img" | tr '/:' '__').txt" 2>&1
    note "build steps of $img -> files/docker_history_$(echo "$img" | tr '/:' '__').txt"
  done

  section "Isaac ROS container config"
  keep "$HOME/.isaac_ros_common-config" home_isaac_ros_common-config
  keep "$HOME/.isaac_ros_dev-dockerargs" home_isaac_ros_dev-dockerargs
  if [ -n "$WS" ]; then
    local irc="$WS/src/isaac_ros_common" f
    keep "$irc/scripts/.isaac_ros_common-config"
    keep "$irc/.isaac_ros_common-config"
    keep "$irc/.isaac_ros_dev-dockerargs"
    cap "ls -la '$irc/docker' '$irc/docker/scripts'"
    for f in "$irc/docker/Dockerfile.yoon" "$irc/docker/Dockerfile.braille" "$irc"/docker/scripts/*; do
      [ -f "$f" ] && keep "$f"
    done
  fi

  section "Serial and USB devices"
  cap "lsusb"
  cap "ls -l /dev/ttyACM* /dev/ttyUSB*; ls -l /dev/serial/by-id/"
  local d
  for d in /dev/ttyACM*; do
    [ -e "$d" ] && cap "udevadm info -q property -n '$d' | grep -E '^(ID_VENDOR|ID_VENDOR_ID|ID_MODEL|ID_MODEL_ID|ID_SERIAL_SHORT|DEVLINKS)='"
  done
  cap "ls -la /etc/udev/rules.d/"
  cap "grep -lE '8086|ttyACM|dialout' /etc/udev/rules.d/*"
  cap "id"

  section "Shell environment (ROS / DDS related only)"
  env_section
  cap "grep -nE 'ISAAC|ROS|CYCLONE|RMW|DDS|alias |source ' '$HOME/.bashrc'"
}

source_ros() {
  section "Sourcing ROS"
  local s
  for s in /opt/ros/humble/setup.bash "$WS/install/setup.bash"; do
    if [ -f "$s" ]; then
      note "source $s"
      # shellcheck disable=SC1090
      . "$s" >>"$R" 2>&1 || note "[source failed]"
    else
      note "missing: $s"
    fi
  done
}

container_sections() {
  section "Container identity"
  cap "id; hostname"
  cap "tr '\0' ' ' </proc/1/cmdline; echo"
  cap "cat /etc/nv_tegra_release; grep -E '^PRETTY_NAME=' /etc/os-release"

  section "Container entrypoint"
  cap "ls -la /usr/local/bin/scripts /usr/local/bin/scripts/entrypoint_additions"
  local f
  for f in /usr/local/bin/scripts/workspace-entrypoint.sh /usr/local/bin/scripts/entrypoint_additions/*; do
    [ -f "$f" ] && keep "$f"
  done

  section "Environment (ROS / DDS related only)"
  env_section

  section "Python"
  cap "which -a python3; python3 --version; pip3 --version"
  pip3 list --format=freeze >"$OUT/files/pip_freeze.txt" 2>&1
  note "pip3 list -> files/pip_freeze.txt"
  cap "python3 '$SCRIPT_DIR/snapshot_ros.py' versions"

  section "OpenCV installs (a mismatch here broke nvblox builds before)"
  cap "dpkg -l | grep -i opencv"
  cap "grep -Hs 'set(OpenCV_VERSION ' /usr/lib/cmake/opencv4/OpenCVConfig-version.cmake /usr/lib/aarch64-linux-gnu/cmake/opencv4/OpenCVConfig-version.cmake /usr/local/lib/cmake/opencv4/OpenCVConfig-version.cmake"
  cap "ls -la /usr/lib/libopencv_core* /usr/lib/aarch64-linux-gnu/libopencv_core* /usr/local/lib/libopencv_core*"

  section "ROS packages"
  dpkg-query -W -f='${Package} ${Version}\n' 'ros-humble-*' >"$OUT/files/ros_humble_debs.txt" 2>&1
  note "installed ros-humble debs -> files/ros_humble_debs.txt"
  timeout 60 ros2 pkg list >"$OUT/files/ros2_pkg_list.txt" 2>&1
  note "ros2 pkg list -> files/ros2_pkg_list.txt"
  cap "ros2 pkg prefix braille_interface; ros2 pkg executables braille_interface"
  cap "ros2 pkg prefix nvblox_ros; ros2 pkg prefix isaac_ros_visual_slam; ros2 pkg prefix cv_bridge"
  if [ -d "$WS" ]; then
    (cd "$WS" && timeout 120 colcon list) >"$OUT/files/colcon_list.txt" 2>&1
    note "colcon list (packages in the workspace and where they live) -> files/colcon_list.txt"
  fi

  section "Serial device inside the container"
  cap "ls -l /dev/ttyACM*"
}

live_sections() {
  section "Live ROS graph"
  cap "timeout 20 ros2 node list"
  cap "timeout 20 ros2 topic list -t"
  local t n f
  for t in /nvblox_node/static_occupancy_grid /visual_slam/tracking/vo_pose /camera0/color/image_raw \
           /braille/heightmap /braille_map /braille_image; do
    cap "timeout 20 ros2 topic info -v '$t'"
  done

  section "Node parameters"
  for n in /nvblox_node /visual_slam_node /braille_heightmap_node /publish_braille_map; do
    f="$OUT/files/params$(echo "$n" | tr '/' '_').yaml"
    (cd "$OUT/files" && timeout 30 ros2 param dump "$n") >"$f" 2>&1
    note "ros2 param dump $n -> files/$(basename "$f")"
  done

  section "One message from each pipeline topic"
  cap "timeout 90 python3 '$SCRIPT_DIR/snapshot_ros.py' topics --out '$OUT/files' --timeout 20"
}

redact() {
  if ! command -v perl >/dev/null 2>&1; then
    echo "WARNING: perl not found, nothing was redacted. Check the output for secrets before sharing."
    return
  fi
  find "$OUT" -type f -print | while IFS= read -r f; do
    grep -Iq . "$f" || continue  # skip binary and empty files
    perl -pi -e 's/((?:API_?KEY|ACCESS_?KEY|SECRET|TOKEN|PASSWORD|PASSWD)\w*\s*[=:]\s*)\S+/$1<redacted>/gi' "$f"
  done
}

# ---------------------------------------------------------------- main

{
  echo "braille pipeline snapshot"
  echo "mode:      $MODE"
  echo "date:      $(date)"
  echo "host:      $(hostname)"
  echo "user:      $(id -un) (uid $(id -u))"
  echo "workspace: ${WS:-<unknown>}"
  echo "script:    $SCRIPT_DIR/$(basename "$0")"
} >"$R"

echo "Writing snapshot ($MODE) to $OUT"
if [ "$MODE" = host ]; then
  copies_section
  compare_section
  workspace_section
  host_sections
  [ $LIVE = 1 ] && echo "  (--live only applies inside the container; skipped)"
else
  source_ros
  container_sections
  workspace_section
  copies_section
  if [ $LIVE = 1 ]; then live_sections; fi
fi
note ""; note "repo clone used for comparison: ${REPO:-<none>}"

rm -f "$OUT/.copies"
redact
(cd "$OUT_BASE" && tar -czf "$(basename "$OUT").tar.gz" "$(basename "$OUT")")
echo
echo "Done."
echo "  folder:  $OUT"
echo "  archive: $OUT.tar.gz"
echo "Please skim report.txt before sharing it."
if [ "$MODE" = host ]; then
  echo "Next: run this script again inside the Isaac ROS container (see tools/README.md)."
fi
