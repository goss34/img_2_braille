#!/usr/bin/env bash
# Deploy the braille_interface package from a branch or commit of the repo clone into the
# Isaac ROS workspace, with a backup and a record of what is deployed.
#
# Run it on the Jetson host (not inside the container), from the repo clone:
#
#   tools/deploy_braille.sh <branch|tag|commit>   deploy that version (asks before changing anything)
#   tools/deploy_braille.sh --dry-run <ref>       only show what would change
#   tools/deploy_braille.sh --status              show what is deployed and any hand edits since
#   tools/deploy_braille.sh --rollback [FILE]     restore the newest backup (or FILE)
#   tools/deploy_braille.sh --list                show deploy history and backups
#
# A branch name deploys what is on GitHub (origin/<branch>, fetched first), not a possibly
# stale local branch. Only committed code can be deployed. Nothing is checked out, so the
# clone's own branch never changes.
#
# Options:
#   -y, --yes       don't ask for confirmation
#   --no-fetch      don't fetch from origin first
#   -r DIR          the repo clone (default: the repo holding this script)
#   -w DIR          Isaac ROS workspace (default: $ISAAC_ROS_WS)
#   -t DIR          deployed package folder (default: <workspace>/src/isaac_ros_common/braille_interface)

PKG_PATH=src/braille_interface    # where the package lives inside the repo

ACTION=deploy; REF=""; ROLLBACK_FILE=""; YES=0; FETCH=1; DRY=0; REPO=""; WS=""; TARGET=""
while [ $# -gt 0 ]; do
  case "$1" in
    -y|--yes) YES=1 ;;
    -n|--dry-run) DRY=1 ;;
    --no-fetch) FETCH=0 ;;
    --status) ACTION=status ;;
    --list) ACTION=list ;;
    --rollback) ACTION=rollback
      if [ $# -ge 2 ] && [ "${2#-}" = "$2" ]; then ROLLBACK_FILE="$2"; shift; fi ;;
    -r|-w|-t)
      [ $# -ge 2 ] || { echo "$1 needs a value" >&2; exit 2; }
      case "$1" in -r) REPO="$2" ;; -w) WS="$2" ;; -t) TARGET="$2" ;; esac
      shift ;;
    -h|--help) sed -n '2,21p' "$0" | cut -c3-; exit 0 ;;
    -*) echo "Unknown option: $1 (see --help)" >&2; exit 2 ;;
    *) [ -z "$REF" ] || { echo "Only one ref, please (got '$REF' and '$1')" >&2; exit 2; }
       REF="$1" ;;
  esac
  shift
done

die() { echo "Error: $*" >&2; exit 1; }
say() { printf '%s\n' "$*"; }

[ -f /.dockerenv ] && die "run this on the Jetson host, not inside the container (the container can't see the repo clone)."

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
[ -n "$REPO" ] || REPO="$(git -C "$SCRIPT_DIR" rev-parse --show-toplevel 2>/dev/null)"
[ -n "$REPO" ] && git -C "$REPO" rev-parse --git-dir >/dev/null 2>&1 || die "no repo clone found (use -r DIR)."
[ -n "$WS" ] || WS="${ISAAC_ROS_WS:-}"
WS="${WS%/}"
[ -n "$WS" ] && [ -d "$WS" ] || die "workspace not found: '$WS' (set ISAAC_ROS_WS or use -w DIR)."
[ -n "$TARGET" ] || TARGET="$WS/src/isaac_ros_common/braille_interface"
TARGET="${TARGET%/}"
[ -L "$TARGET" ] && die "$TARGET is a symlink; refusing to deploy into it."
BACKUPS="$WS/braille_deploy_backups"
HISTORY="$BACKUPS/deploy_history.log"
G() { git -C "$REPO" "$@"; }

TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

# export_commit COMMIT DIR: write the package as it is in COMMIT to DIR/braille_interface.
export_commit() {
  mkdir -p "$2" && G archive "$1" "$PKG_PATH" | tar -x -C "$2" && mv "$2/$PKG_PATH" "$2/braille_interface" \
    && rm -rf "$2/src"
}

# changes OLD NEW: one line per difference, "M|A|D relative/path" (A = only in NEW).
changes() {
  diff -rq -x __pycache__ -x DEPLOYED_VERSION "$1" "$2" 2>/dev/null | while IFS= read -r line; do
    local root kind rest dir name rel
    case "$line" in
      "Only in $1:"*|"Only in $1/"*) root="$1"; kind=D ;;
      "Only in $2:"*|"Only in $2/"*) root="$2"; kind=A ;;
      "Files "*" differ")
        rel="${line#"Files $1/"}"; echo "M ${rel%% and *}"; continue ;;
      *) continue ;;
    esac
    rest="${line#"Only in $root"}"; dir="${rest%%: *}"; name="${rest#*: }"
    rel="${dir#/}"; [ -n "$rel" ] && rel="$rel/"
    echo "$kind $rel$name"
  done
}

# needs_rebuild CHANGES: true if colcon must run again for the change to take effect.
needs_rebuild() {
  printf '%s\n' "$1" | grep -qE '^[MAD] (setup\.py|setup\.cfg|package\.xml|resource/)|^[AD] launch/[^/]*\.launch\.py$'
}

field() { sed -n "s/^$1=//p" "$TARGET/DEPLOYED_VERSION" 2>/dev/null | head -1; }

# describe_target: what is deployed, and whether it was edited by hand since.
describe_target() {
  if [ ! -f "$TARGET/DEPLOYED_VERSION" ]; then
    say "Deployed now:   unknown (no DEPLOYED_VERSION yet; it was placed by hand)"
    return
  fi
  say "Deployed now:   $(field ref) @ $(field commit | cut -c1-7)  ($(field subject))"
  say "                by $(field deployed_by) on $(field deployed_at)"
  local c; c="$(field commit)"
  if G cat-file -e "$c^{commit}" 2>/dev/null && export_commit "$c" "$TMP/recorded" 2>/dev/null; then
    local drift; drift="$(changes "$TMP/recorded/braille_interface" "$TARGET")"
    if [ -n "$drift" ]; then
      say "                EDITED BY HAND since then:"
      printf '%s\n' "$drift" | sed 's/^/                  /'
    else
      say "                no hand edits since"
    fi
    rm -rf "$TMP/recorded"
  else
    say "                (that commit isn't in this clone; can't check for hand edits)"
  fi
}

backup_target() {  # backup_target LABEL -> prints the backup path
  mkdir -p "$BACKUPS" || die "can't create $BACKUPS"
  local f; f="$BACKUPS/$(date +%Y%m%d_%H%M%S)_before_$1.tgz"
  while [ -e "$f" ]; do f="${f%.tgz}_again.tgz"; done
  tar -czf "$f" -C "$(dirname "$TARGET")" "$(basename "$TARGET")" || die "backup failed; nothing was changed."
  printf '%s\n' "$f"
}

confirm() {
  [ $YES = 1 ] && return 0
  [ -t 0 ] || die "not running in a terminal; rerun with -y to proceed without asking."
  printf '%s [y/N] ' "$1"; read -r answer
  case "$answer" in y|Y|yes|YES) return 0 ;; *) say "Nothing changed."; exit 0 ;; esac
}

lock() {
  command -v flock >/dev/null 2>&1 || return 0
  mkdir -p "$BACKUPS" && exec 9>"$BACKUPS/.lock" && flock -n 9 \
    || die "another deploy or rollback is running."
}

pipeline_note() {
  if pgrep -f 'braille_server_node|heightmap_generator_node|publish_map_node|image_to_braille_node' >/dev/null 2>&1; then
    say "Note: the braille pipeline is running. Changes take effect when you restart it."
  fi
}

rebuild_advice() {
  if needs_rebuild "$1"; then
    say "Rebuild needed (setup.py, package.xml, resource/ or the set of launch files changed). Inside the container:"
    say "  cd /workspaces/isaac_ros-dev && colcon build --symlink-install --packages-select braille_interface"
  else
    say "No rebuild needed: restart the pipeline to use the new code."
  fi
}

# ---------------------------------------------------------------- actions

case "$ACTION" in
  status)
    say "Package folder: $TARGET"
    describe_target
    [ -f "$HISTORY" ] && { say ""; say "Last deploys:"; tail -n 5 "$HISTORY" | sed 's/^/  /'; }
    exit 0 ;;

  list)
    say "History ($HISTORY):"
    if [ -f "$HISTORY" ]; then sed 's/^/  /' "$HISTORY"; else say "  (none yet)"; fi
    say ""; say "Backups (newest last):"
    if ls "$BACKUPS"/*.tgz >/dev/null 2>&1; then ls -1tr "$BACKUPS"/*.tgz | sed 's/^/  /'; else say "  (none yet)"; fi
    exit 0 ;;

  rollback)
    [ -d "$TARGET" ] || die "package folder not found: $TARGET"
    if [ -z "$ROLLBACK_FILE" ]; then
      ROLLBACK_FILE="$(ls -1t "$BACKUPS"/*.tgz 2>/dev/null | head -n 1)"
      [ -n "$ROLLBACK_FILE" ] || die "no backups in $BACKUPS"
    fi
    [ -f "$ROLLBACK_FILE" ] || die "backup not found: $ROLLBACK_FILE"
    mkdir -p "$TMP/restore" && tar -xzf "$ROLLBACK_FILE" -C "$TMP/restore" || die "can't read $ROLLBACK_FILE"
    RESTORED="$TMP/restore/$(basename "$TARGET")"
    [ -d "$RESTORED" ] || die "$ROLLBACK_FILE doesn't contain $(basename "$TARGET")/"
    describe_target
    say ""; say "Restore from:   $(basename "$ROLLBACK_FILE")"
    CH="$(changes "$TARGET" "$RESTORED")"
    if [ -z "$CH" ]; then say "The package folder already matches that backup. Nothing to do."; exit 0; fi
    say "Changes:"; printf '%s\n' "$CH" | sed 's/^/  /'
    [ $DRY = 1 ] && { say "(dry run: nothing changed)"; exit 0; }
    confirm "Restore this backup?"
    lock
    B="$(backup_target rollback)" || exit 1
    rsync -a --delete --exclude __pycache__ "$RESTORED/" "$TARGET/" || die "restore failed; the state before it is in $B"
    printf '%s\t%s\trollback\t%s\tbackup=%s\n' "$(date '+%Y-%m-%d %H:%M:%S')" "$(id -un)@$(hostname)" \
      "$(basename "$ROLLBACK_FILE")" "$(basename "$B")" >>"$HISTORY"
    say "Restored. The state before this rollback is saved in $(basename "$B")."
    pipeline_note
    rebuild_advice "$CH"
    exit 0 ;;
esac

# ---------------------------------------------------------------- deploy

[ -n "$REF" ] || die "say which branch, tag or commit to deploy (see --help)."
[ -d "$TARGET" ] || die "package folder not found: $TARGET (use -t DIR)"

if [ $FETCH = 1 ]; then
  G fetch --quiet origin 2>/dev/null || say "Warning: couldn't fetch from origin; using what this clone already has."
fi
RESOLVED="$REF"
if G rev-parse -q --verify "origin/$REF^{commit}" >/dev/null; then
  RESOLVED="origin/$REF"
  if G rev-parse -q --verify "refs/heads/$REF" >/dev/null && \
     [ "$(G rev-parse "refs/heads/$REF")" != "$(G rev-parse "origin/$REF")" ]; then
    say "Note: the local branch '$REF' differs from GitHub; deploying GitHub's version (origin/$REF)."
  fi
fi
COMMIT="$(G rev-parse -q --verify "$RESOLVED^{commit}")" || die "'$REF' isn't a branch, tag or commit in $REPO"
G cat-file -e "$COMMIT:$PKG_PATH/package.xml" 2>/dev/null || die "$RESOLVED has no $PKG_PATH/package.xml"
SUBJECT="$(G log -1 --format=%s "$COMMIT")"

export_commit "$COMMIT" "$TMP/new" || die "couldn't export $RESOLVED"
NEW="$TMP/new/braille_interface"

say "Package folder: $TARGET"
describe_target
say "Deploying:      $RESOLVED @ ${COMMIT:0:7}  ($SUBJECT)"
say ""
CH="$(changes "$TARGET" "$NEW")"
if [ -z "$CH" ]; then
  say "The package folder already matches $RESOLVED."
else
  say "Changes:"; printf '%s\n' "$CH" | sed 's/^/  /'
fi
say ""
rebuild_advice "$CH"
[ $DRY = 1 ] && { say "(dry run: nothing changed)"; exit 0; }
confirm "Deploy $RESOLVED?"

lock
B="$(backup_target deploy)" || exit 1
rsync -a --delete --exclude __pycache__ "$NEW/" "$TARGET/" || die "copy failed; restore the previous state with: $0 --rollback $B"
cat >"$TARGET/DEPLOYED_VERSION" <<EOF
ref=$RESOLVED
commit=$COMMIT
subject=$SUBJECT
deployed_at=$(date '+%Y-%m-%d %H:%M:%S %z')
deployed_by=$(id -un)@$(hostname)
repo=$REPO
backup=$B
EOF
printf '%s\t%s\tdeploy\t%s %s\tbackup=%s\n' "$(date '+%Y-%m-%d %H:%M:%S')" "$(id -un)@$(hostname)" \
  "$RESOLVED" "${COMMIT:0:7}" "$(basename "$B")" >>"$HISTORY"
say ""
say "Deployed $RESOLVED @ ${COMMIT:0:7}. Previous state saved in $(basename "$B")."
say "Undo with: $0 --rollback"
pipeline_note
