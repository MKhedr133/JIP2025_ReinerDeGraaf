#!/usr/bin/env bash
set -euo pipefail

# Save occupancy map (.pgm/.yaml) and/or SLAM pose-graph (.data/.posegraph)
# Defaults:
#   - name      : create3_home_map
#   - dest1     : <repo or install>/create3_lidar_slam/config/maps
#   - dest2     : <repo or install>/create3_localization_bringup/config/maps
# Notes:
#   - Works whether you run from source (git repo) or from an installed ROS workspace.
#   - Requires slam_toolbox running (for serialize_map) if you save the graph.

NAME="create3_home_map"
SAVE_MAP=1
SAVE_GRAPH=1
TIMEOUT=30

usage() {
cat <<'USAGE'
Usage: save_map_and_graph [-n NAME] [--map-only|--graph-only]
Examples:
  ros2 run create3_lidar_slam save_map_and_graph
  ros2 run create3_lidar_slam save_map_and_graph -n my_lab_map
  ros2 run create3_lidar_slam save_map_and_graph --map-only
USAGE
}

# Parse args
while [[ $# -gt 0 ]]; do
  case "$1" in
    -n|--name) NAME="$2"; shift 2;;
    --map-only) SAVE_MAP=1; SAVE_GRAPH=0; shift;;
    --graph-only) SAVE_MAP=0; SAVE_GRAPH=1; shift;;
    -h|--help) usage; exit 0;;
    *) echo "Unknown arg: $1"; usage; exit 1;;
  esac
done

# Resolve destination directories (repo-first, fallback to install/share)
find_repo_or_share_dir() {
  local pkg="$1" subdir="$2"
  local top=""
  if top=$(git rev-parse --show-toplevel 2>/dev/null); then
    if [[ -d "$top/$pkg/$subdir" ]]; then
      echo "$top/$pkg/$subdir"
      return 0
    fi
  fi
  # Fallback to installed share
  local prefix
  prefix=$(ros2 pkg prefix "$pkg")
  if [[ -d "$prefix/share/$pkg/$subdir" ]]; then
    echo "$prefix/share/$pkg/$subdir"
    return 0
  fi
  return 1
}

DEST1=$(find_repo_or_share_dir "create3_lidar_slam" "config/maps") || {
  echo "ERROR: Could not locate create3_lidar_slam/config/maps (repo or install)."; exit 1; }
DEST2=$(find_repo_or_share_dir "create3_localization_bringup" "config/maps") || {
  echo "ERROR: Could not locate create3_localization_bringup/config/maps (repo or install)."; exit 1; }

mkdir -p "$DEST1" "$DEST2"

OUT_BASE_1="${DEST1}/${NAME}"
OUT_BASE_2="${DEST2}/${NAME}"

wait_for_service() {
  local srv="$1" secs="$2"
  echo "[*] Waiting up to ${secs}s for service: ${srv} ..."
  local t=0
  while ! ros2 service list | grep -q --line-regexp "$srv"; do
    sleep 1; t=$((t+1))
    if (( t >= secs )); then
      echo "ERROR: Service ${srv} not available after ${secs}s."
      return 1
    fi
  done
}

# Save occupancy map (.pgm/.yaml)
if (( SAVE_MAP == 1 )); then
  echo "[*] Saving occupancy map to ${OUT_BASE_1}.{pgm,yaml} ..."
  ros2 run nav2_map_server map_saver_cli -f "${OUT_BASE_1}"
  echo "[*] Copying map to ${DEST2} ..."
  cp -f "${OUT_BASE_1}.pgm" "${OUT_BASE_1}.yaml" "${DEST2}/"
  echo "[✓] Map saved & copied."
fi

# Save pose-graph (.data/.posegraph) via slam_toolbox
if (( SAVE_GRAPH == 1 )); then
  wait_for_service "/slam_toolbox/serialize_map" "${TIMEOUT}"
  echo "[*] Serializing SLAM pose-graph to ${OUT_BASE_1}.data/.posegraph ..."
  ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '${OUT_BASE_1}'}"
  echo "[*] Copying graph to ${DEST2} ..."
  for ext in data posegraph; do
    if [[ -f "${OUT_BASE_1}.${ext}" ]]; then cp -f "${OUT_BASE_1}.${ext}" "${DEST2}/"; fi
  done
  echo "[✓] Pose-graph saved & copied."
fi

echo "[✓] Done."
echo "  create3_lidar_slam:           ${DEST1}"
echo "  create3_localization_bringup: ${DEST2}"
