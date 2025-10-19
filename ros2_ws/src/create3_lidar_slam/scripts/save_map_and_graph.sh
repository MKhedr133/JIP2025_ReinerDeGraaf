#!/usr/bin/env bash
set -euo pipefail

# Save occupancy map (.pgm/.yaml) and/or SLAM pose-graph (.data/.posegraph)
# Defaults:
#   - output dir: <repo>/create3_lidar_slam/config/maps
#   - base name : create3_home_maps
# Usage examples:
#   scripts/save_map_and_graph.sh
#   scripts/save_map_and_graph.sh -d /work/ros2_ws/src/create3_lidar_slam/config/maps -n my_lab_map
#   scripts/save_map_and_graph.sh -m             # save map only
#   scripts/save_map_and_graph.sh -g             # save graph only

usage() {
  cat <<'USAGE'
Usage: save_map_and_graph.sh [-d OUTPUT_DIR] [-n NAME] [-m] [-g] [-h]
  -d  Output directory (will be created if missing)
  -n  Base filename (no extension), e.g., "create3_home_maps"
  -m  Save occupancy map only (.pgm/.yaml)
  -g  Save pose-graph only (.data/.posegraph)
  -h  Show this help

Notes:
- Requires a running slam_toolbox node in "mapping" mode to serialize pose-graph.
- Requires /map topic to be available to save occupancy grid.
USAGE
}

# Default output directory: package repo's maps folder
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_DIR="$(cd "${SCRIPT_DIR}/.."; pwd)/config/maps"
OUT_DIR="${DEFAULT_DIR}"
NAME="create3_home_maps"
SAVE_MAP=1
SAVE_GRAPH=1

while getopts ":d:n:mgh" opt; do
  case "${opt}" in
    d) OUT_DIR="${OPTARG}" ;;
    n) NAME="${OPTARG}" ;;
    m) SAVE_GRAPH=0 ;;  # map only
    g) SAVE_MAP=0  ;;   # graph only
    h) usage; exit 0 ;;
    \?) echo "Invalid option: -${OPTARG}" >&2; usage; exit 2 ;;
  esac
done

mkdir -p "${OUT_DIR}"
OUT_BASE="${OUT_DIR%/}/${NAME}"

# Wait helpers (simple, friendly)
wait_for_topic() {
  local topic="$1" timeout="${2:-30}" t=0
  echo "[*] Waiting for topic ${topic} (timeout ${timeout}s)..."
  while ! ros2 topic list | grep -qx "${topic}"; do
    sleep 1; t=$((t+1))
    if [ "${t}" -ge "${timeout}" ]; then
      echo "[!] Timeout waiting for topic ${topic}" >&2; return 1
    fi
  done
  echo "[✓] Found topic ${topic}"
}

wait_for_service() {
  local service="$1" timeout="${2:-30}" t=0
  echo "[*] Waiting for service ${service} (timeout ${timeout}s)..."
  while ! ros2 service list | grep -qx "${service}"; do
    sleep 1; t=$((t+1))
    if [ "${t}" -ge "${timeout}" ]; then
      echo "[!] Timeout waiting for service ${service}" >&2; return 1
    fi
  done
  echo "[✓] Found service ${service}"
}

# Save occupancy grid (.pgm/.yaml)
if [ "${SAVE_MAP}" -eq 1 ]; then
  wait_for_topic "/map" 30
  echo "[*] Saving occupancy map to ${OUT_BASE}.{pgm,yaml} ..."
  ros2 run nav2_map_server map_saver_cli -f "${OUT_BASE}"
  echo "[✓] Map saved."
fi

# Save pose-graph (.data/.posegraph)
if [ "${SAVE_GRAPH}" -eq 1 ]; then
  wait_for_service "/slam_toolbox/serialize_map" 30
  echo "[*] Serializing SLAM pose-graph to ${OUT_BASE}.data/.posegraph ..."
  ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '${OUT_BASE}'}"
  echo "[✓] Pose-graph saved."
fi

echo "[✓] Done. Outputs under: ${OUT_DIR}"
