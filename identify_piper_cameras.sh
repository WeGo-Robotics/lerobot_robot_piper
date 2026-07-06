#!/bin/bash
set -euo pipefail

BASE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

WIDTH="${WIDTH:-640}"
HEIGHT="${HEIGHT:-480}"
FPS="${FPS:-30}"
FOURCC="${FOURCC:-}"
ROBOT_PORT="${ROBOT_PORT:-can_follower1}"
TELEOP_PORT="${TELEOP_PORT:-can_leader1}"
ROBOT_ID="${ROBOT_ID:-piper_follower}"
TELEOP_ID="${TELEOP_ID:-piper_leader}"
DATASET_REPO_ID="${DATASET_REPO_ID:-jis/piper_right_cam_test}"
DATASET_ROOT="${DATASET_ROOT:-/lerobot/data/piper_right_cam_test}"
DATASET_TASK="${DATASET_TASK:-teleoperate piper arm}"
DATASET_NUM_EPISODES="${DATASET_NUM_EPISODES:-10}"
DATASET_EPISODE_TIME_S="${DATASET_EPISODE_TIME_S:-30}"
DATASET_RESET_TIME_S="${DATASET_RESET_TIME_S:-5}"

# Default: identify only the right camera.
# top/left are intentionally kept disabled for now because only the right
# camera is currently connected.
#
# To identify all cameras later:
#   IDENTIFY_ROLES=top,right,left ./identify_piper_cameras.sh
IDENTIFY_ROLES="${IDENTIFY_ROLES:-right}"
IFS=',' read -r -a ROLES <<< "${IDENTIFY_ROLES}"

declare -A CAMERA_PATHS
declare -A CAMERA_INFO
declare -A CAMERA_FOURCC
declare -A CAMERA_WIDTH
declare -A CAMERA_HEIGHT
declare -A CAMERA_FPS

wait_key() {
  local prompt="$1"
  echo
  echo "${prompt}"
  echo "Press Enter or Space to continue."
  IFS= read -r -n 1 _ || true
  echo
}

list_camera_paths() {
  local paths=()

  if [ -d /dev/v4l/by-id ]; then
    mapfile -t paths < <(find /dev/v4l/by-id -maxdepth 1 -type l -print 2>/dev/null | sort)
    if [ "${#paths[@]}" -gt 0 ]; then
      printf '%s\n' "${paths[@]}"
      return 0
    fi
  fi

  if compgen -G "/dev/video*" >/dev/null; then
    compgen -G "/dev/video*" | sort
    return 0
  fi

  true
}

path_to_realdev() {
  local path="$1"
  readlink -f "${path}" 2>/dev/null || echo "${path}"
}

describe_camera() {
  local path="$1"
  local realdev
  realdev="$(path_to_realdev "${path}")"

  echo "path=${path}"
  echo "realdev=${realdev}"

  if command -v v4l2-ctl >/dev/null 2>&1; then
    v4l2-ctl --device="${realdev}" --info 2>/dev/null \
      | sed -n 's/^[[:space:]]*Card type[[:space:]]*:[[:space:]]*/card=/p; s/^[[:space:]]*Bus info[[:space:]]*:[[:space:]]*/bus=/p' \
      || true
  fi

  if command -v udevadm >/dev/null 2>&1; then
    udevadm info --query=property --name="${realdev}" 2>/dev/null \
      | awk -F= '/^(ID_MODEL=|ID_SERIAL=|ID_PATH=|ID_V4L_PRODUCT=)/ {print tolower($1) "=" $2}' \
      || true
  fi
}

capture_paths() {
  local output_var="$1"
  mapfile -t "${output_var}" < <(list_camera_paths)
}

print_paths() {
  local title="$1"
  shift
  local paths=("$@")

  echo "${title}"
  if [ "${#paths[@]}" -eq 0 ]; then
    echo "  (none)"
    return
  fi

  local p
  for p in "${paths[@]}"; do
    echo "  - ${p} -> $(path_to_realdev "${p}")"
  done
}

print_camera_debug() {
  echo
  echo "Camera debug:"
  echo "  /dev/video*:"
  ls -l /dev/video* 2>/dev/null | sed 's/^/    /' || echo "    (none)"
  echo "  /dev/v4l/by-id:"
  ls -l /dev/v4l/by-id 2>/dev/null | sed 's/^/    /' || echo "    (none)"
  if command -v lsusb >/dev/null 2>&1; then
    echo "  lsusb camera-like devices:"
    lsusb | grep -i -E "camera|video|intel|realsense|depth|webcam" | sed 's/^/    /' || echo "    (none)"
  else
    echo "  lsusb: not installed"
  fi
}

removed_paths() {
  local before_name="$1"
  local after_name="$2"
  local -n before_ref="${before_name}"
  local -n after_ref="${after_name}"

  local before after found
  for before in "${before_ref[@]}"; do
    found=0
    for after in "${after_ref[@]}"; do
      if [ "${before}" = "${after}" ]; then
        found=1
        break
      fi
    done
    if [ "${found}" -eq 0 ]; then
      echo "${before}"
    fi
  done
}

camera_group_key() {
  local path="$1"
  local base
  base="$(basename "${path}")"

  if [[ "${base}" == *-video-index* ]]; then
    echo "${base%-video-index*}"
  else
    echo "${base}"
  fi
}

camera_group_paths() {
  local path="$1"
  local key
  key="$(camera_group_key "${path}")"

  if [ -d /dev/v4l/by-id ]; then
    find /dev/v4l/by-id -maxdepth 1 -type l -name "${key}-video-index*" -print 2>/dev/null | sort
  else
    echo "${path}"
  fi
}

camera_group_realdevs() {
  local path="$1"
  local p

  while IFS= read -r p; do
    path_to_realdev "${p}"
  done < <(camera_group_paths "${path}") | sort -V | uniq
}

camera_format_summary() {
  local path="$1"
  local realdev
  realdev="$(path_to_realdev "${path}")"

  if command -v v4l2-ctl >/dev/null 2>&1; then
    v4l2-ctl --device="${realdev}" --get-fmt-video 2>/dev/null \
      | awk -F: '/Width\/Height|Pixel Format/ {gsub(/^[ \t]+/, "", $2); printf "%s ", $2}' \
      || true
  fi
}

camera_fourcc() {
  local path="$1"
  local realdev
  realdev="$(path_to_realdev "${path}")"

  if command -v v4l2-ctl >/dev/null 2>&1; then
    v4l2-ctl --device="${realdev}" --get-fmt-video 2>/dev/null \
      | sed -n "s/.*Pixel Format.*'\([^']*\)'.*/\1/p" \
      | head -n 1
  fi
}

lerobot_opencv_camera_lines() {
  if ! command -v python >/dev/null 2>&1; then
    return 1
  fi

  python - <<'PY'
try:
    from lerobot.cameras.opencv import OpenCVCamera
except Exception as exc:
    raise SystemExit(f"lerobot import failed: {exc}")

for cam in OpenCVCamera.find_cameras():
    profile = cam.get("default_stream_profile", {})
    print(
        "|".join(
            [
                str(cam.get("id", "")),
                str(profile.get("fourcc", "")),
                str(profile.get("width", "")),
                str(profile.get("height", "")),
                str(profile.get("fps", "")),
            ]
        )
    )
PY
}

choose_lerobot_camera_path() {
  local role="$1"
  shift
  local candidates=("$@")

  if [ "${#candidates[@]}" -eq 0 ]; then
    echo "[ERROR] No video nodes found for '${role}' after replug." >&2
    exit 1
  fi

  echo >&2
  echo "Using LeRobot OpenCVCamera.find_cameras() to select '${role}' camera node..." >&2

  local lines=()
  if ! mapfile -t lines < <(lerobot_opencv_camera_lines); then
    echo "[ERROR] Could not run LeRobot camera detection in this shell." >&2
    echo "        Run this script inside the LeRobot Docker container, or install LeRobot in this environment." >&2
    exit 1
  fi

  if [ "${#lines[@]}" -eq 0 ]; then
    echo "[ERROR] LeRobot did not detect any OpenCV camera." >&2
    echo "        Try inside Docker: lerobot-find-cameras opencv --output-dir /lerobot/data/captured_images" >&2
    exit 1
  fi

  echo "LeRobot detected OpenCV cameras:" >&2
  local line id fourcc width height fps path candidate
  local working=()
  local working_meta=()
  for line in "${lines[@]}"; do
    IFS='|' read -r id fourcc width height fps <<< "${line}"
    echo "  - ${id} fourcc=${fourcc} ${width}x${height}@${fps}" >&2
    for candidate in "${candidates[@]}"; do
      path="$(path_to_realdev "${candidate}")"
      if [ "${id}" = "${path}" ]; then
        working+=("${id}")
        working_meta+=("${fourcc}|${width}|${height}|${fps}")
      fi
    done
  done

  if [ "${#working[@]}" -eq 1 ]; then
    IFS='|' read -r fourcc width height fps <<< "${working_meta[0]}"
    CAMERA_FOURCC["${role}"]="${fourcc}"
    CAMERA_WIDTH["${role}"]="${width}"
    CAMERA_HEIGHT["${role}"]="${height}"
    CAMERA_FPS["${role}"]="${fps%.*}"
    echo "${working[0]}"
    return
  fi

  if [ "${#working[@]}" -gt 1 ]; then
    echo >&2
    echo "Multiple LeRobot OpenCV nodes match '${role}'." >&2
    local i
    for i in "${!working[@]}"; do
      IFS='|' read -r fourcc width height fps <<< "${working_meta[$i]}"
      echo "  [$((i + 1))] ${working[$i]} fourcc=${fourcc} ${width}x${height}@${fps}" >&2
    done

    local choice
    while true; do
      read -r -p "Select the LeRobot camera node for ${role} [1-${#working[@]}]: " choice
      if [[ "${choice}" =~ ^[0-9]+$ ]] && [ "${choice}" -ge 1 ] && [ "${choice}" -le "${#working[@]}" ]; then
        IFS='|' read -r fourcc width height fps <<< "${working_meta[$((choice - 1))]}"
        CAMERA_FOURCC["${role}"]="${fourcc}"
        CAMERA_WIDTH["${role}"]="${width}"
        CAMERA_HEIGHT["${role}"]="${height}"
        CAMERA_FPS["${role}"]="${fps%.*}"
        echo "${working[$((choice - 1))]}"
        return
      fi
    done
  fi

  echo "[ERROR] LeRobot detected cameras, but none match the replugged ${role} camera nodes." >&2
  echo "        Candidate nodes from this physical camera:" >&2
  local i
  for i in "${!candidates[@]}"; do
    echo "        - $(path_to_realdev "${candidates[$i]}")" >&2
  done
  exit 1
}

wait_until_path_returns() {
  local path="$1"
  local role="$2"
  local timeout_s="${CAMERA_RETURN_TIMEOUT_S:-30}"
  local deadline=$((SECONDS + timeout_s))

  while [ "${SECONDS}" -lt "${deadline}" ]; do
    if [ -e "${path}" ]; then
      echo "[OK] ${role} camera is back: ${path}"
      return 0
    fi
    sleep 1
  done

  echo "[WARN] ${role} camera path did not return within ${timeout_s}s: ${path}"
  echo "       Continuing anyway; reconnect or rerun this script if the generated command fails."
}

choose_removed_path() {
  local role="$1"
  shift
  local candidates=("$@")

  if [ "${#candidates[@]}" -eq 0 ]; then
    echo "[ERROR] No removed camera was detected for '${role}'." >&2
    echo "        Make sure exactly the ${role} camera was unplugged, then rerun this script." >&2
    exit 1
  fi

  if [ "${#candidates[@]}" -eq 1 ]; then
    echo "${candidates[0]}"
    return
  fi

  echo >&2
  echo "Multiple removed video nodes were detected for '${role}'." >&2
  echo "Treating them as one physical camera and auto-selecting the stream node after replug." >&2
  local i
  for i in "${!candidates[@]}"; do
    echo "  - ${candidates[$i]} -> $(path_to_realdev "${candidates[$i]}")" >&2
  done

  echo "${candidates[0]}"
}

camera_config_for_right() {
  local top_camera="${CAMERA_PATHS[top]:-}"
  local right_camera="${CAMERA_PATHS[right]:-}"
  local left_camera="${CAMERA_PATHS[left]:-}"
  local right_fourcc="${CAMERA_FOURCC[right]:-${FOURCC}}"
  local right_width="${CAMERA_WIDTH[right]:-${WIDTH}}"
  local right_height="${CAMERA_HEIGHT[right]:-${HEIGHT}}"
  local right_fps="${CAMERA_FPS[right]:-${FPS}}"

  if [ -z "${right_camera}" ]; then
    echo "[ERROR] right camera is not identified."
    exit 1
  fi

  if [ -n "${right_fourcc}" ]; then
    echo "{right: {type: opencv, index_or_path: '${right_camera}', fps: ${right_fps}, width: ${right_width}, height: ${right_height}, fourcc: ${right_fourcc}}}"
  else
    echo "{right: {type: opencv, index_or_path: '${right_camera}', fps: ${right_fps}, width: ${right_width}, height: ${right_height}}}"
  fi

  # top/left are intentionally not included by default.
  # After identifying them with:
  #   IDENTIFY_ROLES=top,right,left ./identify_piper_cameras.sh
  # add entries like these to camera_config if needed:
  #   top: {type: opencv, index_or_path: '${top_camera}', fps: ${FPS}, width: ${WIDTH}, height: ${HEIGHT}}
  #   left: {type: opencv, index_or_path: '${left_camera}', fps: ${FPS}, width: ${WIDTH}, height: ${HEIGHT}}
}

print_teleop_command() {
  local camera_config
  camera_config="$(camera_config_for_right)"

  cat <<EOF
lerobot-teleoperate \\
  --robot.type=piper_follower \\
  --robot.port="${ROBOT_PORT}" \\
  --robot.id="${ROBOT_ID}" \\
  --robot.cameras="${camera_config}" \\
  --teleop.type=piper_leader \\
  --teleop.port="${TELEOP_PORT}" \\
  --teleop.id="${TELEOP_ID}" \\
  --display_data=true
EOF
}

print_record_command() {
  local camera_config
  camera_config="$(camera_config_for_right)"

  cat <<EOF
lerobot-record \\
  --robot.type=piper_follower \\
  --robot.port="${ROBOT_PORT}" \\
  --robot.id="${ROBOT_ID}" \\
  --robot.cameras="${camera_config}" \\
  --teleop.type=piper_leader \\
  --teleop.port="${TELEOP_PORT}" \\
  --teleop.id="${TELEOP_ID}" \\
  --dataset.repo_id="${DATASET_REPO_ID}" \\
  --dataset.root="${DATASET_ROOT}" \\
  --dataset.single_task="${DATASET_TASK}" \\
  --dataset.fps=${FPS} \\
  --dataset.num_episodes=${DATASET_NUM_EPISODES} \\
  --dataset.episode_time_s=${DATASET_EPISODE_TIME_S} \\
  --dataset.reset_time_s=${DATASET_RESET_TIME_S} \\
  --dataset.push_to_hub=false \\
  --display_data=true
EOF
}

echo "Piper camera role identifier"
echo
echo "This script identifies USB cameras by unplugging one camera at a time."
echo "Active identify roles: ${IDENTIFY_ROLES}"
echo
echo "Default is right only."
echo "top/left are implemented but disabled for now."
echo "To identify all cameras later:"
echo "  IDENTIFY_ROLES=top,right,left ./identify_piper_cameras.sh"
echo
echo "Teleop defaults:"
echo "  robot port : ${ROBOT_PORT}"
echo "  teleop port: ${TELEOP_PORT}"
echo "  camera     : ${WIDTH}x${HEIGHT}@${FPS}"
echo "  record repo: ${DATASET_REPO_ID}"
echo "  record root: ${DATASET_ROOT}"
if [ -n "${FOURCC}" ]; then
  echo "  fourcc     : ${FOURCC}"
fi

capture_paths BASELINE
print_paths "Currently detected camera video nodes:" "${BASELINE[@]}"
if [ "${#BASELINE[@]}" -eq 0 ]; then
  print_camera_debug
fi

if [ "${#BASELINE[@]}" -lt "${#ROLES[@]}" ]; then
  echo
  echo "[WARN] Fewer camera video nodes were detected than active roles."
  echo "       If this is running on the host, make sure all cameras are plugged in."
  echo "       If this is running in Docker, make sure /dev is mounted into the container."
fi

for role in "${ROLES[@]}"; do
  role="$(echo "${role}" | tr -d '[:space:]')"
  if [ -z "${role}" ]; then
    continue
  fi

  echo
  echo "============================================================"
  echo "Identify '${role}' camera"
  echo "============================================================"

  capture_paths BEFORE
  print_paths "Before unplugging ${role}:" "${BEFORE[@]}"

  wait_key "Unplug ONLY the '${role}' camera now."

  sleep 1
  capture_paths AFTER
  print_paths "After unplugging ${role}:" "${AFTER[@]}"

  mapfile -t REMOVED < <(removed_paths BEFORE AFTER)
  removed="$(choose_removed_path "${role}" "${REMOVED[@]}")"

  echo
  echo "[OK] ${role} physical camera identified:"
  describe_camera "${removed}" | sed 's/^/  /'

  wait_key "Plug the '${role}' camera back in now."
  wait_until_path_returns "${removed}" "${role}"

  mapfile -t GROUP_CANDIDATES < <(camera_group_realdevs "${removed}")
  selected="$(choose_lerobot_camera_path "${role}" "${GROUP_CANDIDATES[@]}")"
  real_selected="$(path_to_realdev "${selected}")"
  selected_fourcc="${CAMERA_FOURCC[${role}]:-$(camera_fourcc "${selected}")}"
  CAMERA_PATHS["${role}"]="${real_selected}"
  CAMERA_FOURCC["${role}"]="${selected_fourcc}"
  CAMERA_INFO["${role}"]="$(describe_camera "${selected}")"

  echo
  echo "[OK] ${role} stream node selected:"
  echo "${CAMERA_INFO[${role}]}" | sed 's/^/  /'
  echo "  selected_for_lerobot=${real_selected}"
  if [ -n "${selected_fourcc}" ]; then
    echo "  selected_fourcc=${selected_fourcc}"
  fi
done

echo
echo "============================================================"
echo "Final camera mapping"
echo "============================================================"
for role in "${ROLES[@]}"; do
  echo "${role}: ${CAMERA_PATHS[${role}]} -> $(path_to_realdev "${CAMERA_PATHS[${role}]}")"
done

echo
echo "[Done] Copy and run these inside the LeRobot Docker container:"
echo
echo "# Teleoperate"
print_teleop_command
echo
echo "# Record dataset"
print_record_command
