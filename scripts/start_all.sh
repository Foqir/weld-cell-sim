#!/usr/bin/env bash
# Usage: start_all.sh [hdr20_17|hh020|...] [--headless] [--no-rviz] [--demo] [--no-shell]
#                     [--offset DX DY DYAW]   (real part displacement in the fixture: m, m, deg)
# Starts Gazebo (welding cell) + HD Hyundai robot with torch + ros2_control + MoveIt 2 + RViz.
source /opt/scripts/env.sh

MODEL=hdr20_17
HEADLESS=false
RVIZ=true
DEMO=
OPEN_SHELL=1
DX=0.0; DY=0.0; DYAW=0.0
while [ $# -gt 0 ]; do
  case "$1" in
    --headless) HEADLESS=true ;;
    --no-rviz) RVIZ=false ;;
    --demo) DEMO=1 ;;
    --no-shell) OPEN_SHELL= ;;
    --offset) DX=$2; DY=$3; DYAW=$4; shift 3 ;;
    -h|--help) sed -n '2,4p' "$0"; exit 0 ;;
    *) MODEL="$1" ;;
  esac
  shift
done
export ROBOT_MODEL=$MODEL PART_DX=$DX PART_DY=$DY PART_DYAW=$DYAW

LOG_DIR=/root/shared/logs/sim_$(date +%Y%m%d_%H%M%S)
mkdir -p "$LOG_DIR"
PIDS=()

cleanup() {
  echo "Stopping simulation..."
  kill "${PIDS[@]}" 2>/dev/null
  pkill -f "gz sim" 2>/dev/null
  wait 2>/dev/null
}
trap cleanup EXIT
trap 'exit 130' INT TERM

wait_for() {  # wait_for <timeout_s> <description> <command...>
  local timeout=$1 what=$2
  shift 2
  for ((i = 0; i < timeout; i++)); do
    if "$@" >/dev/null 2>&1; then echo "  [ok] $what"; return 0; fi
    sleep 1
  done
  echo "  [FAIL] $what - see logs in $LOG_DIR"
  return 1
}

echo "Logs: $LOG_DIR"
echo "Starting welding cell: robot $MODEL, part offset ($DX m, $DY m, $DYAW deg)..."
ros2 launch weld_cell weld_cell.launch.py robot_model:="$MODEL" headless:=$HEADLESS rviz:=$RVIZ \
  part_dx:=$DX part_dy:=$DY part_dyaw:=$DYAW > "$LOG_DIR/launch.log" 2>&1 &
PIDS+=($!)

wait_for 120 "Gazebo running" bash -c "gz topic -l | grep -q /world/weld_cell/clock" || exit 1
wait_for 120 "controllers active" \
  bash -c "ros2 control list_controllers 2>/dev/null | grep joint_trajectory_controller | grep -q active" || exit 1
wait_for 120 "MoveIt ready" bash -c "ros2 service list | grep -q /compute_cartesian_path" || exit 1

cat <<EOF

=== Welding cell is running (robot $MODEL) ===
  Weld program : weld_demo.sh               (all seams, report in /root/shared/logs/weld_*)
                 weld_demo.sh -p seams:="['tube_ring']" -p speed_factor:=2.0
  Topics       : /weld/arc_on /weld/state /weld/markers /joint_states
  Controller   : ros2 control list_controllers
  Logs         : $LOG_DIR
Type 'exit' to stop everything.

EOF

if [ -n "$DEMO" ]; then
  weld_demo.sh
fi

if [ -n "$OPEN_SHELL" ]; then
  bash -i
else
  wait "${PIDS[0]}"
fi
