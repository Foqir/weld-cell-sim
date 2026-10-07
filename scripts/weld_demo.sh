#!/usr/bin/env bash
# Runs the weld program (all seams of weld_program.yaml) in the running cell.
# Extra arguments go to the node, e.g.: weld_demo.sh -p seams:="['rib_left']" -p speed_factor:=2.0 -p cycles:=3
source /opt/scripts/env.sh
ros2 run weld_cell weld_program.py --ros-args -p use_sim_time:=true \
  -p part_dx:=${PART_DX:-0.0} -p part_dy:=${PART_DY:-0.0} -p part_dyaw:=${PART_DYAW:-0.0} "$@"
