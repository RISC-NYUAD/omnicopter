#!/usr/bin/env bash
# ROS 2 Jazzy + Gazebo Harmonic. Data recording only.
set -Eeuo pipefail
export PATH="/usr/bin:/bin:$PATH"
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
workspace=${1:-"$script_dir/.."}
workspace=$(cd -- "$workspace" && pwd)
set +u
source /opt/ros/jazzy/setup.bash
[[ -f "$workspace/install/setup.bash" ]] || { echo "Build first: cd \"$workspace\" && colcon build"; exit 1; }
source "$workspace/install/setup.bash"
set -u
run_root=${OMNI_RESULTS_DIR:-"$workspace/simulation_results"}
mkdir -p -- "$run_root"
run_dir=$(mktemp -d "$run_root/S2_motor1_$(date +%Y%m%d_%H%M%S)_XXXXXX")
cp -- "${BASH_SOURCE[0]}" "$run_dir/run_S2_used.sh"
exec ros2 run firmware run_failure_scenario.py S2 --output "$run_dir"
