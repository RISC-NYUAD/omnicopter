#!/usr/bin/env bash
# ROS 2 Jazzy/Harmonic S3: complete motor 6 failure (rotor_5), no video.
# Run inside the container: bash /ros_ws/bash_Scripts/run_S3.sh /ros_ws
set -Eeuo pipefail
export PATH="/usr/bin:/bin:$PATH"
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
workspace=${1:-"$script_dir/.."}
workspace=$(cd -- "$workspace" && pwd)
set +u
source /opt/ros/jazzy/setup.bash
[[ -f "$workspace/install/setup.bash" ]] || { echo "Build the workspace first." >&2; exit 1; }
source "$workspace/install/setup.bash"
set -u

# Add S3 to the common runner while preserving S1/S2 and previous fixes.
# The source backup is created only once; this patch is idempotent.
/usr/bin/python3 - "$workspace" <<'PY'
from pathlib import Path
import ast
import shutil
import sys
import yaml
root = Path(sys.argv[1])
path = root / 'src/firmware/scripts/run_failure_scenario.py'
source = path.read_text()
updated = source.replace("choices=['S1', 'S2']", "choices=['S1', 'S2', 'S3']")
updated = updated.replace("motor = 4 if args.scenario == 'S1' else 1",
                          "motor = {'S1': 4, 'S2': 1, 'S3': 6}[args.scenario]")
if "choices=['S1', 'S2', 'S3']" not in updated or "'S3': 6" not in updated:
    raise SystemExit('Unrecognized scenario runner; no changes made. Share the current runner for adaptation.')
ast.parse(updated)
config = yaml.safe_load((root / 'src/controller/config/failure_aware.yaml').read_text())
params = config['failure_controller']['ros__parameters']
if float(params['allocation_moment_weight']) != 1000.0:
    raise SystemExit('Keep the successful S2 setting first: allocation_moment_weight must be 1000.0. No gains changed.')
if updated != source:
    backup = path.with_name('run_failure_scenario.before_S3.py')
    if not backup.exists():
        shutil.copy2(path, backup)
    path.write_text(updated)
    print('Added S3 to the shared runner; saved original source:', backup)
else:
    print('Shared runner already supports S3.')
print('Retaining current controller settings: force weight=%s, moment weight=%s, reserve=%s, gamma=%s' %
      (params['allocation_force_weight'], params['allocation_moment_weight'],
       params['actuator_reserve'], params['gamma']))
PY

# Install the updated runner and the source configuration; no controller changes.
(
  cd -- "$workspace"
  colcon build --packages-select firmware controller --cmake-args -DCMAKE_BUILD_TYPE=Release
)
set +u
source "$workspace/install/setup.bash"
set -u
run_root=${OMNI_RESULTS_DIR:-"$workspace/simulation_results"}
mkdir -p -- "$run_root"
run_dir=$(mktemp -d "$run_root/S3_motor6_$(date +%Y%m%d_%H%M%S)_XXXXXX")
cp -- "${BASH_SOURCE[0]}" "$run_dir/run_S3_used.sh"
exec ros2 run firmware run_failure_scenario.py S3 --output "$run_dir"
