#!/usr/bin/env bash
# ROS 2 S5: simultaneous complete loss of motors 4 and 8, horizontal hover.
# Run: bash /ros_ws/bash_Scripts/run_S5.sh /ros_ws
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
/usr/bin/python3 - "$workspace" <<'PY_S5'
from pathlib import Path
import ast
import shutil
import sys
import yaml
root = Path(sys.argv[1])
path = root / 'src/firmware/scripts/run_failure_scenario.py'
source = path.read_text()
updated = source
if '# S5_DOUBLE_FAILURE_V1' not in source:
    old_choices = "choices=['S1', 'S2', 'S3', 'S4']"
    old_map = "motor = {'S1': 4, 'S2': 1, 'S3': 6, 'S4': 6}[args.scenario]"
    old_failure = """        lower, upper = original[:8], original[8:]
        lower[motor-1] = upper[motor-1] = 0.
        node.event('failure_requested', motor=motor, rotor_index=motor-1, upper_N=0.)
        node.call(SetMotorLimits, '/failure_controller/set_motor_limits', lower=lower, upper=upper)
        node.wait(lambda: list(node.data['limits'].data) == lower+upper, 5, 'failure bounds confirmation')
        node.event('failure_confirmed', motor=motor)
"""
    for anchor in (old_choices, old_map, old_failure):
        if anchor not in updated:
            raise SystemExit('Expected the shared S1-S4 runner; no edits made. Run the corrected S4 installer first.')
    if '# S4_MIDTRAJECTORY_V2' not in source and '# S4_MIDTRAJECTORY_V3' not in source:
        raise SystemExit('Install the corrected S4 timing version first; no edits made.')
    updated = updated.replace(old_choices, "choices=['S1', 'S2', 'S3', 'S4', 'S5']")
    updated = updated.replace(old_map,
        "motor = {'S1': 4, 'S2': 1, 'S3': 6, 'S4': 6, 'S5': 4}[args.scenario]\n"
        "    # S5_DOUBLE_FAILURE_V1: both bounds change in one service request.\n"
        "    failed_motors = [4, 8] if args.scenario == 'S5' else [motor]")
    updated = updated.replace("scenario=args.scenario, motor_number=motor, rotor_index=motor-1, effectiveness=0.,",
        "scenario=args.scenario, motor_number=None if args.scenario == 'S5' else motor,\n"
        "            rotor_index=None if args.scenario == 'S5' else motor-1,\n"
        "            failed_motor_numbers=failed_motors, failed_rotor_indices=[m-1 for m in failed_motors],\n"
        "            effectiveness=0.,")
    updated = updated.replace("args.scenario + '_motor' + str(motor)",
        "args.scenario + ('_motors4_8' if args.scenario == 'S5' else '_motor' + str(motor))")
    updated = updated.replace("8. if args.scenario == 'S4' else 7.",
        "8. if args.scenario in ('S4', 'S5') else 7.")
    new_failure = """        lower, upper = original[:8], original[8:]
        for failed_motor in failed_motors:
            lower[failed_motor-1] = upper[failed_motor-1] = 0.
        node.event('failure_requested', motor=motor if len(failed_motors) == 1 else None,
                   motors=failed_motors, rotor_indices=[m-1 for m in failed_motors], upper_N=0.)
        node.call(SetMotorLimits, '/failure_controller/set_motor_limits', lower=lower, upper=upper)
        node.wait(lambda: list(node.data['limits'].data) == lower+upper, 5, 'failure bounds confirmation')
        node.event('failure_confirmed', motor=motor if len(failed_motors) == 1 else None,
                   motors=failed_motors)
"""
    updated = updated.replace(old_failure, new_failure)
    ast.parse(updated)
config = yaml.safe_load((root / 'src/controller/config/failure_aware.yaml').read_text())
params = config['failure_controller']['ros__parameters']
if float(params['allocation_moment_weight']) != 1000.:
    raise SystemExit('Expected the successful moment weight 1000. No settings changed.')
# Check this pair against the actual allocation model before accepting the test.
import numpy as np
A = np.array(params['allocation'], dtype=float).reshape(6, 8)
healthy = [i for i in range(8) if i not in (3, 7)]
wrench = np.array([0., 0., params['mass']*params.get('gravity', 9.81), 0., 0., 0.])
try:
    hover = np.linalg.solve(A[:, healthy], wrench)
except np.linalg.LinAlgError:
    raise SystemExit('The six remaining columns are singular for motors 4/8 failure.')
lo = np.array(params['thrust_min'])[healthy]
hi = np.array(params['thrust_max'])[healthy]
margin = np.minimum(params['actuator_reserve'], .2*(hi-lo))
if np.any(hover < lo+margin-1e-6) or np.any(hover > hi-margin+1e-6):
    raise SystemExit('Motors 4/8 do not preserve horizontal hover with the current planning reserve.')
if updated != source:
    backup = path.with_name('run_failure_scenario.before_S5.py')
    if not backup.exists():
        shutil.copy2(path, backup)
    path.write_text(updated)
    print('S5 installed; saved original runner:', backup)
else:
    print('S5 support already installed.')
print('Verified horizontal hover feasibility after simultaneous motors 4/8 failure.')
print('Controller settings unchanged; no video recording.')
PY_S5
(
  cd -- "$workspace"
  colcon build --packages-select firmware controller --cmake-args -DCMAKE_BUILD_TYPE=Release
)
set +u
source "$workspace/install/setup.bash"
set -u
run_root=${OMNI_RESULTS_DIR:-"$workspace/simulation_results"}
mkdir -p -- "$run_root"
run_dir=$(mktemp -d "$run_root/S5_motors4_8_$(date +%Y%m%d_%H%M%S)_XXXXXX")
cp -- "${BASH_SOURCE[0]}" "$run_dir/run_S5_used.sh"
exec ros2 run firmware run_failure_scenario.py S5 --output "$run_dir"
