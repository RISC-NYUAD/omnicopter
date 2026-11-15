#!/usr/bin/env bash
# Target: +2 m x, +1 m y, altitude 1.7 m; duration 20 s; failure at 10 s.
# ROS 2 Jazzy/Harmonic S4: complete motor 6 failure during a smooth 6D maneuver; no video.
# Run inside the container: bash /ros_ws/bash_Scripts/run_S4.sh /ros_ws
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

# Extend the common runner, preserving previous fixes and current gains.
/usr/bin/python3 - "$workspace" <<'PY'
from pathlib import Path
import ast
import re
import shutil
import sys
import yaml
root = Path(sys.argv[1])
path = root / 'src/firmware/scripts/run_failure_scenario.py'
source = path.read_text()
updated = source
if not any(marker in source for marker in ('# S4_SMOOTH_6D_V1', '# S4_MIDTRAJECTORY_V2', '# S4_MIDTRAJECTORY_V3')):
    updated, count = re.subn(r"choices=\['S1', 'S2'(?:, 'S3')?\]",
                            "choices=['S1', 'S2', 'S3', 'S4']", updated)
    if count != 1:
        raise SystemExit('Unrecognized scenario list; no changes made.')
    old_motor = "motor = 4 if args.scenario == 'S1' else 1"
    old_map = "motor = {'S1': 4, 'S2': 1, 'S3': 6}[args.scenario]"
    new_map = "motor = {'S1': 4, 'S2': 1, 'S3': 6, 'S4': 6}[args.scenario]"
    if old_motor in updated:
        updated = updated.replace(old_motor, new_map)
    elif old_map in updated:
        updated = updated.replace(old_map, new_map)
    else:
        raise SystemExit('Unrecognized motor mapping; no changes made.')
    old_import = 'from maneuver.srv import ArmDisarm, LiftOff, Land'
    if old_import not in updated:
        raise SystemExit('Unrecognized maneuver imports; no changes made.')
    updated = updated.replace(old_import, old_import + ', Goto6DPoint')
    anchor = '        node.check_hover()\n'
    if updated.count(anchor) != 1:
        raise SystemExit('Cannot locate pre-failure hover check; no changes made.')
    maneuver = """        # S4_SMOOTH_6D_V1: keep requesting the same trajectory after failure.
        if args.scenario == 'S4':
            start = node.data['reference'].pose.position
            target = dict(x=float(start.x)+2.0, y=float(start.y)+1.0, z=1.7,
                          roll=math.radians(10.), pitch=math.radians(-10.),
                          yaw=math.radians(20.), duration=20.)
            metadata_path = folder / 'scenario.json'
            metadata = json.loads(metadata_path.read_text())
            metadata.update(maneuver_service='/goto_6d_point', maneuver_target=target,
                            failure_after_maneuver_start_seconds=8.,
                            intended_settled_observation_after_maneuver_seconds=18.)
            metadata_path.write_text(json.dumps(metadata, indent=2)+'\\n')
            node.event('maneuver_requested', **target)
            node.call(Goto6DPoint, '/goto_6d_point', **target)
            node.event('maneuver_accepted')
            node.sim_wait(8.)
            reference = node.data['reference'].pose.position
            distance = math.sqrt((reference.x-start.x)**2+(reference.y-start.y)**2+
                                 (reference.z-start.z)**2)
            if distance < .005:
                raise RuntimeError('S4 reference did not move; aborting before failure injection')
            node.event('maneuver_pre_failure_check', reference_displacement_m=distance)
"""
    updated = updated.replace(anchor, anchor + maneuver)
    updated = updated.replace("node.event('post_failure_hover_end')",
                              "node.event('post_failure_maneuver_observation_end' if args.scenario == 'S4' else 'post_failure_hover_end')")
    ast.parse(updated)
config = yaml.safe_load((root / 'src/controller/config/failure_aware.yaml').read_text())
params = config['failure_controller']['ros__parameters']
if float(params['allocation_moment_weight']) != 1000.:
    raise SystemExit('Expected successful S2/S3 moment weight 1000. No settings changed.')
if updated != source:
    backup = path.with_name('run_failure_scenario.before_S4.py')
    if not backup.exists():
        shutil.copy2(path, backup)
    path.write_text(updated)
    print('S4 added; saved source backup:', backup)
else:
    print('S4 support already installed.')
print('Controller settings unchanged. S4 support installed; checking midpoint timing next.')
PY

# Correct sampling units and enforce a moving-reference midpoint check.
/usr/bin/python3 - "$workspace" <<'PY_MID'
from pathlib import Path
import ast,re,shutil,sys
root=Path(sys.argv[1])
codes=root/'src/maneuver/src/maneuver_codes.cc'
helpers=root/'src/maneuver/src/helpers.cc'
runner=root/'src/firmware/scripts/run_failure_scenario.py'
header=root/'src/maneuver/include/maneuver/maneuver.hpp'
if not re.search(r'int\s+period\s*=\s*10\s*;',header.read_text()):
    raise SystemExit('Expected the port with a 10 ms maneuver timer; no timing edits applied.')
old_codes=codes.read_text();old_helpers=helpers.read_text();old_runner=runner.read_text()
new_codes,count=re.subn(r'1000\.0\s*/\s*maneuver->period','maneuver->period',old_codes)
# Sampler arguments are milliseconds, and the publisher timer is 10 ms.
new_helpers=old_helpers.replace('static const double dt = maneuver_control_period_ms / 1000.;',
                                'const double dt = maneuver_control_period_ms / 1000.;')
new_runner=old_runner
if '# S4_MIDTRAJECTORY_V3' not in old_runner:
    begin=next((old_runner.find(marker) for marker in ('        # S4_SMOOTH_6D_V1:', '        # S4_MIDTRAJECTORY_V2:') if marker in old_runner), -1)
    end=old_runner.find('        lower, upper = original[:8], original[8:]',begin)
    if begin<0 or end<0:
        raise SystemExit('Cannot locate the installed S4 block; no timing edits applied.')
    block='''        # S4_MIDTRAJECTORY_V3: inject only while the reference is moving.
        if args.scenario == 'S4':
            start = node.data['reference'].pose.position
            target = dict(x=float(start.x)+2.0, y=float(start.y)+1.0, z=1.7,
                          roll=math.radians(10.), pitch=math.radians(-10.),
                          yaw=math.radians(20.), duration=20.)
            metadata_path = folder / 'scenario.json'
            metadata = json.loads(metadata_path.read_text())
            metadata.update(maneuver_service='/goto_6d_point', maneuver_target=target,
                            failure_after_maneuver_start_seconds=10.,
                            intended_settled_observation_after_maneuver_seconds=20.,
                            trajectory_sample_period_ms=10,
                            midtrajectory_reference_check=True)
            metadata_path.write_text(json.dumps(metadata, indent=2)+'\\n')
            node.event('maneuver_requested', **target)
            node.call(Goto6DPoint, '/goto_6d_point', **target)
            maneuver_start = node.sim_time()
            node.event('maneuver_accepted')
            node.sim_wait(9.8)
            before = node.data['reference'].pose.position
            before_xyz = (before.x, before.y, before.z)
            before_time = node.sim_time()
            node.sim_wait(.2)
            reference = node.data['reference'].pose.position
            xyz = (reference.x, reference.y, reference.z)
            elapsed = node.sim_time()-maneuver_start
            sample_dt = node.sim_time()-before_time
            measured_speed = math.sqrt(sum((x-y)**2 for x,y in zip(xyz,before_xyz)))/sample_dt
            remaining = math.sqrt(sum((x-target[k])**2 for x,k in zip(xyz,('x','y','z'))))
            traveled = math.sqrt((reference.x-start.x)**2+(reference.y-start.y)**2+
                                 (reference.z-start.z)**2)
            velocity = node.data['reference'].vel.linear
            feedforward_speed = math.sqrt(velocity.x**2+velocity.y**2+velocity.z**2)
            if measured_speed < .001 or feedforward_speed < .001 or remaining < .01 or traveled < .01:
                raise RuntimeError('S4 reference is not mid-trajectory; aborting BEFORE failure injection')
            if not .5 < measured_speed/feedforward_speed < 2.:
                raise RuntimeError('S4 reference timing/velocity mismatch; aborting BEFORE failure injection')
            if not 9.5 <= elapsed <= 10.5:
                raise RuntimeError('S4 missed the midpoint timing window; aborting BEFORE failure injection')
            node.event('maneuver_pre_failure_check', elapsed_seconds=elapsed,
                       reference_speed_m_s=measured_speed, distance_to_goal_m=remaining,
                       reference_displacement_m=traveled)
'''
    new_runner=old_runner[:begin]+block+old_runner[end:]
    # Give takeoff enough time for the planner's measured minimum of ~7.4 s.
    new_runner=new_runner.replace("node.event('takeoff_requested', height=1.5, duration=7.)", "node.event('takeoff_requested', height=1.5, duration=8. if args.scenario == 'S4' else 7.)")
    new_runner=new_runner.replace("node.call(LiftOff, '/lift_off', height=1.5, duration=7.)", "node.call(LiftOff, '/lift_off', height=1.5, duration=8. if args.scenario == 'S4' else 7.)")
    new_runner=new_runner.replace('node.sim_wait(7.)','node.sim_wait(8. if args.scenario == \'S4\' else 7.)')
    new_runner=new_runner.replace('takeoff_seconds=7.,', "takeoff_seconds=8. if args.scenario == 'S4' else 7.,")
ast.parse(new_runner)
for path,old,new in [(codes,old_codes,new_codes),(helpers,old_helpers,new_helpers),(runner,old_runner,new_runner)]:
    if old==new:continue
    backup=path.with_name(path.name+'.before_S4_longer_maneuver')
    if not backup.exists():shutil.copy2(path,backup)
    path.write_text(new)
print('Corrected sampler units: %d period conversions; publisher and samples both use 10 ms.' % count)
print('S4 now fails motor 6 at 10 s of a 20 s maneuver, only after a live-reference check.')
PY_MID

# Install the corrected maneuver code and runner; retain controller gains.
(
  cd -- "$workspace"
  colcon build --packages-select firmware controller maneuver --cmake-args -DCMAKE_BUILD_TYPE=Release
)
set +u
source "$workspace/install/setup.bash"
set -u
run_root=${OMNI_RESULTS_DIR:-"$workspace/simulation_results"}
mkdir -p -- "$run_root"
run_dir=$(mktemp -d "$run_root/S4_motor6_$(date +%Y%m%d_%H%M%S)_XXXXXX")
cp -- "${BASH_SOURCE[0]}" "$run_dir/run_S4_used.sh"
exec ros2 run firmware run_failure_scenario.py S4 --output "$run_dir"
