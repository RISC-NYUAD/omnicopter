#!/usr/bin/env bash
# S6 using the SAME simulation, recorder and landing sequence as S1-S5.
# Tilted hover -> motor 4 failure -> horizontal request -> ordinary landing.
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
/usr/bin/python3 - "$workspace" <<'PY_SIMPLE_S6'
from pathlib import Path
import ast,json,re,shutil,sys
root=Path(sys.argv[1]);firm=root/'src/firmware'
source=(firm/'scripts/run_failure_scenario.py').read_text()
# Use a separate copy of the established runner; S1-S5 remain untouched.
source=re.sub(r"choices=\[([^\]]+)\]",lambda m:"choices=["+m.group(1)+", 'S6']" if "'S6'" not in m.group(1) else m.group(0),source,count=1)
if "motor = 4 if args.scenario == 'S1' else 1" in source:
    source=source.replace("motor = 4 if args.scenario == 'S1' else 1","motor = 4 if args.scenario in ('S1', 'S6') else 1")
else:
    source,count=re.subn(r"motor = (\{[^\n]+\})\[args.scenario\]",r"motor = {**\1, 'S6': 4}[args.scenario]",source,count=1)
    if count!=1:raise SystemExit('Unrecognized motor mapping; no changes made.')
if 'from maneuver.srv import ArmDisarm, LiftOff, Land, Goto6DPoint' not in source:
    source=source.replace('from maneuver.srv import ArmDisarm, LiftOff, Land','from maneuver.srv import ArmDisarm, LiftOff, Land, Goto6DPoint')
anchor='        node.check_hover()\n'
if source.count(anchor)!=1:raise SystemExit('Cannot locate hover stage; no changes made.')
start='''        # SIMPLE_S6: initial tilted hover, followed by the ordinary failure stage.
        if args.scenario == 'S6':
            s6_start = node.data['reference'].pose.position
            s6_x, s6_y = float(s6_start.x), float(s6_start.y)
            tilted = dict(x=s6_x, y=s6_y, z=1.5, roll=math.radians(25.),
                          pitch=math.radians(-20.), yaw=math.radians(30.), duration=12.)
            metadata_path = folder / 'scenario.json'
            metadata = json.loads(metadata_path.read_text())
            metadata.update(initial_attitude_degrees=[25.,-20.,30.],
                            recovery_attitude_degrees=[0.,0.,30.],
                            landing_mode='Same land service and shutdown as S1-S5')
            metadata_path.write_text(json.dumps(metadata,indent=2)+'\\n')
            node.event('tilted_attitude_requested', **tilted)
            node.call(Goto6DPoint, '/goto_6d_point', **tilted)
            node.sim_wait(17.)
            q = node.data['pose'].pose.orientation
            qr = node.data['reference'].pose.orientation
            dot = abs(q.x*qr.x+q.y*qr.y+q.z*qr.z+q.w*qr.w)
            angle_error = math.degrees(2.*math.acos(min(1.,dot)))
            tilt = math.degrees(math.acos(max(-1.,min(1.,1.-2.*(q.x*q.x+q.y*q.y)))))
            if tilt < 15. or angle_error > 5.:
                raise RuntimeError('S6 tilted starting attitude not achieved; failure not injected')
            node.event('tilted_hover_confirmed', tilt_degrees=tilt)
'''
source=source.replace(anchor,anchor+start)
old='        node.sim_wait(30.)\n'
if source.count(old)!=1:raise SystemExit('Cannot locate post-failure stage; no changes made.')
recovery='''        if args.scenario == 'S6':
            node.sim_wait(2.)
            horizontal = dict(x=s6_x, y=s6_y, z=1.5, roll=0., pitch=0.,
                              yaw=math.radians(30.), duration=10.)
            node.event('horizontal_attitude_requested', **horizontal)
            node.call(Goto6DPoint, '/goto_6d_point', **horizontal)
            node.sim_wait(20.)
            q = node.data['pose'].pose.orientation
            tilt = math.degrees(math.acos(max(-1.,min(1.,1.-2.*(q.x*q.x+q.y*q.y)))))
            if tilt > 5.:
                raise RuntimeError('Horizontal recovery incomplete; inspect the bag before landing')
            node.event('horizontal_recovery_confirmed', tilt_degrees=tilt)
        else:
            node.sim_wait(30.)
'''
source=source.replace(old,recovery)
# Eight-second takeoff in this dedicated S6 runner, avoiding the old 7.4 s minimum warning.
source=re.sub(r"node.event\('takeoff_requested', height=1.5, duration=[^\n]+\)","node.event('takeoff_requested', height=1.5, duration=8.)",source)
source=re.sub(r"node.call\(LiftOff, '/lift_off', height=1.5, duration=[^\n]+\)","node.call(LiftOff, '/lift_off', height=1.5, duration=8.)",source)
a=source.index('        node.flight = True\n');b=source.index("        node.event('pre_failure_hover_start')",a)
source=source[:a]+"        node.flight = True\n        node.sim_wait(8.)\n"+source[b:]
source=re.sub(r'takeoff_seconds=[^\n]+?, pre_failure_hover_seconds=10\.,','takeoff_seconds=8., pre_failure_hover_seconds=10.,',source)
ast.parse(source)
path=firm/'scripts/run_s6_simple.py'
if path.exists() and path.read_text()!=source:
    backup=path.with_name('run_s6_simple.before_update.py')
    if not backup.exists():shutil.copy2(path,backup)
path.write_text(source)
cmake=firm/'CMakeLists.txt';text=cmake.read_text()
if 'scripts/run_s6_simple.py' not in text:
    backup=cmake.with_name('CMakeLists.txt.before_simple_S6')
    if not backup.exists():shutil.copy2(cmake,backup)
    text=text.replace('ament_package()','install(PROGRAMS scripts/run_s6_simple.py DESTINATION lib/${PROJECT_NAME})\nament_package()')
    cmake.write_text(text)
print('Simple S6 installed: normal failure_aware launch, same topics and same land service as previous runs.')
print('No contact bridge, extra sensors, custom world, or controller changes used by this S6.')
PY_SIMPLE_S6
(
 cd -- "$workspace"
 colcon build --packages-select firmware --cmake-args -DCMAKE_BUILD_TYPE=Release
)
set +u
source "$workspace/install/setup.bash"
set -u
run_root=${OMNI_RESULTS_DIR:-"$workspace/simulation_results"}
mkdir -p -- "$run_root"
run_dir=$(mktemp -d "$run_root/S6_motor4_$(date +%Y%m%d_%H%M%S)_XXXXXX")
cp -- "${BASH_SOURCE[0]}" "$run_dir/run_S6_used.sh"
exec ros2 run firmware run_s6_simple.py S6 --output "$run_dir"
