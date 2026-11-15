#!/usr/bin/env bash
# S2: horizontal hover, complete failure of paper motor 1 (rotor_0).
# Run: bash run_S2.sh [catkin_workspace]
# Records ROS data only; no screen/video recording.
# Close an existing simulation first. One Enter starts the entire flight.
set -Eeuo pipefail

# ROS Noetic uses Ubuntu system Python, including when Conda base is active.
export PATH="/usr/bin:/bin:$PATH"

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
workspace=${1:-${CATKIN_WS:-}}
if [[ -z "$workspace" ]]; then
  for candidate in "$script_dir/.." "$script_dir" "$HOME/Downloads/omnicopter_failure_integration/my_code"; do
    if [[ -f "$candidate/devel/setup.bash" ]]; then workspace=$candidate; break; fi
  done
fi
# ROS setup scripts are not guaranteed to tolerate nounset.
set +u
if [[ -n "$workspace" && -f "$workspace/devel/setup.bash" ]]; then
  source "$workspace/devel/setup.bash"
fi
set -u
for command in roslaunch rosparam rospack /usr/bin/python3 setsid; do
  command -v "$command" >/dev/null || { echo "Missing $command. Source your catkin devel/setup.bash or pass the workspace path." >&2; exit 1; }
done
/usr/bin/python3 -c 'import rospy, rosbag, rosservice, rosnode' || exit 1
rospack find omni_firmware >/dev/null

# Never launch a second controller/plant into an existing run.
/usr/bin/python3 - <<'PY'
import sys, rosgraph
try:
    nodes = rosgraph.Master('/s2_preflight').getSystemState()
except Exception:
    sys.exit(0)  # roslaunch may create a new master
owned = {n for section in nodes for _, names in section for n in names}
conflicts = sorted(n for n in owned if n in ('/gazebo', '/failure_controller')
                   or n.rsplit('/', 1)[-1] in ('maneuver', 'controller'))
if conflicts:
    sys.exit('Close the existing simulation/controllers first: ' + ', '.join(conflicts))
PY

run_root=${S2_OUTPUT_DIR:-"$HOME/omnicopter_results"}
mkdir -p -- "$run_root"
run_root=$(cd -- "$run_root" && pwd -P)
run_dir=$(mktemp -d "$run_root/S2_motor1_$(date +%Y%m%d_%H%M%S)_XXXXXX")
run_dir=$(cd -- "$run_dir" && pwd)
cp -- "${BASH_SOURCE[0]}" "$run_dir/run_S2_used.sh"
echo "Saving S2 data to: $run_dir"
launch_pid=''
cleanup() {
  local result=$?
  trap - EXIT INT TERM
  if [[ -n "$launch_pid" ]] && kill -0 "$launch_pid" 2>/dev/null; then
    kill -INT -- "-$launch_pid" 2>/dev/null || true
    for ((i=0; i<100; i++)); do
      kill -0 "$launch_pid" 2>/dev/null || break
      sleep 0.1
    done
    kill -TERM -- "-$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
  fi
  echo "Run files: $run_dir"
  exit "$result"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

# Parameters after the include override the included defaults before nodes start.
cat > "$run_dir/S2.launch" <<'XML'
<launch>
  <include file="$(find omni_firmware)/launch/failure_aware.launch">
    <arg name="allocation_mode" value="governed"/>
  </include>
  <param name="/failure_controller/allocation_mode" value="governed"/>
  <param name="/failure_controller/actuator_reserve" value="1.5"/>
  <param name="/failure_controller/allocation_force_weight" value="100.0"/>
  <param name="/failure_controller/allocation_moment_weight" value="10.0"/>
  <param name="/failure_controller/beta" value="0.00001"/>
  <param name="/failure_controller/gamma" value="0.1"/>
  <param name="/failure_controller/frequency" value="250.0"/>
  <param name="/omnicopter_sim/thrust_tau_up" value="0.15"/>
  <param name="/omnicopter_sim/thrust_tau_down" value="0.21"/>
</launch>
XML
setsid roslaunch "$run_dir/S2.launch" >"$run_dir/roslaunch.log" 2>&1 &
launch_pid=$!
export S2_RUN_DIR="$run_dir"
/usr/bin/python3 -u - <<'PY' 2>&1 | tee "$run_dir/scenario.log"
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

import rosgraph
import rosservice
import rospy
from omni_firmware.msg import FullPose
from std_msgs.msg import Bool, Float64MultiArray, String
from controller.msg import AllocatorStatus

RUN = Path(os.environ['S2_RUN_DIR'])
RUN.mkdir(parents=True, exist_ok=True)
MOTOR = 1                    # Paper/user number; zero-based array index is 0.
HEIGHT = 1.5
TAKEOFF = 7.0
PRE_FAILURE = 10.0           # Hover after completing takeoff.
POST_FAILURE = 30.0
LAND_DURATION = 9.0
TOPICS = [
    '/pose_full', '/pose_d', '/failure_controller/status',
    '/omnicopter/motor_thrust', '/omnicopter/motor_thrust_actual',
    '/omnicopter/motor_limits', '/clock', '/toggle_topic',
    '/gazebo/model_states', '/rosout_agg', '/simulation_scenarios/events',
]
state = {}
lock = threading.Lock()
bag = None
flight_started = False
bag_node = '/s2_bag_' + str(os.getpid())


def receive(key):
    def callback(msg):
        with lock:
            state[key] = (msg, time.monotonic())
    return callback


def latest(key, age=2.0):
    with lock:
        value = state.get(key)
    if value is None or time.monotonic() - value[1] > age:
        raise RuntimeError('Missing/stale ' + key + ' data')
    return value[0]


def wait_for(predicate, timeout, description):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if rospy.is_shutdown():
            raise RuntimeError('ROS shutdown while ' + description)
        try:
            if predicate():
                return
        except (RuntimeError, rospy.ROSException, OSError):
            pass
        time.sleep(0.05)
    raise RuntimeError('Timed out: ' + description)


def check_recording():
    if bag is None or bag.poll() is not None:
        raise RuntimeError('rosbag stopped; see rosbag.log')


def sim_wait(seconds):
    """Use simulated seconds, and fail if simulation time stalls/reverses."""
    start = previous = rospy.Time.now().to_sec()
    progressed = time.monotonic()
    while rospy.Time.now().to_sec() - start < seconds:
        check_recording()
        latest('pose')
        latest('actual_thrust')
        if flight_started:
            latest('status')
            latest('reference')
        now = rospy.Time.now().to_sec()
        if now < previous:
            raise RuntimeError('Simulation clock moved backwards')
        if now > previous:
            previous, progressed = now, time.monotonic()
        elif time.monotonic() - progressed > 20:
            raise RuntimeError('Simulation clock stalled for 20 wall seconds')
        if rospy.is_shutdown():
            raise RuntimeError('ROS shutdown')
        time.sleep(0.02)


def service(name, **arguments):
    check_recording()
    rospy.wait_for_service(name, timeout=10)
    kind = rosservice.get_service_class_by_name(name)
    if kind is None:
        raise RuntimeError('Cannot resolve service ' + name)
    response = rospy.ServiceProxy(name, kind)(**arguments)
    if not bool(getattr(response, 'success', getattr(response, 'status', False))):
        raise RuntimeError(name + ' rejected request: ' + str(response))
    print(name + ': ' + str(response).replace('\n', '; '), flush=True)
    return response


def event(label, **details):
    payload = dict(event=label, sim_time=rospy.Time.now().to_sec(), **details)
    with (RUN / 'events.jsonl').open('a') as stream:
        stream.write(json.dumps(payload) + '\n')
    event_pub.publish(String(data=json.dumps(payload)))
    print(json.dumps(payload), flush=True)


def check_hover():
    pose = latest('pose')
    reference = latest('reference')
    p, r = pose.pose.position, reference.pose.position
    error = math.sqrt((p.x-r.x)**2 + (p.y-r.y)**2 + (p.z-r.z)**2)
    speed = math.sqrt(sum(getattr(pose.vel.linear, a)**2 for a in 'xyz'))
    q = reference.pose.orientation
    qnorm = math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
    tilt = math.acos(max(-1., min(1., 1.-2.*(q.x*q.x+q.y*q.y)/(qnorm*qnorm))))
    status = latest('status')
    if status.allocation_mode != 'governed':
        raise RuntimeError('Wrong allocation mode in status message')
    if error > .10 or speed > .10 or abs(p.z-HEIGHT) > .15 or tilt > math.radians(2):
        raise RuntimeError('Not settled in horizontal hover: position error %.3f m, speed %.3f m/s, reference tilt %.2f deg' % (error, speed, math.degrees(tilt)))


# Start a fresh simulation only; ArmDisarm toggles state and is not idempotent.
try:
    end = time.monotonic() + 120
    while not rospy.is_shutdown():
        try:
            rosgraph.Master('/s2_startup').getPid()
            break
        except Exception:
            if time.monotonic() > end:
                raise RuntimeError('ROS master startup timed out; see roslaunch.log')
            time.sleep(.2)
    rospy.init_node('s2_scenario', anonymous=True, disable_signals=True)
    signal.signal(signal.SIGINT, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    subscribers = [
        rospy.Subscriber('/pose_full', FullPose, receive('pose'), queue_size=1),
        rospy.Subscriber('/pose_d', FullPose, receive('reference'), queue_size=1),
        rospy.Subscriber('/omnicopter/motor_limits', Float64MultiArray, receive('limits'), queue_size=1),
        rospy.Subscriber('/omnicopter/motor_thrust_actual', Float64MultiArray, receive('actual_thrust'), queue_size=1),
        rospy.Subscriber('/failure_controller/status', AllocatorStatus, receive('status'), queue_size=1),
        rospy.Subscriber('/toggle_topic', Bool, receive('armed'), queue_size=1),
    ]
    event_pub = rospy.Publisher('/simulation_scenarios/events', String, queue_size=20, latch=True)
    for name in ['/ArmDisarm', '/lift_off', '/land', '/failure_controller/set_motor_limits']:
        rospy.wait_for_service(name, timeout=120)
    wait_for(lambda: rospy.Time.now().to_sec() > 0 and latest('pose') and latest('actual_thrust'),
             120, 'live pose, actual thrust and simulation clock; check dynamics patch and unpause Gazebo')
    if not rospy.get_param('/use_sim_time', False):
        raise RuntimeError('/use_sim_time must be true')
    for name, wanted in dict(allocation_mode='governed', actuator_reserve=1.5,
                             allocation_force_weight=100., allocation_moment_weight=10.,
                             beta=1e-5, gamma=.1, frequency=250.).items():
        got = rospy.get_param('/failure_controller/' + name)
        if got != wanted:
            raise RuntimeError('Unexpected controller parameter %s=%s (expected %s)' % (name, got, wanted))
    bounds = rospy.wait_for_message('/omnicopter/motor_limits', Float64MultiArray, timeout=10).data
    if len(bounds) != 16:
        raise RuntimeError('Expected eight lower bounds followed by eight upper bounds')
    lower, upper = list(bounds[:8]), list(bounds[8:])
    if any(abs(x) > 1e-9 for x in lower) or any(x <= 0 for x in upper):
        raise RuntimeError('S2 requires an initially healthy motor-limit vector')
    p = latest('pose').pose.position
    if p.z > .5:
        raise RuntimeError('Expected fresh simulation on the ground before arming')
    publishers = rosgraph.Master(rospy.get_name()).getSystemState()[0]
    motor_publishers = next((names for topic, names in publishers if topic == '/omnicopter/motor_thrust'), [])
    if motor_publishers != ['/failure_controller']:
        raise RuntimeError('Expected only failure_controller on motor_thrust; found ' + str(motor_publishers))
    # Confirm the output directory exists immediately before external writers.
    RUN.mkdir(parents=True, exist_ok=True)
    with (RUN / 'write_check.tmp').open('w') as probe:
        probe.write('output directory writable\n')
    (RUN / 'write_check.tmp').unlink()
    subprocess.run(['rosparam', 'dump', str(RUN / 'parameters_before.yaml')], check=True)
    for name, filename in [('/failure_controller/status', 'AllocatorStatus_definition.txt'),
                           ('/pose_full', 'FullPose_definition.txt')]:
        kind = subprocess.check_output(['rostopic', 'type', name], text=True).strip()
        with (RUN / filename).open('w') as stream:
            subprocess.run(['rosmsg', 'show', kind], stdout=stream, check=True)
    metadata = dict(scenario='S2', motor_number=MOTOR, rotor_index=MOTOR-1,
                    remaining_effectiveness=0., height_m=HEIGHT, takeoff_seconds=TAKEOFF,
                    pre_failure_hover_seconds=PRE_FAILURE, post_failure_seconds=POST_FAILURE,
                    added_detection_delay_seconds=0., healthy_lower_N=lower, healthy_upper_N=upper,
                    expected_behavior='Adapt attitude as needed while retaining position reference; quantify from data.',
                    topics=TOPICS, note='Motor remains failed during landing. Failure time is relative to hover, not simulator startup. See events.jsonl and recorded limit change.')
    (RUN / 'scenario.json').write_text(json.dumps(metadata, indent=2) + '\n')
    print('\nS2: motor 1 (rotor_0), complete failure; 1.5 m horizontal hover.\n'
          'Recording starts now. One Enter starts arm -> takeoff -> 10 s hover -> failure -> 30 s hover -> land.\n'
          'The failure remains active through landing. Ctrl+C stops this simulation and closes the bag.\n', flush=True)
    bag_log = (RUN / 'rosbag.log').open('w')
    bag = subprocess.Popen(['rosbag', 'record', '--buffsize=256', '-O', str(RUN / 'S2_motor1.bag'),
                            *TOPICS, '__name:=' + bag_node[1:]], stdout=bag_log,
                           stderr=subprocess.STDOUT, start_new_session=True)
    def recorder_ready():
        check_recording()
        registrations = rosgraph.Master(rospy.get_name()).getSystemState()[1]
        registered = {t for t, names in registrations if bag_node in names}
        return set(TOPICS).issubset(registered) and event_pub.get_num_connections() > 0
    wait_for(recorder_ready, 30, 'rosbag topic subscriptions')
    event('recording_ready')
    # stdin contains this heredoc's Python source; read the keyboard from tty.
    print('Press [Enter] to run S2: ', end='', flush=True)
    with open('/dev/tty', 'r') as terminal:
        if not terminal.readline():
            raise RuntimeError('No terminal input available for the start prompt')
    event('arm_requested')
    service('/ArmDisarm')
    wait_for(lambda: latest('armed').data, 5, 'armed confirmation')
    sim_wait(.5)
    event('takeoff_requested', height=HEIGHT, duration=TAKEOFF)
    service('/lift_off', height=HEIGHT, duration=TAKEOFF)
    wait_for(lambda: latest('status') and latest('reference'), 5, 'controller status and reference')
    flight_started = True
    sim_wait(TAKEOFF)
    event('pre_failure_hover_start')
    sim_wait(PRE_FAILURE)
    check_hover()
    index = MOTOR - 1
    failed_lower, failed_upper = lower[:], upper[:]
    failed_lower[index] = failed_upper[index] = 0.
    event('failure_requested', motor=MOTOR, rotor_index=index, upper_N=0.)
    service('/failure_controller/set_motor_limits', lower=failed_lower, upper=failed_upper)
    wait_for(lambda: list(latest('limits').data) == failed_lower + failed_upper,
             5, 'updated motor limits')
    event('failure_confirmed', motor=MOTOR)
    sim_wait(POST_FAILURE)
    event('post_failure_hover_end')
    event('landing_requested')
    service('/land', height_1=1.2, duration_1=3., height_2=.35, duration_2=6.)
    sim_wait(LAND_DURATION + 5.)
    # The existing land service finishes at 0.35 m; verify low, slow flight
    # before toggling disarm. This shutdown is NOT S6 landing validation.
    pose = latest('pose')
    speed = math.sqrt(sum(getattr(pose.vel.linear, a)**2 for a in 'xyz'))
    if pose.pose.position.z > .5 or speed > .15:
        raise RuntimeError('Landing endpoint not low/slow; stopping the owned simulation without toggling disarm')
    event('disarm_requested')
    flight_started = False  # Status is published only while armed.
    service('/ArmDisarm')
    wait_for(lambda: not latest('armed').data, 5, 'disarmed confirmation')
    sim_wait(3.)
    event('scenario_complete')
    subprocess.run(['rosparam', 'dump', str(RUN / 'parameters_after.yaml')], check=True)
    (RUN / 'COMPLETE').write_text('S2 sequence completed. Assess recovery from the bag; completion is not a performance pass.\n')
except (Exception, KeyboardInterrupt) as error:
    print('\nS2 INTERRUPTED: ' + (str(error) or type(error).__name__), flush=True)
    try:
        RUN.mkdir(parents=True, exist_ok=True)
        (RUN / 'INTERRUPTED.txt').write_text(str(error) or type(error).__name__)
    except OSError as log_error:
        print('Could not save interruption details: ' + str(log_error), flush=True)
    raise SystemExit(1)
finally:
    if bag is not None:
        if bag.poll() is None:
            os.killpg(bag.pid, signal.SIGINT)
            try:
                bag.wait(timeout=30)
            except subprocess.TimeoutExpired:
                print('Recorder did not close within 30 s; sending SIGTERM. Check .bag.active before analysis.', flush=True)
                os.killpg(bag.pid, signal.SIGTERM)
                bag.wait(timeout=10)
        if bag.returncode not in (0, -signal.SIGINT):
            print('Recorder exit code:', bag.returncode, flush=True)
        bag_log.close()
    rospy.signal_shutdown('S2 finished')
PY
