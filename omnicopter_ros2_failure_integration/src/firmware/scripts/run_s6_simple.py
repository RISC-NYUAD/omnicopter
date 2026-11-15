#!/usr/bin/env python3
"""One-Enter S1/S2 runners. Timing is simulated; recordings are rosbag2 SQLite."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rcl_interfaces.srv import GetParameters
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, String, Float64MultiArray
from maneuver.msg import FullPose
from maneuver.srv import ArmDisarm, LiftOff, Land, Goto6DPoint
from controller.msg import AllocatorStatus
from controller.srv import SetMotorLimits

TOPICS = [
    '/pose_full', '/pose_d', '/failure_controller/status',
    '/omnicopter/motor_thrust', '/omnicopter/motor_thrust_actual',
    '/omnicopter/motor_limits', '/clock', '/toggle_topic',
    '/rosout', '/parameter_events', '/simulation_scenarios/events',
]


def stop_process(process, label):
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=30)
    except subprocess.TimeoutExpired:
        print(label + ' did not close in 30 s; sending SIGTERM.', flush=True)
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=10)


class Scenario(Node):
    def __init__(self, folder):
        super().__init__('failure_scenario', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.folder = folder
        self.data = {}
        self.received = {}
        self.bag = None
        self.launch = None
        self.flight = False
        self.subscriptions_kept = []
        durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        clock_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        for topic, kind, key, qos in [
            ('/pose_full', FullPose, 'pose', 10),
            ('/pose_d', FullPose, 'reference', 10),
            ('/clock', Clock, 'clock', clock_qos),
            ('/omnicopter/motor_thrust_actual', Float64MultiArray, 'actual', 10),
            ('/omnicopter/motor_limits', Float64MultiArray, 'limits', durable),
            ('/toggle_topic', Bool, 'armed', 10),
            ('/failure_controller/status', AllocatorStatus, 'status', 10),
        ]:
            def receive(message, key=key):
                self.data[key] = message
                self.received[key] = time.monotonic()
            self.subscriptions_kept.append(self.create_subscription(kind, topic, receive, qos))
        self.events = self.create_publisher(String, '/simulation_scenarios/events', durable)

    def wait(self, predicate, timeout, what, monitor=True):
        end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=.02)
            if monitor:
                for process, name in [(self.launch, 'Simulation launch'), (self.bag, 'Recorder')]:
                    if process is not None and process.poll() is not None:
                        raise RuntimeError(name + ' stopped; check the saved logs')
            if predicate():
                return
        raise RuntimeError('Timed out: ' + what)

    def sim_time(self):
        return self.get_clock().now().nanoseconds / 1e9

    def sim_wait(self, seconds):
        start = previous = self.sim_time()
        progressed = time.monotonic()
        while self.sim_time() - start < seconds:
            self.wait(lambda: True, 1, 'simulation spin')
            now = self.sim_time()
            if now < previous:
                raise RuntimeError('Simulation time moved backwards')
            if now > previous:
                previous, progressed = now, time.monotonic()
            if time.monotonic() - progressed > 20:
                raise RuntimeError('Simulation clock stalled')
            for key in (['pose', 'actual', 'status', 'reference'] if self.flight else ['pose', 'actual']):
                if time.monotonic() - self.received.get(key, 0) > 3:
                    raise RuntimeError('Missing/stale ' + key)

    def call(self, kind, name, **kwargs):
        client = self.create_client(kind, name)
        try:
            self.wait(lambda: client.service_is_ready(), 30, name + ' availability')
            request = kind.Request()
            for key, value in kwargs.items():
                setattr(request, key, value)
            future = client.call_async(request)
            self.wait(future.done, 15, name + ' response')
            response = future.result()
            if response is None:
                raise RuntimeError(name + ' returned no response')
            success = getattr(response, 'success', getattr(response, 'status', None))
            if success is not None and not success:
                raise RuntimeError(name + ' rejected request: ' + str(response))
            return response
        finally:
            self.destroy_client(client)

    def event(self, name, **details):
        record = dict(event=name, sim_time=self.sim_time(), wall_time_unix=time.time(), **details)
        text = json.dumps(record)
        with (self.folder / 'events.jsonl').open('a') as stream:
            stream.write(text + '\n')
        self.events.publish(String(data=text))
        print(text, flush=True)

    def snapshot(self):
        names = ['use_sim_time', 'allocation_mode', 'mass', 'inertia', 'allocation', 'thrust_min',
                 'thrust_max', 'position_kp', 'position_kd', 'position_ki', 'attitude_kp',
                 'attitude_kd', 'frequency', 'gravity', 'actuator_reserve', 'beta', 'gamma',
                 'allocation_force_weight', 'allocation_moment_weight',
                 'max_solve_seconds', 'max_evaluations', 'constraint_tolerance',
                 'governor_warm_budget', 'governor_seed_budget', 'recovery_max_evaluations',
                 'position_error_limit', 'velocity_error_limit', 'integral_limit', 'state_timeout']
        response = self.call(GetParameters, '/failure_controller/get_parameters', names=names)
        values = {}
        for key, value in zip(names, response.values):
            fields = {1: 'bool_value', 2: 'integer_value', 3: 'double_value', 4: 'string_value',
                      8: 'double_array_value', 7: 'integer_array_value'}
            field = fields.get(value.type)
            if field is None:
                raise RuntimeError('Missing/unsupported parameter ' + key)
            result = getattr(value, field)
            values[key] = list(result) if value.type in (7, 8) else result
        if values['allocation_mode'] != 'governed' or not values['use_sim_time']:
            raise RuntimeError('Incorrect controller mode or time source')
        (self.folder / 'controller_parameters.json').write_text(json.dumps(values, indent=2) + '\n')
        return values

    def check_hover(self):
        p, ref = self.data['pose'], self.data['reference']
        error = math.sqrt(sum((getattr(p.pose.position, a)-getattr(ref.pose.position, a))**2 for a in 'xyz'))
        speed = math.sqrt(sum(getattr(p.vel.linear, a)**2 for a in 'xyz'))
        q = ref.pose.orientation
        tilt = math.acos(max(-1., min(1., 1.-2.*(q.x*q.x+q.y*q.y))))
        if error > .1 or speed > .1 or abs(p.pose.position.z-1.5) > .15 or tilt > math.radians(2):
            raise RuntimeError('Not settled in horizontal hover: error %.3f m; speed %.3f m/s' % (error, speed))


def run():
    parser = argparse.ArgumentParser()
    parser.add_argument('scenario', choices=['S1', 'S2', 'S3', 'S4', 'S5', 'S6'])
    parser.add_argument('--output', required=True)
    parser.add_argument('--headless', action='store_true')
    args = parser.parse_args()
    folder = Path(args.output).resolve()
    folder.mkdir(parents=True, exist_ok=True)
    motor = {**{'S1': 4, 'S2': 1, 'S3': 6, 'S4': 6, 'S5': 4}, 'S6': 4}[args.scenario]
    # S5_DOUBLE_FAILURE_V1: both bounds change in one service request.
    failed_motors = [4, 8] if args.scenario == 'S5' else [motor]
    def interrupt(*_):
        raise KeyboardInterrupt()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, interrupt)
    signal.signal(signal.SIGINT, interrupt)
    node = Scenario(folder)
    logs = []
    completed = False
    try:
        # Existing nodes are not ours to terminate or re-arm.
        for _ in range(30):
            rclpy.spin_once(node, timeout_sec=.05)
        conflicts = set(node.get_node_names()) & {'failure_controller', 'maneuver', 'beta_motor_plugin', 'controller'}
        if conflicts:
            raise RuntimeError('Close the existing simulation first: ' + ', '.join(sorted(conflicts)))
        launch_log = (folder / 'roslaunch.log').open('w')
        logs.append(launch_log)
        node.launch = subprocess.Popen(
            ['ros2', 'launch', 'firmware', 'failure_aware.launch.py',
             'gui:=' + ('false' if args.headless else 'true')],
            stdout=launch_log, stderr=subprocess.STDOUT, start_new_session=True)
        node.wait(lambda: all(key in node.data for key in ['pose', 'actual', 'limits', 'clock'])
                  and node.sim_time() > 0, 120, 'simulation, thrust dynamics and clock')
        if node.data['pose'].pose.position.z > .6:
            raise RuntimeError('Expected fresh simulation on the ground')
        config = node.snapshot()
        original = list(node.data['limits'].data)
        if len(original) != 16 or any(abs(v) > 1e-9 for v in original[:8]) or any(v <= 0 for v in original[8:]):
            raise RuntimeError('Expected eight healthy motor-limit intervals')
        (folder / 'scenario.json').write_text(json.dumps(dict(
            scenario=args.scenario, motor_number=None if args.scenario == 'S5' else motor,
            rotor_index=None if args.scenario == 'S5' else motor-1,
            failed_motor_numbers=failed_motors, failed_rotor_indices=[m-1 for m in failed_motors],
            effectiveness=0.,
            height_m=1.5, takeoff_seconds=8., pre_failure_hover_seconds=10.,
            post_failure_seconds=30., detection_delay_seconds=0., video=False,
            topics=TOPICS, physical_model='worlds/models/omnicopter_failure/model.sdf',
            tau_up=.15, tau_down=.21,
            note='Failure remains active through landing; sequence completion is not a performance pass.'
        ), indent=2) + '\n')
        qos = folder / 'recording_qos.yaml'
        qos.write_text('/omnicopter/motor_limits:\n  reliability: reliable\n  durability: transient_local\n  history: keep_last\n  depth: 1\n/clock:\n  reliability: best_effort\n  durability: volatile\n  history: keep_last\n  depth: 10\n')
        bag_log = (folder / 'rosbag.log').open('w')
        logs.append(bag_log)
        node.bag = subprocess.Popen([
            'ros2', 'bag', 'record', '--disable-keyboard-controls', '--storage', 'sqlite3', '--output',
            str(folder / (args.scenario + ('_motors4_8' if args.scenario == 'S5' else '_motor' + str(motor)))),
            '--qos-profile-overrides-path', str(qos), '--topics', *TOPICS
        ], stdin=subprocess.DEVNULL, stdout=bag_log, stderr=subprocess.STDOUT, start_new_session=True)
        # The event topic has only the recorder as a subscriber in this fresh run.
        def recorder_ready():
            if node.events.get_subscription_count() == 0:
                return False
            for topic in ['/clock', '/pose_full', '/omnicopter/motor_thrust_actual', '/omnicopter/motor_limits']:
                if not any('rosbag' in entry.node_name for entry in node.get_subscriptions_info_by_topic(topic)):
                    return False
            return True
        node.wait(recorder_ready, 30, 'recorder subscriptions')
        node.event('recording_ready')
        print('Recording ROS data only. Position the Gazebo camera if desired.', flush=True)
        input('Press [Enter] to run ' + args.scenario + ': ')
        # DDS discovery continues while waiting for the prompt; drain queued callbacks.
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=.01)
        node.event('arm_requested')
        node.call(ArmDisarm, '/arm_disarm')
        node.wait(lambda: node.data.get('armed') is not None and node.data['armed'].data, 5, 'armed')
        node.sim_wait(.5)
        node.event('takeoff_requested', height=1.5, duration=8.)
        node.call(LiftOff, '/lift_off', height=1.5, duration=8.)
        node.wait(lambda: 'status' in node.data and 'reference' in node.data, 5, 'reference and allocator status')
        node.flight = True
        node.sim_wait(8.)
        node.event('pre_failure_hover_start')
        node.sim_wait(10.)
        node.check_hover()
        # SIMPLE_S6: initial tilted hover, followed by the ordinary failure stage.
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
            metadata_path.write_text(json.dumps(metadata,indent=2)+'\n')
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
        # S4_MIDTRAJECTORY_V3: inject only while the reference is moving.
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
            metadata_path.write_text(json.dumps(metadata, indent=2)+'\n')
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
        lower, upper = original[:8], original[8:]
        for failed_motor in failed_motors:
            lower[failed_motor-1] = upper[failed_motor-1] = 0.
        node.event('failure_requested', motor=motor if len(failed_motors) == 1 else None,
                   motors=failed_motors, rotor_indices=[m-1 for m in failed_motors], upper_N=0.)
        node.call(SetMotorLimits, '/failure_controller/set_motor_limits', lower=lower, upper=upper)
        node.wait(lambda: list(node.data['limits'].data) == lower+upper, 5, 'failure bounds confirmation')
        node.event('failure_confirmed', motor=motor if len(failed_motors) == 1 else None,
                   motors=failed_motors)
        if args.scenario == 'S6':
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
        node.event('post_failure_maneuver_observation_end' if args.scenario == 'S4' else 'post_failure_hover_end')
        node.event('landing_requested')
        node.call(Land, '/land', height_1=1.2, duration_1=3., height_2=.35, duration_2=6.)
        node.sim_wait(14.)
        p = node.data['pose']
        speed = math.sqrt(sum(getattr(p.vel.linear, a)**2 for a in 'xyz'))
        if p.pose.position.z > .5 or speed > .15:
            raise RuntimeError('Landing endpoint not low/slow; stopping owned simulation without toggling disarm')
        node.flight = False
        node.event('disarm_requested')
        node.call(ArmDisarm, '/arm_disarm')
        node.wait(lambda: not node.data['armed'].data, 5, 'disarmed')
        node.sim_wait(3.)
        node.event('scenario_complete')
        # Give DDS / rosbag time to receive the final event before closing.
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=.02)
        completed = True
    except (Exception, KeyboardInterrupt) as error:
        print('\nScenario interrupted: ' + (str(error) or type(error).__name__), flush=True)
        try:
            folder.mkdir(parents=True, exist_ok=True)
            (folder / 'INTERRUPTED.txt').write_text(str(error) or type(error).__name__)
        except OSError:
            pass
    finally:
        try:
            stop_process(node.bag, 'Recorder')
        finally:
            stop_process(node.launch, 'Simulation')
            for log in logs:
                log.close()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        if completed:
            (folder / 'COMPLETE').write_text('Scenario sequence completed; evaluate the recorded performance.\n')
        print('Run files: ' + str(folder), flush=True)
    return 0 if completed else 1


if __name__ == '__main__':
    raise SystemExit(run())
