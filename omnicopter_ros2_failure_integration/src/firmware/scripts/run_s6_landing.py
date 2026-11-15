#!/usr/bin/env python3
"""S6: tilted hover -> motor 4 loss -> level recovery -> contact-confirmed landing."""
import argparse,json,math,os,signal,subprocess,time
from pathlib import Path
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile,ReliabilityPolicy
from ros_gz_interfaces.msg import Contacts
from maneuver.srv import ArmDisarm,LiftOff,Goto6DPoint
from controller.srv import SetMotorLimits
from run_failure_scenario import Scenario,TOPICS,stop_process

CONTACT_TOPICS={'base':'/omnicopter/s6/contact_base',**{'rotor_'+str(i):'/omnicopter/s6/contact_rotor_'+str(i) for i in range(8)}}

def tilt_deg(pose):
    q=pose.pose.orientation
    n=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w
    return math.degrees(math.acos(max(-1.,min(1.,1.-2.*(q.x*q.x+q.y*q.y)/n))))

def speed(pose):return math.sqrt(sum(getattr(pose.vel.linear,a)**2 for a in 'xyz'))
def omega(pose):return math.sqrt(sum(getattr(pose.vel.angular,a)**2 for a in 'xyz'))

class LandingScenario(Scenario):
    def __init__(self,folder):
        super().__init__(folder)
        self.contacts={};self.last_contact={};self.ever_rotor_contact=False;self.monitor_rotors=False
        qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT)
        for key,topic in CONTACT_TOPICS.items():
            def receive(message,key=key):
                pairs=[(c.collision1.name,c.collision2.name) for c in message.contacts]
                ground=any('ground_plane' in a or 'ground_plane' in b for a,b in pairs)
                self.contacts[key]=(ground,self.sim_time(),pairs)
                if ground:self.last_contact[key]=self.sim_time()
                if ground and key!='base':self.ever_rotor_contact=True
            self.subscriptions_kept.append(self.create_subscription(Contacts,topic,receive,qos))
    def on_ground(self):
        sample=self.contacts.get('base')
        # Require a live contact observation; do not reuse initial ground contact.
        return bool(sample and sample[0] and self.sim_time()-sample[1]<.15)
    def sim_wait(self,seconds):
        if self.monitor_rotors and self.ever_rotor_contact:
            raise RuntimeError('Rotor-ground contact detected; not a successful landing')
        super().sim_wait(seconds)
        if self.monitor_rotors and self.ever_rotor_contact:
            raise RuntimeError('Rotor-ground contact detected; not a successful landing')
    def stable_for(self,predicate,seconds,timeout,label):
        start=self.sim_time();since=None
        while self.sim_time()-start<timeout:
            self.sim_wait(.05)
            if predicate():
                if since is None:since=self.sim_time()
                if self.sim_time()-since>=seconds:return
            else:since=None
        raise RuntimeError('Timed out: '+label)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True);args=parser.parse_args()
    folder=Path(args.output).resolve();folder.mkdir(parents=True,exist_ok=True)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    def interrupt(*_):raise KeyboardInterrupt()
    signal.signal(signal.SIGINT,interrupt);signal.signal(signal.SIGTERM,interrupt)
    node=LandingScenario(folder);logs=[];done=False;metrics={}
    try:
        for _ in range(30):rclpy.spin_once(node,timeout_sec=.05)
        conflicts=set(node.get_node_names())&{'failure_controller','maneuver','beta_motor_plugin','controller'}
        if conflicts:raise RuntimeError('Close the existing simulation first: '+str(sorted(conflicts)))
        log=(folder/'roslaunch.log').open('w');logs.append(log)
        node.launch=subprocess.Popen(['ros2','launch','firmware','failure_s6.launch.py'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        node.wait(lambda:all(k in node.data for k in ('pose','clock','limits','actual')) and node.sim_time()>0,120,'Gazebo and controller')
        node.wait(node.on_ground,30,'initial ground contact from the contact sensor/bridge')
        node.stable_for(lambda:node.on_ground() and speed(node.data['pose'])<.05,1.,10.,'initial grounded rest')
        ground_height=float(node.data['pose'].pose.position.z)
        config=node.snapshot();original=list(node.data['limits'].data)
        if len(original)!=16 or any(v<=0 for v in original[8:]):raise RuntimeError('Expected initially healthy motor limits')
        topics=TOPICS+list(CONTACT_TOPICS.values())
        (folder/'scenario.json').write_text(json.dumps(dict(scenario='S6',motor_number=4,rotor_index=3,
            effectiveness=0.,tilted_attitude_degrees=[25.,-20.,30.],recovery_attitude_degrees=[0.,0.,30.],
            ground_rest_com_height_m=ground_height,descent_duration_seconds=20.,video=False,topics=topics,
            geometry_note='Original body box and rotor collisions; no separate landing-gear collisions. Contact does not prove gear-specific touchdown.'),indent=2)+'\n')
        qos=folder/'recording_qos.yaml'
        qos.write_text('/omnicopter/motor_limits:\n  reliability: reliable\n  durability: transient_local\n  history: keep_last\n  depth: 1\n/clock:\n  reliability: best_effort\n  durability: volatile\n  history: keep_last\n  depth: 10\n'+''.join(t+':\n  reliability: best_effort\n  durability: volatile\n  history: keep_last\n  depth: 10\n' for t in CONTACT_TOPICS.values()))
        log=(folder/'rosbag.log').open('w');logs.append(log)
        node.bag=subprocess.Popen(['ros2','bag','record','--disable-keyboard-controls','--storage','sqlite3','--output',str(folder/'S6_motor4'),
            '--qos-profile-overrides-path',str(qos),'--topics',*topics],stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        node.wait(lambda:node.events.get_subscription_count()>0 and all(any('rosbag' in i.node_name for i in node.get_subscriptions_info_by_topic(t)) for t in CONTACT_TOPICS.values()),30,'recorder contact subscriptions')
        node.event('recording_ready')
        input('Press [Enter] to run S6: ')
        for _ in range(10):rclpy.spin_once(node,timeout_sec=.01)
        node.event('arm_requested');node.call(ArmDisarm,'/arm_disarm')
        node.wait(lambda:node.data.get('armed') is not None and node.data['armed'].data,5,'armed')
        node.sim_wait(.5)
        node.event('takeoff_requested',height=1.5,duration=8.)
        node.call(LiftOff,'/lift_off',height=1.5,duration=8.)
        node.wait(lambda:'status' in node.data and 'reference' in node.data,5,'feedback/reference')
        node.flight=True;node.monitor_rotors=True;node.sim_wait(8.);node.sim_wait(8.);node.check_hover()
        start=node.data['reference'].pose.position;x=float(start.x);y=float(start.y)
        tilted=dict(x=x,y=y,z=1.5,roll=math.radians(25.),pitch=math.radians(-20.),yaw=math.radians(30.),duration=12.)
        node.event('tilted_attitude_requested',**tilted);node.call(Goto6DPoint,'/goto_6d_point',**tilted)
        node.sim_wait(12.);node.sim_wait(5.)
        def near_tilt():
            p=node.data['pose'];ref=node.data['reference'].pose.orientation;q=p.pose.orientation
            dot=abs(q.x*ref.x+q.y*ref.y+q.z*ref.z+q.w*ref.w)
            angle=math.degrees(2.*math.acos(min(1.,dot)))
            return angle<3. and speed(p)<.1 and omega(p)<.1 and abs(p.pose.position.z-1.5)<.1
        node.stable_for(near_tilt,2.,15.,'settled tilted hover before failure')
        if tilt_deg(node.data['pose'])<15.:raise RuntimeError('Vehicle did not achieve the intended tilted starting attitude')
        node.event('tilted_hover_confirmed',tilt_degrees=tilt_deg(node.data['pose']))
        lower,upper=original[:8],original[8:];lower[3]=upper[3]=0.
        node.event('failure_requested',motor=4,rotor_index=3,upper_N=0.)
        node.call(SetMotorLimits,'/failure_controller/set_motor_limits',lower=lower,upper=upper)
        node.wait(lambda:list(node.data['limits'].data)==lower+upper,5,'failed motor bounds')
        node.event('failure_confirmed',motor=4);node.sim_wait(2.)
        level=dict(x=x,y=y,z=1.5,roll=0.,pitch=0.,yaw=math.radians(30.),duration=10.)
        node.event('horizontal_recovery_requested',**level);node.call(Goto6DPoint,'/goto_6d_point',**level);node.sim_wait(10.)
        def level_stable():
            p=node.data['pose']
            return tilt_deg(p)<3. and speed(p)<.1 and omega(p)<.1 and math.sqrt((p.pose.position.x-x)**2+(p.pose.position.y-y)**2+(p.pose.position.z-1.5)**2)<.1
        node.stable_for(level_stable,2.,20.,'horizontal recovery before descent')
        node.event('horizontal_recovery_confirmed',tilt_degrees=tilt_deg(node.data['pose']))
        # Command just below the measured ground-rest COM height to establish support.
        descent=dict(x=x,y=y,z=max(0.,ground_height-.02),roll=0.,pitch=0.,yaw=math.radians(30.),duration=20.)
        node.event('landing_requested',**descent);node.call(Goto6DPoint,'/goto_6d_point',**descent)
        begin=node.sim_time();first=False;stable_since=None
        while node.sim_time()-begin<40.:
            node.sim_wait(.02);p=node.data['pose']
            if node.on_ground() and not first:
                first=True
                # The last sample is close to impact; retain full pose/contact data for pre-impact metrics.
                metrics.update(touchdown_sim_time=node.sim_time(),observed_contact_vertical_speed_m_s=p.vel.linear.z,
                    observed_contact_speed_m_s=speed(p),touchdown_tilt_degrees=tilt_deg(p),
                    landing_xy_error_m=math.hypot(p.pose.position.x-x,p.pose.position.y-y))
                node.event('touchdown_detected',**metrics)
            quiet=node.on_ground() and tilt_deg(p)<5. and speed(p)<.08 and omega(p)<.1
            if first and quiet:
                if stable_since is None:stable_since=node.sim_time()
                if node.sim_time()-stable_since>=2.:break
            else:stable_since=None
        else:raise RuntimeError('No sustained stable ground contact; refusing to disarm by altitude alone')
        node.event('stable_ground_contact_confirmed');node.flight=False
        node.event('disarm_requested');node.call(ArmDisarm,'/arm_disarm')
        node.wait(lambda:not node.data['armed'].data,5,'disarmed')
        node.flight=False # Status stops while disarmed; contact/rotor monitoring remains active.
        node.sim_wait(5.)
        p=node.data['pose'];metrics.update(final_tilt_degrees=tilt_deg(p),final_speed_m_s=speed(p),rotor_ground_contact_observed=node.ever_rotor_contact,
            final_xy_error_m=math.hypot(p.pose.position.x-x,p.pose.position.y-y),gear_contact_verified=False)
        if not node.on_ground() or tilt_deg(p)>5. or speed(p)>.08:raise RuntimeError('Vehicle did not remain stable after disarming')
        node.event('scenario_complete',**metrics);(folder/'landing_metrics.json').write_text(json.dumps(metrics,indent=2)+'\n')
        for _ in range(20):rclpy.spin_once(node,timeout_sec=.02)
        done=True
    except (Exception,KeyboardInterrupt) as error:
        print('S6 interrupted: '+(str(error) or type(error).__name__),flush=True)
        (folder/'INTERRUPTED.txt').write_text(str(error) or type(error).__name__)
    finally:
        try:stop_process(node.bag,'Recorder')
        finally:
            stop_process(node.launch,'Simulation')
            for log in logs:log.close()
            node.destroy_node()
            if rclpy.ok():rclpy.shutdown()
        if done:(folder/'COMPLETE').write_text('Stable ground contact and post-disarm stability verified in existing collision geometry; landing-gear contact not separately modeled.\n')
        print('Run files: '+str(folder),flush=True)
    return 0 if done else 1
if __name__=='__main__':raise SystemExit(main())
