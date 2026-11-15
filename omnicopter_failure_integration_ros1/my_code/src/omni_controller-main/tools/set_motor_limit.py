#!/usr/bin/env python3
"""Inject a known thrust-cap fault into BOTH controller and Gazebo (0-based motor)."""
import argparse
import rospy
from std_msgs.msg import Float64MultiArray
from controller.srv import SetMotorLimits

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--motor',type=int,choices=range(8))
parser.add_argument('--fraction',type=float,default=0.,help='fraction of nominal MAX THRUST, not speed')
parser.add_argument('--restore',action='store_true',help='restore all nominal limits')
args=parser.parse_args()
if not args.restore and args.motor is None: parser.error('--motor is required unless --restore')
if not 0<=args.fraction<=1: parser.error('fraction must be between zero and one')
rospy.init_node('set_motor_limit',anonymous=True)
nominal=rospy.get_param('/failure_controller/thrust_max')
minimum=rospy.get_param('/failure_controller/thrust_min')
rospy.wait_for_service('/failure_controller/set_motor_limits',timeout=5)
if args.restore:
    lower,upper=minimum,nominal
else:
    current=rospy.wait_for_message('/omnicopter/motor_limits',Float64MultiArray,timeout=5)
    if len(current.data)!=16: raise RuntimeError('Invalid current limits')
    lower=list(current.data[:8]);upper=list(current.data[8:])
    upper[args.motor]=nominal[args.motor]*args.fraction
    lower[args.motor]=min(lower[args.motor],upper[args.motor])
result=rospy.ServiceProxy('/failure_controller/set_motor_limits',SetMotorLimits)(lower,upper)
if not result.success: raise RuntimeError(result.message)
print(result.message)
print('Thrust upper bounds [N]:',upper)
