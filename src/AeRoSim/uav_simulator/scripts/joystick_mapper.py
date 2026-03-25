#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from mav_msgs.msg import RollPitchYawrateThrust
from geometry_msgs.msg import Vector3
from std_srvs.srv import Empty
from std_msgs.msg import Int16
import math

class Controller():
    def __init__(self, joy_topic):
        # subscribe to the joystick at the end to make sure that all required
        # services were found
        self._buttons = None
        self._axMsg = RollPitchYawrateThrust()
        self._robaxMsg = Vector3()
        self._chooser = 0 # if 1, we talk to robot, if 0 we talk to UAV
        self._axes = [0]*5
        self._Landing = False
        self._Taking_Off = False
        rospy.Subscriber(joy_topic, Joy, self._joyChanged)
        self.pub = rospy.Publisher('uav_setpoint', RollPitchYawrateThrust, queue_size=1)
        self.but = rospy.Publisher('uav_button_msg', Int16, queue_size=1)
        self.rob_pub = rospy.Publisher('/m1002/ax_setpoint', Vector3, queue_size=1)
        self.rob_but = rospy.Publisher('/m1002/button_msg', Int16, queue_size=1)

    def _joyChanged(self, data):
        # First the buttons
        for i in range(0, len(data.buttons)):
            if self._buttons == None or data.buttons[i] != self._buttons[i]:
                if i == 2 and data.buttons[i] == 1:
                    # Button B pressed, Land
                    msg = Int16()
                    if(self._chooser==1):
                        msg.data = 2
                        self.rob_but.publish(msg)
                    else:
                        msg.data = 1
                        self.but.publish(msg)
                    self._Landing = True
                    print('Land')
                if i == 0 and data.buttons[i] == 1:
                    # Button B pressed, Land
                    msg = Int16()
                    msg.data = 30
                    if(self._chooser==1):
                        self.rob_but.publish(msg)
                    else:
                        self.but.publish(msg)
                    #print('Gripper Action')                    
                if i == 3 and data.buttons[i] == 1:
                    # Button Y pressed, emergency stop
                    msg = Int16()
                    if(self._chooser==1):
                        msg.data = 0
                        self.rob_but.publish(msg)
                    else:
                        msg.data = 2
                        self.but.publish(msg)
                    print('Emergency')
                if i == 1 and data.buttons[i] == 1:
                    # Button A pressed, Take Off
                    msg = Int16()
                    msg.data = 3
                    if(self._chooser==1):
                        self.rob_but.publish(msg)
                    else:
                        self.but.publish(msg)                    
                    self._Taking_Off = True
                    print('Take_off')
                if i == 4 and data.buttons[i] == 1:
                    # L1 pressed -> stabilize mode
                    #msg = Int16()
                    #msg.data = 4
                    #self.but.publish(msg)
                    print('Changed Target')
                    if(self._chooser==1):
                        self._chooser=0
                    else:
                        self._chooser=1
                if i == 6 and data.buttons[i] == 1:
                    msg = Int16()
                    msg.data = 5
                    if(self._chooser==1):
                        self.rob_but.publish(msg)
                    else:
                        self.but.publish(msg)
                    print('Position Hold')
            # better have a separate publisher for the above one-shots
        self._buttons = data.buttons

        # Now the axes
        for i in range(0, 4):
            self._axes[i] = data.axes[i]
            # Perhaps a publisher here for mapping the inputs to setpoints
            # _axes[0] = yaw, _axes[1] = z, _axes[2] = y, _axes[3] = x
        self._axMsg.thrust.z = self._axes[1]
        self._axMsg.yaw_rate = -self._axes[0]/10.0
        self._axMsg.pitch = self._axes[3]/2.5
        self._axMsg.roll = -self._axes[2]/2.5
        self._axMsg.thrust.x = self._axes[4]
        self._robaxMsg.x = self._axes[0]/10
        self._robaxMsg.y = self._axes[1]/10
        self._robaxMsg.z = self._axes[2]
        if(self._chooser==1):
            self.rob_pub.publish(self._robaxMsg)
        else:
            self.pub.publish(self._axMsg)

if __name__ == '__main__':
    rospy.init_node('joy_controller', anonymous=True)
    joy_topic = '/joy'
    controller = Controller(joy_topic)
    rospy.spin()

