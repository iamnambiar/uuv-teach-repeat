#!/usr/bin/env python3
# Copyright 2023 iamnambiar
# All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

import sys
from select import select
import termios
import tty

class ROVTeleop(object):
    '''
    Base class for teleop
    '''
    def __init__(self):
        twistTopic = rospy.get_param('~twist_topic', '/cmd_vel')
        recordTopic = rospy.get_param('~record_topic', '/record_pressed')
        startTopic = rospy.get_param('~start_topic', '/start_pressed')
        saveTopic = rospy.get_param('~save_topic', '/save_pressed')

        self._twistPub = rospy.Publisher(twistTopic, Twist, queue_size=1)
        self._recordPressedPub = rospy.Publisher(recordTopic, Bool, queue_size=1)
        self._startPressedPub = rospy.Publisher(startTopic, Bool, queue_size=1)
        self._savePressedPub = rospy.Publisher(saveTopic, Bool, queue_size=1)

class ROVJoystickTeleop(ROVTeleop):
    '''
    This class is used to control the robot / rov using Joystick.
    '''
    def __init__(self):
        super().__init__()
        self._axisKeys = { "x":1, "y":0, "z":7, "roll":-1, "pitch":3, "yaw":2 }
        self._axisGains = { "x":0.5, "y":0.5, "z":0.5, "roll":0.5, "pitch":0.5, "yaw":0.5 }
        self._buttonKeys = { "record":0, "start":4, "save":11 }
        self._isRecordPressed = False
        self._isStartPressed = False
        self._isSavePressed = False
        self._deadzone = 0.2        # Deadzone is used to remove the unwanted motion created because of joystick error.

        for key, val in self._axisKeys.items():
            if rospy.has_param('axis_{0}'.format(key)):
                self._axisKeys[key] = int(rospy.get_param('axis_{0}'.format(key)))
            if rospy.has_param('gain_{0}'.format(key)):
                self._axisKeys[key] = int(rospy.get_param('gain_{0}'.format(key)))
        
        # Subscriber to read joystick command
        self._joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)

    def parse_joystick_command(self, joy=None):
        twistMsg = Twist()
        recordStateChanged = False
        startStateChanged = False
        saveStateChanged = False
        if joy is not None:
            isRecordPressed = bool(joy.buttons[self._buttonKeys["record"]] > 0)
            if (isRecordPressed and (self._isRecordPressed != isRecordPressed)):
                recordStateChanged = True
            
            isStartPressed = bool(joy.buttons[self._buttonKeys["start"]] > 0)
            if (isStartPressed and (self._isStartPressed != isStartPressed)):
                startStateChanged = True
            
            isSavePressed = bool(joy.buttons[self._buttonKeys["save"]] > 0)
            if (isSavePressed and (self._isSavePressed != isSavePressed)):
                saveStateChanged = True
            
            self._isRecordPressed = isRecordPressed
            self._isStartPressed = isStartPressed
            self._isSavePressed = isSavePressed
            if (self._axisKeys["x"] >= 0 and (abs(joy.axes[self._axisKeys["x"]]) > self._deadzone)):
                twistMsg.linear.x = (joy.axes[self._axisKeys["x"]] * self._axisGains["x"])

            if (self._axisKeys["y"] >= 0 and (abs(joy.axes[self._axisKeys["y"]]) > self._deadzone)):
                twistMsg.linear.y = (joy.axes[self._axisKeys["y"]] * self._axisGains["y"])

            if (self._axisKeys["z"] >= 0 and (abs(joy.axes[self._axisKeys["z"]]) > self._deadzone)):
                twistMsg.linear.z = (joy.axes[self._axisKeys["z"]] * self._axisGains["z"])

            if (self._axisKeys["roll"] >= 0 and (abs(joy.axes[self._axisKeys["roll"]]) > self._deadzone)):
                twistMsg.angular.x = (joy.axes[self._axisKeys["roll"]] * self._axisGains["roll"])
            
            if (self._axisKeys["pitch"] >= 0 and (abs(joy.axes[self._axisKeys["pitch"]]) > self._deadzone)):
                twistMsg.angular.y = (joy.axes[self._axisKeys["pitch"]] * self._axisGains["pitch"])
            
            if (self._axisKeys["yaw"] >= 0 and (abs(joy.axes[self._axisKeys["yaw"]]) > self._deadzone)):
                twistMsg.angular.z = (joy.axes[self._axisKeys["yaw"]] * self._axisGains["yaw"])
        
        return twistMsg, [recordStateChanged, startStateChanged, saveStateChanged]
    
    def joy_callback(self, joy:Joy):
        try:
            twistMsg, others = self.parse_joystick_command(joy)
            self._twistPub.publish(twistMsg)
            self._recordPressedPub.publish(Bool(others[0]))
            self._startPressedPub.publish(Bool(others[1]))
            self._savePressedPub.publish(Bool(others[2]))
        except Exception as e:
            rospy.logerr('Unable to parse joystick input. Please check if the joy_id corresponds to the joystick being used. message={}'.format(e))

class ROVKeyboardTeleop(ROVTeleop):
    '''
    This class is used to control the robot using keyboard teleop.
    '''
    def __init__(self):
        super().__init__()
        self.settings = termios.tcgetattr(sys.stdin)
        self._axisKeys = { "x+":'w', "x-":'s', "z+":'i', "z-":'k', "az+":'a', "az-":'d' }
        self._axisGains = { "x+":0.5, "x-":0.5, "z+":0.5, "z-":0.5, "az+":0.5, "az-":0.5 }
        # Ctrl + Q = Start
        # Ctrl + R = Record
        # Ctrl + S = Save
        self._buttonKeys = { "record":'\x12', "start":'\x11', "save":'\x13' }
        self._isRecordPressed = False
        self._isStartPressed = False
        self._isSavePressed = False

        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            rate.sleep()
            self.keyboard_callback()
    
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def parse_keyboard(self):
        twistMsg = Twist()
        recordStateChanged = False
        startStateChanged = False
        saveStateChanged = False

        key_pressed = self.get_key()
        if key_pressed != '':
            if (key_pressed == self._axisKeys['x+']):
                twistMsg.linear.x = self._axisGains['x+']
            elif (key_pressed == self._axisKeys['x-']):
                twistMsg.linear.x = -1 * self._axisGains['x-']
            elif (key_pressed == self._axisKeys['z+']):
                twistMsg.linear.z = self._axisGains['z+']
            elif (key_pressed == self._axisKeys['z-']):
                twistMsg.linear.z = -1 * self._axisGains['z-']
            elif (key_pressed == self._axisKeys['az+']):
                twistMsg.angular.z = self._axisGains['az+']
            elif (key_pressed == self._axisKeys['az-']):
                twistMsg.angular.z = -1 * self._axisGains['az-']
            
            if (key_pressed == self._buttonKeys['record']):
                recordStateChanged = True
            elif (key_pressed == self._buttonKeys['start']):
                startStateChanged = True
            elif (key_pressed == self._buttonKeys['save']):
                saveStateChanged = True

        if (key_pressed == '\x03'):
            rospy.loginfo('Keyboard Interrupt Pressed')
            exit(-1)
        
        return twistMsg, [recordStateChanged, startStateChanged, saveStateChanged] 
    
    def keyboard_callback(self):
        try:
            twistMsg, others = self.parse_keyboard()
            self._twistPub.publish(twistMsg)
            self._recordPressedPub.publish(Bool(others[0]))
            self._startPressedPub.publish(Bool(others[1]))
            self._savePressedPub.publish(Bool(others[2]))
        except Exception as e:
            rospy.logerr('Unable to parse keyboard input. message={}'.format(e))


if __name__ == "__main__":
    rospy.init_node("teleop_node")
    rospy.logwarn("Initialising teleop_node")

    teleopType = rospy.get_param('~teleop_type', 'joystick')
    if teleopType == 'joystick':
        ROVJoystickTeleop()
        rospy.spin()
    elif teleopType == 'keyboard':
        ROVKeyboardTeleop()
    else:
        rospy.logerr('Unsupported teleop_type: {0}. Only \'{1}\' and \'{2}\' are supported.'.format(teleopType, 'joystick', 'keyboard'))
    
    rospy.logwarn("Shutting down teleop_node")