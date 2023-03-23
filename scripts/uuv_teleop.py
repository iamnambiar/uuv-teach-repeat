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

class ROVTeleop(object):
    '''
    Base class for teleop
    '''
    def __init__(self):
        twistTopic = 'cmd_vel'
        recordTopic = '/record_pressed'
        startTopic = '/start_pressed'

        if (rospy.has_param("twist_topic")):
            twistTopic = rospy.get_param("twist_topic")
        
        if (rospy.has_param("record_topic")):
            recordTopic = rospy.get_param("record_topic")

        if (rospy.has_param("start_topic")):
            startTopic = rospy.get_param("start_topic")

        self._twistPub = rospy.Publisher(twistTopic, Twist, queue_size=1)
        self._recordPressedPub = rospy.Publisher(recordTopic, Bool, queue_size=1)
        self._startPressedPub = rospy.Publisher(startTopic, Bool, queue_size=1)

class ROVJoystickTeleop(ROVTeleop):
    '''
    This class is used to control the robot / rov using Joystick.
    '''
    def __init__(self):
        super().__init__()
        self._axisKeys = { "x":1, "y":0, "z":7, "roll":-1, "pitch":3, "yaw":2 }
        self._axisGains = { "x":1.0, "y":1.0, "z":1.0, "roll":1.0, "pitch":1.0, "yaw":1.0 }
        self._buttonKeys = { "record":0, "start":11 }
        self._isRecordON = False
        self._isStartON = False
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
        if joy is not None:
            isRecordPressed = bool(joy.buttons[self._buttonKeys["record"]] > 0)
            if (isRecordPressed and (self._isRecordON != isRecordPressed)):
                recordStateChanged = True
            
            isStartPressed = bool(joy.buttons[self._buttonKeys["start"]] > 0)
            if (isStartPressed and (self._isStartON != isStartPressed)):
                startStateChanged = True
            
            self._isRecordON = isRecordPressed
            self._isStartON = isStartPressed
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
        
        return twistMsg, [recordStateChanged, startStateChanged]
    
    def joy_callback(self, joy:Joy):
        try:
            twistMsg, others = self.parse_joystick_command(joy)
            self._twistPub.publish(twistMsg)
            self._recordPressedPub.publish(Bool(others[0]))
            self._startPressedPub.publish(Bool(others[1]))
        except Exception as e:
            rospy.logerr('Unable to parse joystick input. Please check if the joy_id corresponds to the joystick being used. message={}'.format(e))

class ROVKeyboardTeleop(ROVTeleop):
    '''
    This class is used to control the robot using keyboard teleop.
    '''
    def __init__(self):
        super().__init__()

