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
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from uuv_teach_repeat.msg import TrackPoint, TrackLog
import yaml

class Teach(object):
    def __init__(self):
        isRecordPressedTopic = "/record_pressed"
        robotPoseTopic = "/pose"
        leftCameraImageTopic = "/left/image"
        rightCameraImageTopic = "/right/image"
        self._frameId = 'world'
        self._robotPose = None
        self._leftImage = None
        self._rightImage = None
        self._trackLog = TrackLog()
        self._recordPressedSub = rospy.Subscriber(isRecordPressedTopic, Bool, self.record_pressed_callback)
        robotPoseSub = rospy.Subscriber(robotPoseTopic, Odometry, self.robot_pose_callback)
        leftImageSub = rospy.Subscriber(leftCameraImageTopic, Image, self.left_image_callback)
        rightImageSub = rospy.Subscriber(rightCameraImageTopic, Image, self.right_image_callback)
    
    def robot_pose_callback(self, poseData:Odometry):
        if poseData is not None:
            self._robotPose = poseData.pose.pose

    def get_trackpoint(self, includeAll=True):
        trackPoint = TrackPoint()
        trackPoint.header.frame_id = self._frameId
        trackPoint.header.stamp = rospy.Time.now()
        trackPoint.pose = self._robotPose
        trackPoint.isFixed = includeAll
        if includeAll:
            trackPoint.leftImage = self._leftImage
            trackPoint.rightImage = self._rightImage
        return trackPoint

    def record_pressed_callback(self, msg=None):
        if ((msg is not None) and self._robotPose is not None):
            if msg.data:
                trackPoint = self.get_trackpoint()
                self._trackLog.trackpoints.append(trackPoint)
    
    def left_image_callback(self, image):
        if image is not None:
            self._leftImage = image

    def right_image_callback(self, image):
        if image is not None:
            self._rightImage = image
    
    def exportWayPosesToFile(self, filePath):
        try:
            data = dict(trackpoints=list())
            for tp in self._trackLog.trackpoints:
                positionDict = dict(position=[float(tp.pose.position.x), float(tp.pose.position.y), float(tp.pose.position.z)])
                orientationDict = dict(orientation=[float(tp.pose.orientation.x), float(tp.pose.orientation.y), float(tp.pose.orientation.z), float(tp.pose.orientation.w)])
                trackPointDict = dict(pose=[positionDict, orientationDict], isFixed=tp.isFixed)
                data["trackpoints"].append(trackPointDict)
            with open(filePath, 'w') as wp_file:
                yaml.dump(data, wp_file, default_flow_style=False)
            
            rospy.loginfo("File dumped")
            return True
        except Exception as e:
            rospy.logerr("Error occured while exporting recorded poses to file, message = {}".format(e))
            return False

class ContinuousTeach(Teach):
    def __init__(self):
        super().__init__()
        self._continuousModeOn = False
        self._lastRecordedTime = rospy.Time.now()
        startPressedTopic = '/start_pressed'
        self._startRecordingPressedSub = rospy.Subscriber(startPressedTopic, Bool, self.start_recording_callback)
    
    def start_recording_callback(self, msg=None):
        if ((msg is not None) and self._robotPose is not None):
            if msg.data:
                if not self._continuousModeOn:
                    rospy.loginfo("Start Recording")
                else:
                    rospy.loginfo("Stop Recording")
                self._continuousModeOn = not self._continuousModeOn
    
    def robot_pose_callback(self, poseData: Odometry):
        super().robot_pose_callback(poseData)
        if self._continuousModeOn:
            duration = rospy.Time.now() - self._lastRecordedTime
            if (duration.to_sec() > 0.5):
                trackPoint = self.get_trackpoint(includeAll=False)
                self._trackLog.trackpoints.append(trackPoint)  
                self._lastRecordedTime = rospy.Time.now()
                