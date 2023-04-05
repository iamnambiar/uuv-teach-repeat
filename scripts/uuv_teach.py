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

import os
import rospy 
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from uuv_teach_repeat.msg import TrackPoint, TrackLog
from detection_msgs.msg import BoundingBoxes
import yaml

class Teach(object):
    def __init__(self):
        isRecordPressedTopic = rospy.get_param('~record_topic', '/record_pressed')
        isSavePressedTopic = rospy.get_param('~save_topic', '/save_pressed')
        robotPoseTopic = rospy.get_param('~pose_topic', '/pose')
        objDetectOutputTopic = rospy.get_param('~bounding_boxes_topic', '/yolov5/detections')
        self._frameId = 'world'
        self._robotPose = None
        self._boundingBoxes = None
        self._trackLog = TrackLog()
        self._recordPressedSub = rospy.Subscriber(isRecordPressedTopic, Bool, self.record_pressed_callback)
        self._savePressedSub = rospy.Subscriber(isSavePressedTopic, Bool, self.save_pressed_callback)
        robotPoseSub = rospy.Subscriber(robotPoseTopic, Odometry, self.robot_pose_callback)
        boundingBoxesSub = rospy.Subscriber(objDetectOutputTopic, BoundingBoxes, self.bounding_boxes_callback)
    
    def robot_pose_callback(self, poseData):
        if poseData is not None:
            self._robotPose = poseData.pose.pose
    
    def bounding_boxes_callback(self, bb):
        if bb is not None:
            self._boundingBoxes = bb

    def get_trackpoint(self, includeAll=True):
        trackPoint = TrackPoint()
        trackPoint.header.frame_id = self._frameId
        trackPoint.header.stamp = rospy.Time.now()
        trackPoint.pose = self._robotPose
        trackPoint.isRecorded = includeAll
        if includeAll:
            trackPoint.boundingBoxes = self._boundingBoxes
        return trackPoint

    def record_pressed_callback(self, msg=None):
        if ((msg is not None) and self._robotPose is not None):
            if msg.data:
                trackPoint = self.get_trackpoint()
                self._trackLog.trackpoints.append(trackPoint)
                # rospy.loginfo(trackPoint)
    
    def save_pressed_callback(self, msg=None):
        if msg is not None:
            if msg.data:
                filepath = rospy.get_param('~filepath')
                if os.path.exists(filepath):
                    filename = os.path.join(filepath, 'tracklog.yaml')
                    self.export_tracklog_to_file(filename)
                else:
                    rospy.logerr('The path \'{0}\' doesnot exist')
    
    def export_tracklog_to_file(self, fileName):
        try:
            data = dict(header_frame=self._frameId, tracklog=list())
            for tp in self._trackLog.trackpoints:
                positionDict = dict(position=[float(tp.pose.position.x), float(tp.pose.position.y), float(tp.pose.position.z)])
                orientationDict = dict(orientation=[float(tp.pose.orientation.x), float(tp.pose.orientation.y), float(tp.pose.orientation.z), float(tp.pose.orientation.w)])
                trackPointDict = dict(pose=[positionDict, orientationDict], isRecorded=tp.isRecorded, boundingBoxes=list())
                if (tp.boundingBoxes is not None) and (tp.boundingBoxes.bounding_boxes is not None):
                    for bb in tp.boundingBoxes.bounding_boxes:
                        bb_dict = dict(Class=bb.Class, probability=bb.probability, xmin=bb.xmin, ymin=bb.ymin, xmax=bb.xmax, ymax=bb.ymax)
                        trackPointDict['boundingBoxes'].append(bb_dict)
                data["tracklog"].append(trackPointDict)
            with open(fileName, 'w') as wp_file:
                yaml.dump(data, wp_file, default_flow_style=False)
            
            rospy.loginfo("File dumped to {0}".format(fileName))
            return True
        except Exception as e:
            rospy.logerr("Error occured while exporting recorded poses to file, message = {}".format(e))
            return False

class ContinuousTeach(Teach):
    def __init__(self):
        super().__init__()
        self._continuousModeOn = False
        self._lastRecordedTime = rospy.Time.now()
        startPressedTopic = rospy.get_param('~start_topic', '/start_pressed')
        self._startRecordingPressedSub = rospy.Subscriber(startPressedTopic, Bool, self.start_recording_callback)
    
    def start_recording_callback(self, msg=None):
        if ((msg is not None) and self._robotPose is not None):
            if msg.data:
                if not self._continuousModeOn:
                    rospy.loginfo("Start Recording (ON)")
                else:
                    rospy.loginfo("Stop Recording (OFF)")
                self._continuousModeOn = not self._continuousModeOn
    
    def robot_pose_callback(self, poseData: Odometry):
        super().robot_pose_callback(poseData)
        if self._continuousModeOn:
            duration = rospy.Time.now() - self._lastRecordedTime
            if (duration.to_sec() > 0.5):
                trackPoint = self.get_trackpoint(includeAll=False)
                self._trackLog.trackpoints.append(trackPoint)  
                self._lastRecordedTime = rospy.Time.now()
                