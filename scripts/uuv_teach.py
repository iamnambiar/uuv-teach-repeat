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
from geometry_msgs.msg import Pose
import yaml

class Teach(object):
    def __init__(self):
        isRecordPressedTopic = "/record_pressed"
        robotPoseTopic = "/rexrov/pose_gt"
        self._robotPose = None
        self._wayPoseSet = WayPoseSet()
        self._recordPressedSub = rospy.Subscriber(isRecordPressedTopic, Bool, self.record_pressed_callback)
        self._robotPoseSub = rospy.Subscriber(robotPoseTopic, Odometry, self.robot_pose_callback)
    
    def robot_pose_callback(self, poseData:Odometry):
        if poseData is not None:
            self._robotPose = poseData.pose.pose

    def record_pressed_callback(self, msg=None):
        if ((msg is not None) and self._robotPose is not None):
            if msg.data:
                rospy.loginfo("Record Pressed")
                # Get the current pose data and change it into waypoints. In the end save it to a file.
                wayPose = WayPose()
                wayPose.pose = self._robotPose
                self._wayPoseSet.wayPoseList.append(wayPose)
                rospy.loginfo("Pose Saved")
    
    def exportWayPosesToFile(self, filePath):
        try:
            data = dict(wayposes=list())
            for wp in self._wayPoseSet.wayPoseList:
                positionDict = dict(position=[float(wp.pose.position.x), float(wp.pose.position.y), float(wp.pose.position.z)])
                orientationDict = dict(orientation=[float(wp.pose.orientation.x), float(wp.pose.orientation.y), float(wp.pose.orientation.z)])
                individualPoseDict = dict(pose=[positionDict, orientationDict])
                data["wayposes"].append(individualPoseDict)
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
                wayPose = WayPose()
                wayPose.pose = self._robotPose
                self._wayPoseSet.wayPoseList.append(wayPose)  
                self._lastRecordedTime = rospy.Time.now()
                # rospy.loginfo('Recorded: {0} seconds'.format(duration.to_sec()))


if __name__ == "__main__":
    rospy.init_node("teach_node", anonymous=True)
    rospy.loginfo('Starting teach_node')

    teach = ContinuousTeach()

    rospy.spin()
    teach.exportWayPosesToFile('/home/iamnambiar/poseConfig.yaml')
    rospy.loginfo('Shutting down teach_node')