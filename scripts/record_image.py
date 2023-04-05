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
import sys
import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2

class ImageRecorder(object):
    def __init__(self, imageTopic='/image', filepath=''):
        recordTopic = '/record_pressed'
        self.imageTopic = imageTopic
        self.filepath = filepath
        self._image = None
        self._bridge = CvBridge()
        recordPressedSub = rospy.Subscriber(recordTopic, Bool, self.record_callback)
        publishedTopics = rospy.get_published_topics()
        for topic in publishedTopics:
            if topic[0] == imageTopic:
                if topic[1] == "sensor_msgs/Image":
                    imageSub = rospy.Subscriber(imageTopic, Image, self.image_callback)
                elif topic[1] == "sensor_msgs/CompressedImage":
                    imageSub = rospy.Subscriber(imageTopic, CompressedImage, self.image_callback)
                else:
                    rospy.logerr('Topic name is not matching. Expected: {0}. Required: {1}'.format(imageTopic, topic[1]))
    
    def record_callback(self, msg=None):
        filename = os.path.join(self.filepath, 'Image-{0}.jpeg'.format(rospy.get_time()))
        if ((msg is not None) and self._image is not None):
            if msg.data:
                cv2.imwrite(filename, self._image)
    
    def image_callback(self, image):
        if isinstance(image, Image):
            try:
                self._image = self._bridge.imgmsg_to_cv2(image, 'bgr8')
            except CvBridgeError as e:
                rospy.logerr(e)
        elif isinstance(image, CompressedImage):
            try:
                self._image = self._bridge.compressed_imgmsg_to_cv2(image, 'bgr8')
            except CvBridgeError as e:
                rospy.logerr(e)
        else:
            rospy.logwarn("Unsupported image message ({0})".format(type(image)))


if __name__ == "__main__":
    rospy.init_node('record_node', anonymous=True)
    filepath = ''
    imageTopic = '/image'
    if len(sys.argv) > 2:
        imageTopic = sys.argv[1]
    elif len(sys.argv) > 1:
        filepath = sys.argv[1]
    
    ImageRecorder(imageTopic, filepath)
    rospy.spin()