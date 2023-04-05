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
from sensor_msgs.msg import CompressedImage

class YOLOInitialiser(object):
    def __init__(self) -> None:
        readImageTopic = rospy.get_param('~read_image_topic', '/camera_image/compressed')
        writeImageTopic = rospy.get_param('~write_image_topic', '/camera/compressed_to_yolo')
        self._readImageSub = rospy.Subscriber(readImageTopic, CompressedImage, self.read_image_callback)
        self._writeImagePub = rospy.Publisher(writeImageTopic, CompressedImage, queue_size=1)
        self._readImage = None
    
    def read_image_callback(self, image):
        self._readImage = image
    
    def main(self):
        rate = rospy.Rate(4)
        while not rospy.is_shutdown():
            if self._readImage is not None:
                self._writeImagePub.publish(self._readImage)
                rate.sleep()


if __name__ == "__main__":
    rospy.init_node('yolo_initialiser_node')
    init = YOLOInitialiser()
    init.main()