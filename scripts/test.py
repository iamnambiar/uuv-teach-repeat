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
import uuv_teleop
from uuv_teach import Teach

if __name__ == "__main__":
    rospy.init_node("uuv_teach_and_repeat_node")
    rospy.loginfo("Starting uuv_teach_and_repeat_node")
    rospy.loginfo("Initialising teleop")
    
    uuv_teleop.ROVJoystickTeleop()
    rospy.spin()
    rospy.loginfo("Shutting down uuv_teach_and_repeat_node")