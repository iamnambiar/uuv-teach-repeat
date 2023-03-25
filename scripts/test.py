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
import uuv_teleop
import uuv_teach
import uuv_repeat

if __name__ == "__main__":
    rospy.init_node("uuv_teach_and_repeat_node")
    rospy.loginfo("Starting uuv_teach_and_repeat_node")

    mode = rospy.get_param('~mode', 'teach')
    rospy.loginfo('{0} is the selected mode.'.format(mode))
    if str.lower(mode) == 'teach':
        rospy.loginfo("Teach mode selected.")
        rospy.loginfo("Initialising teleop...")
        uuv_teleop.ROVJoystickTeleop()
        rospy.loginfo("Initialising continuous teach mode...")
        teach = uuv_teach.ContinuousTeach()
        rospy.spin()

    elif str.lower(mode) == 'repeat':
        repeat = uuv_repeat.Repeat()
        filepath = rospy.get_param('~filepath')
        filename = os.path.join(filepath, 'tracklog.yaml')
        repeat.read_tracklog_from_file(filename)
        repeat.repeat_points()
    
    else:
        rospy.logerr('Invalid mode selected... Please select \'teach\' or \'repeat\' mode')
    rospy.loginfo("Shutting down uuv_teach_and_repeat_node")