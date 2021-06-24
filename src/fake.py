#!/usr/bin/env python3

import rospy
import numpy as np
import os
from library.get_path import get_path
from path_planning.msg import PathPlanningInput
# %%%%%%%%%%%%
dir_path = os.path.dirname(os.path.realpath(__file__))+'/'
# %%%%%%%%%%%%
rospy.init_node('fake')
pub = rospy.Publisher('/path_planning/input', PathPlanningInput, latch=True, queue_size=1)
msg = PathPlanningInput()
# 16 503 504
path = get_path(123)
msg.pos_path = dir_path + path['BD_path']
msg.retry_path = dir_path + path['retry_path']
msg.output_path = dir_path + path['output_path']
msg.neg_path = [ dir_path+item for item in path['obs_path_list']]
msg.start_gps = path['start_gps']
rospy.sleep(0.5)
pub.publish(msg)
