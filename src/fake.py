#!/usr/bin/env python3

import rospy
import numpy as np
from obs_expand.msg import ObsExpandInput

rospy.init_node('fake')
pub = rospy.Publisher('/obs_expand/input', ObsExpandInput, latch=True, queue_size=1)
msg = ObsExpandInput()

""" Required """
mower_gps = [25.6215,121.5595]
heading_degree = 90 # degrees
heading_bias = 1 # meter

""" Optional """
obstacle_radius = 0.5 # meter 半徑
output_path = 'output/a'


msg.mower_gps = mower_gps
msg.heading_degree = heading_degree
msg.heading_bias = heading_bias
msg.obstacle_radius = obstacle_radius
msg.output_path = output_path

rospy.sleep(0.5)
pub.publish(msg)
