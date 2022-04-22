#!/usr/bin/env python

from get_waypoints import GetPaths
import rospy

rospy.init_node("main")
get_paths_psm1 = GetPaths('psm1')

while not rospy.is_shutdown():
    print(get_paths_psm1.get_needle_path())
