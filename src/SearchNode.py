#!/usr/bin/python

from Search import Search

import rospy
import sys

if __name__ == "__main__":
	rospy.init_node("search")

	s = Search()
	if not s.Initialize(6,6,4,4,4,4,1,0,300,800,1000,3,True):
		rospy.logerr("Failed to initialize")
		sys.exit(1)

	rospy.spin()
