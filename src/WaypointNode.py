#!/usr/bin/python

from WaypointServer import WaypointServer

import rospy
import sys

if __name__ == "__main__":
	rospy.init_node("waypoint_server")

	server = WaypointServer()
	if not server.Initialize():
		rospy.logerr("Failed to initialize waypoint server node")
		sys.exit(1)
	rospy.spin()

