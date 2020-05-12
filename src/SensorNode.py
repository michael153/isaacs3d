#!/usr/bin/python

from Sensor import Sensor

import rospy
import sys

if __name__ == "__main__":
	rospy.init_node("sensor")

	sensor = Sensor()
	if not sensor.initialize():
		rospy.logerr("Failed to initialize the sensor node")
		sys.exit(1)
	rospy.spin()
