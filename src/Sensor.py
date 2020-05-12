import rospy
import std_msgs.msg
import std_srvs.srv
from crazyflie_isaacs_radsearch.srv import SensorReading

class Sensor(object):
	def __init__(self):
		self._initialized = False

	def initialize(self):
		self._name = rospy.get_name() + "/sensor"

#		self.fake_measurement = fake_measurement

		if not self.RegisterCallbacks():
			rospy.logerr("Error registering callbacks for sensor node")
			return False

		self._initialized = True
		return self._initialized

	def RegisterCallbacks(self):
		self.sensor_srv = rospy.Service("/sensor_reading", SensorReading, self.new_measurement)

		return True

	def new_measurement(self, req):
		m = req.m

		return m
