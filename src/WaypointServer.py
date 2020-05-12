from crazyflie_msgs.msg import PositionVelocityStateStamped, Waypoints

import rospy
import std_srvs.srv

import numpy as np

class WaypointServer(object):
	def __init__(self):
		self._initialized = False

	def Initialize(self):
		self._name = rospy.get_name() + "/waypoint_server"

		if not self.LoadParameters():
			rospy.logerr("Error loading parameters")
			return False

		if not self.RegisterCallbacks():
			rospy.logerr("Error registerin callbacks")
			return False

		self._refs = []

		self._current_idx = 0

		self._initialized = True

		return True

	def LoadParameters(self):
#		print(rospy.get_param_names())
		if not rospy.has_param("/waypoint_server/topics/ref"):
			return False
		self._ref_topic = rospy.get_param("/waypoint_server/topics/ref")
		return True

	def RegisterCallbacks(self):
		self._ref_pub = rospy.Publisher(self._ref_topic, PositionVelocityStateStamped, queue_size=1)

		self._move_srv = rospy.Service("/move", std_srvs.srv.Empty, self.MoveCallback)

		rospy.Subscriber('waypoints', Waypoints, self.UpdateCallback)

		return True

	def MoveCallback(self, req):
		rospy.loginfo("%s: Moving to reference point #%d", self._name, self._current_idx)

		msg = PositionVelocityStateStamped()
		msg.header.stamp = rospy.Time.now()
		msg.state.x = self._refs[self._current_idx][0]
		msg.state.y = self._refs[self._current_idx][1]
		msg.state.z = self._refs[self._current_idx][2]
		msg.state.x_dot = 0.0
		msg.state.y_dot = 0.0
		msg.state.z_dot = 0.0

		self._ref_pub.publish(msg)
		self._current_idx = (self._current_idx + 1) % len(self._refs)
		return []

	def UpdateCallback(self, req):
		if len(req.x) < 1:
			rospy.logerr("No waypoints passed")
		if not (len(req.x) == len(req.y) == len(req.z)):
			rospy.logerr("Coordinate arrays are different lengths")

		for i in range(len(req.x)):
			pos = np.array([req.x[i], req.y[i], req.z[i]])
			self._refs.append(pos)
		self._current_idx = 0
	
