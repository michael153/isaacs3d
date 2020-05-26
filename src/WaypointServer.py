from crazyflie_msgs.msg import PositionVelocityStateStamped, Waypoints, PositionVelocityYawStateStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA

import rospy
import std_srvs.srv

import numpy as np
import time

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

		self._waiting = False

		return True

	def LoadParameters(self):
#		print(rospy.get_param_names())
		if not rospy.has_param("/waypoint_server/topics/ref"):
			return False
		self._ref_topic = rospy.get_param("/waypoint_server/topics/ref")
		return True

	def RegisterCallbacks(self):
		self._ref_pub = rospy.Publisher(self._ref_topic, PositionVelocityStateStamped, queue_size=1)

		self._viz_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

		self._move_srv = rospy.Service("/move", std_srvs.srv.Empty, self.MoveCallback)

		rospy.Subscriber('waypoints', Waypoints, self.UpdateCallback)

		rospy.Subscriber('state', PositionVelocityYawStateStamped, self.StateCallback)

		return True

	def StateCallback(self, req):
		if self._waiting:
			if np.abs(req.state.x_dot) < 0.05 and np.abs(req.state.y_dot) < 0.05 and np.abs(req.state.z_dot) < 0.05:
				idx = (self._current_idx - 1) % len(self._refs)
				if np.abs(req.state.x - self._refs[idx][0]) < 0.05 and np.abs(req.state.y - self._refs[idx][1]) < 0.05 and np.abs(req.state.z - self._refs[idx][2]) < 0.05:
 					self._waiting = False

	def MoveCallback(self, req):
		rospy.loginfo("%s: Moving to reference point #%d", self._name, self._current_idx)

		pos = self._refs[self._current_idx]
		marker = Marker()
		marker.id = self._current_idx + 2000
		marker.ns = "waypoint"
		marker.action = 0
		marker.type = Marker.SPHERE
		marker.frame_locked = True
		marker.header = Header(frame_id="world", stamp = rospy.Time.now())
		marker.lifetime = rospy.Duration(0)
		marker.pose = Pose(Point(pos[0],pos[1],pos[2]), Quaternion(0,0,0,1))
		marker.scale = Vector3(0.3,0.3,0.3)
		marker.color = ColorRGBA(0.0,1.0,0.0,1.0)
		self._viz_pub.publish(marker)

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
		self._waiting = True
		while self._waiting:
			pass
		marker.action = 2
		self._viz_pub.publish(marker)
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
	
