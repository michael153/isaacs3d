from AdaSearch2D import Adasearch2D
from AdaSearch2_5D import Adasearch2_5D
from GridMap2D import GridMap2D
from GridMap2_5D import GridMap2_5D
from Drone import Drone
from Flight import Flight
import sys
import math
import argparse
import time
sys.path.insert(1, './Utils/')
from DroneStatus import DroneStatus
from DroneType import DroneType
from Position import Position
from Velocity import Velocity
import numpy as np
import rospkg
import rospy
import std_srvs.srv
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from crazyflie_msgs.msg import Waypoints
from crazyflie_isaacs_radsearch.srv import SensorReading

class Search:
	def __init__(self):
		self._initialized = False

	def Initialize(self, gridx, gridy, gridz, resx, resy, resz, sources, back_min, back_max, rad_min, rad_max, dimensions, verbose, emissions=None): 
		self.name = rospy.get_name() + "/search_node"
		self.tau = 1
		self.background_min = back_min
		self.background_max = back_max
		self.rad_min = rad_min
		self.rad_max = rad_max
		self.verbose = verbose 
		self.solved = False
		rospack = rospkg.RosPack()
		if dimensions == 2:
			self.grid_map = GridMap2D(gridx, gridy, resx, resy, rad_max, sources)
			self.search = Adasearch2D(0, sources, back_min, back_max, rad_min, rad_max, gridx, gridy, resx, resy, self.grid_map)
		elif dimensions == 3:
			pkg_path = rospack.get_path('crazyflie_isaacs_radsearch')
#			print(pkg_path)
			self.grid_map = GridMap2_5D(sources, pkg_path+"/src/test_env.txt")
			self.search = Adasearch2_5D(0, sources, back_min, back_max, rad_min, rad_max, self.grid_map)
		else:
			raise Exception("Invalid number of dimensions. Should be 2 or 3.")
		self.drone = Drone(DroneType.Matrice100, Position(), Velocity())
		true_emissions_grid = emissions
		if true_emissions_grid == None:
			true_emissions_grid = self.grid_map.generate_emissions_grid(self.background_min, self.background_max, self.rad_min, self.rad_max)
		else:
			self.grid_map.set_emissions_grid(true_emissions_grid)
		self.raster_path = self.grid_map.get_raster_path()

		if not self.load_parameters():
			rospy.logerr("Error loading parameters")
			return False

		if not self.register_callbacks():
			rospy.logerr("Error Registering callbacks")
			return False

		self._initialized = True
		self.pub = rospy.Publisher('waypoints', Waypoints, queue_size=1)
		return self._initialized

	def load_parameters(self):
		if not rospy.has_param("~topics/wp"):
			return False
		self._wp_topic = rospy.get_param("~topics/wp")
		if not rospy.has_param("~services/move"):
			return False
		self._move_service = rospy.get_param("~services/move")
		if not rospy.has_param("~services/sensor_reading"):
			return False
		self._sensor_reading = rospy.get_param("~services/sensor_reading")
		if not rospy.has_param("~services/takeoff"):
			return False
		self._takeoff_service = rospy.get_param("~services/takeoff")
		return True

	def register_callbacks(self):
		self.pub = rospy.Publisher(self._wp_topic, Waypoints, queue_size=1)

		self.viz_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

		self.start_srv = rospy.Service("/start", std_srvs.srv.Empty, self.start_callback)
		return True

	def start_callback(self, req):
		print("Sending marker")
		time.sleep(1)
		containers = self.grid_map.get_containers()
		i=0
#		print(self.grid_map
		for container in containers:
			pos, size = container
			marker = Marker()
			marker.type = Marker.CUBE
			marker.id = i
			marker.ns = "container"
			marker.frame_locked = True
			marker.action = 0
			marker.header = Header(frame_id="world",stamp=rospy.Time.now())
			marker.lifetime = rospy.Duration(0)
			marker.pose = Pose(Point(pos[0],pos[1],pos[2]), Quaternion(0,0,0,1))
			marker.scale = Vector3(size[0]+0.1,size[1]+0.1,size[2]+0.1)
			marker.color = ColorRGBA(0.5,0.5,0.5,0.5)
#			self.viz_pub.publish(marker)
			#self.viz_pub.publish(marker_list)
			i+=1
			time.sleep(1)
		time.sleep(3)
		rospy.wait_for_service(self._takeoff_service)
		self._takeoff = rospy.ServiceProxy(self._takeoff_service, std_srvs.srv.Empty)
		try:
			resp = self._takeoff()
		except rospy.ServiceExeception as e:
			rospy.logerr("Couldn't takeoff")
		time.sleep(3)
		iteration = 0
		while not self.solved:
			emitters = self.run_iteration(iteration)
			iteration+=1
		print(emitters)
		return []

	def run_iteration(self, iteration):
		print("Running iteration "+ str(iteration))
		if self.grid_map.solved():
			emitters = []
			for emitter in self.grid_map.get_emitters():
				emitters.append(emitter.get_pos())
				if self.verbose:
					print(emitter.get_pos())
			self.solved = True
			return emitters
		else:
			i=0
			print(self.grid_map.possible_sources)
			for cell in self.grid_map.all_cells():
				cb = cell.get_confidence_bounds()
				print(str(cb.get_LB()), str(cb.get_UB()))
			voxel_markers = []
			for container in self.grid_map.containers:
				cells = container.get_cells()
				dist_per = container.voxel_size
				cell_pos = []
				cell_col = []
				for cell in cells:
					p = cell.get_pos()
					cell_pos.append(Point(p[0],p[1],p[2]))
					if cell in  self.grid_map.possible_sources:
						cell_col.append(ColorRGBA(1.0,0.0,0.0,1.0))
					else:
						cell_col.append(ColorRGBA(0.5,0.5,0.5,1.0))
				marker_list = Marker()
				marker_list.type = Marker.CUBE_LIST
				marker_list.id = i
				marker_list.ns = "container"
				marker_list.frame_locked = True
				marker_list.action = 0
				marker_list.header = Header(frame_id="world", stamp=rospy.Time.now())
				marker_list.lifetime = rospy.Duration(0)
				marker_list.pose = Pose(Point(0,0,0), Quaternion(0,0,0,1))
				marker_list.scale = Vector3(dist_per[0], dist_per[1], dist_per[2])
				marker_list.points = cell_pos
				marker_list.colors = cell_col
				voxel_markers.append(marker_list)
				self.viz_pub.publish(marker_list)
				i+=1
			last = self.raster_path[0]
			self.raster_path = self.grid_map.get_raster_path()
			intermediate_path = []
			if last != self.raster_path[0]:
				intermediate_path = self.grid_map.get_no_collision_path(last, self.raster_path[0])
			else:
				pass
			sensing_config = self.grid_map.get_sensing_config(self.raster_path, self.tau, iteration)
			#Simulate flying a round of data collection
			sensing_config = intermediate_path.extend(sensing_config)
			x = []
			y = []
			z = []
			for config in sensing_config:
				x.append(config[0][0])
				y.append(config[0][1])
				z.append(config[0][2])
			msg = Waypoints()
			msg.x = x
			msg.y = y
			msg.z = z
			self.pub.publish(msg)
			time.sleep(2)
			self.fly_round(sensing_config, self.search.new_measurement)
			if self.verbose:
				print("Iteration complete")
			for m in voxel_markers:
				m.action = 2
				self.viz_pub.publish(m)

	"""
	Runs the main algorithm
	"""
	def start(self, emissions=None):
		#Get the "true" emissions grid and raster_path
		true_emissions_grid = emissions
		if true_emissions_grid == None:
			true_emissions_grid = self.grid_map.generate_emissions_grid(self.background_min, self.background_max, self.rad_min, self.rad_max)
		else:
			self.grid_map.set_emissions_grid(true_emissions_grid)
		self.raster_path = self.grid_map.get_raster_path()

		'''
		containers = self.grid_map.containers
		observation_points = []
		for container in container:
			for cell in container.cells:
				obs = cell.get_observation_point()
				observation_points.append(obs)
		raster_path = TSP.getPath(self.grid_map, observation_points)
		'''

		iteration = 0
		if self.verbose:
			print(true_emissions_grid)
		#Runs until we have found the number of emitters we are looking for
		while not self.grid_map.solved():
			#Get the new sensing_config for the current iteration
			sensing_config = self.grid_map.get_sensing_config(raster_path, self.tau, iteration)
			print(sensing_config)
			#Simulate flying a round of data collection
			self.fly_round(sensing_config, self.search.new_measurement)
			iteration+=1
			if self.verbose:
				print("Iteration complete")
		emitters = []
		for emitter in self.grid_map.get_emitters():
			emitters.append(emitter.get_pos())
			if self.verbose:
				print(emitter.get_pos())
		return emitters

	def move_drone(self, pos):
		try:
			resp = self._move()
		except rospy.ServiceException as e:
			rospy.logerr("Move Service failed")

	"""
	Simulates flying a round of data collection

	inputs:
	path -- path of points to fly
	new_measurement -- callback function from the search algorithm for new measurements
	"""
	def fly_round(self, path, new_measurement):
		rospy.wait_for_service(self._move_service)
		self._move = rospy.ServiceProxy(self._move_service, std_srvs.srv.Empty)
		rospy.wait_for_service(self._sensor_reading)
		self._get_measurement = rospy.ServiceProxy(self._sensor_reading,SensorReading) 
		#flight = Flight(path, self.drone, self.grid_map, new_measurement, self.move_drone)
		#flight.start()
		for sensing_config in path:
			p = sensing_config[0]
			t = sensing_config[1]
			self.move_drone(list(p))
			time.sleep(2)
			for _ in range(0,t):
				m = self.grid_map.fake_measurement(p)
				try:
					measurement = self._get_measurement(m).measurement
					print(measurement)
				except rospy.ServiceException as e:
					rospy.logerr("Measurement Service Failed")
				(mean, lcb, ucb) = self.search.new_measurement(p, measurement)

				self.grid_map.update_sets(lcb, ucb)
		self._move = None

#parser = argparse.ArgumentParser()
#parser.add_argument("-v", "--verbose", help="Verbose", default=False, type=bool)

#args = parser.parse_args()

#if __name__ == '__main__':
#	s = Search(4, 4, 4, 4, 4, 4, 1, 0, 200, 800, 1000, 3, args.verbose)
#	iteration = 0
#	while not s.solved:
#		emitters = s.run_iteration(iteration)
#		iteration+=1
#	print(emitters)


