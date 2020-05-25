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

		self.start_srv = rospy.Service("/start", std_srvs.srv.Empty, self.start_callback)
		return True

	def start_callback(self, req):
		rospy.wait_for_service(self._takeoff_service)
		self._takeoff = rospy.ServiceProxy(self._takeoff_service, std_srvs.srv.Empty)
		try:
			resp = self._takeoff()
		except rospy.ServiceExeception as e:
			rospy.logerr("Couldn't takeoff")
		time.sleep(5)
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
			sensing_config = self.grid_map.get_sensing_config(self.raster_path, self.tau, iteration)
			#Simulate flying a round of data collection
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


