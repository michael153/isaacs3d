import numpy as np
import sys
import time
sys.path.insert(1, './Utils/')
from Graph import Graph
import ShortestPath
#import roslibpy

class Flight:
	def __init__(self, path, drone, gridmap, new_measurement, move_drone):
		self.path = path 
		self.drone = drone
		self.grid_map = gridmap
		self.new_measurement = new_measurement
		self.move_drone = move_drone
#		self.client = roslibpy.Ros(host='192.168.50.160', port='9090')
#		self.client.run()
#		self.talker = roslibpy.Topic(self.client, '/waypoints', 'std_msgs/Float32MultiArray')
#		self.listener = roslibpy.Topic(self.client, '/mission_complete', 'std_msgs/String')
#		self.listener.subscribe(self.mission_complete),
		self.complete = False
		#self.graph = Graph(gridmap)
		self.observation_points = []

#	def __del__(self):
#		self.client.terminate()
#		self.talker.unadvertise()

	def get_drone(self):
		return self.drone

	def get_map(self):
		return self.map

	def get_path(self):
		return self.path

	def set_drone(self, drone):
		self.drone = drone

	def set_path(self, path):
		self.path = path

	def get_measurement(self, point):
		return self.grid_map.fake_measurement(point)

	def mission_complete(self, message):
		self.complete = True
	"""
	Simulates one data collection flight
	"""
	def start(self):
		#For now, just does what the original implementation did and fakes data
		for sensing_config in self.path:
#			talker.publish(roslibpy.Message({'data': list(sensing_config)}))
#			while not self.complete:
#				pass
			self.move_drone(list(sensing_config[0]))
#			time.sleep(2)
			p = sensing_config[0]
			t = sensing_config[1]
			for _ in range(0, t):
				#Temporary fake measurements
				measurement = self.get_measurement(p)
				#Call callback in search algorithm and receive updated values
				(mean, lcb, ucb) = self.new_measurement(p, measurement)
				#Update sets in grid map
				self.grid_map.update_sets(lcb, ucb)
			self.complete = False



	
