from ContainerCell import ContainerCell
from ConfidenceBounds import ConfidenceBound
from GridMap import GridMap
from Container import Container
from PathPlanner import PathPlanner
import sys
import os
import math
import numpy as np
sys.path.insert(1, './Utils/')
from Position import Position
from Container import Container
from Graph import Graph
from ShortestPath import get_shortest_path

class GridMap2_5D(GridMap):
	def __init__(self, k, filename):
		GridMap.__init__(self)
		self.k = k
		self.sensor_constant = 1
		self.emitters = []
		#self.possible_sources = []
		self.containers = []
		self.emissions_grid = {}
		self.container_path = None
		self.path_planner = PathPlanner()
		self.parse_env_file(filename)
		self.possible_sources = self.all_cells()
		pass

	def parse_env_file(self, filename):
		print(os.getcwd())
		with open(filename, 'r') as file:
			d = file.read().replace("\n", " ").split(" ")
			d = [float(x) if x != '' else -999 for x in d]
			lx, ly, lz = (d[0],d[1],d[2])
			ux, uy, uz = (d[3],d[4],d[5])
			self.container_count = int(d[6])
			i = 7
			for j in range(0, self.container_count):
				mix = d[i]
				miy = d[i+1]
				miz = d[i+2]
				min_c = (mix, miy, miz)
				mx = d[i+3]
				my = d[i+4]
				mz = d[i+5]
				max_c = (mx, my, mz)
				container = Container(min_c, max_c)
				self.containers.append(container)
				i+=7

	def get_containers(self):
		container_pos = []
		for container in self.containers:
			container_pos.append((container.get_center(), container.get_max() - container.get_min()))
		return container_pos

	def solved(self):
		return len(self.emitters) >= self.k

	def get_emitters(self):
		return self.emitters

	def distance(self, loc, drone):
		x, y, z = loc
		dx, dy, dz = drone
		return np.sqrt((dx - x)**2 + (dy-y)**2 + (dz-z)**2)

	def h(self, x, z):
		return self.sensor_constant / (self.distance(x, z)**2)

	def generate_emissions_grid(self, back_min, back_max, rad_min, rad_max):
		source_containers = np.random.randint(0, self.container_count, (1, self.k))
		true_emissions = {}
		for i in range(0, self.container_count):
			num_sources = (source_containers == i).sum()
			container_grid = self.containers[i].generate_emissions_grid(num_sources, back_min, back_max, rad_min, rad_max, true_emissions)
			true_emissions = dict(true_emissions.items() + container_grid.items())
		self.emissions_grid = true_emissions
		print(true_emissions)
		return true_emissions

	def get_raster_path(self):
		all_support_points = []
		all_obs_points = []
		for container in self.containers:
			all_support_points.extend(container.support_points)
		for cell in self.all_cells():
			if cell in self.possible_sources:
				all_obs_points.append(cell.get_observation_point())
		path = self.path_planner.tsp(all_obs_points, voxel_size=self.containers[0].voxel_size, contains=self.all_contains, support_points=all_support_points)
		return path

	def get_no_collision_path(self, p1, p2):
		points = [p1,p2]
		all_support_points = []
		for container in self.containers:
			all_support_points.extend(container.support_points)
		path = self.path_planner.tsp(points, voxel_size=self.containers[0].voxel_size, contains=self.all_contains, support_points=all_support_points)
		return path[:len(path)-1]

	def fake_measurement(self, loc):
		# Y := total observed measurement at a given time
		Y = 0
		for cell in self.all_cells():
			p = cell.get_pos()
			emission = np.random.poisson(self.emissions_grid[p])
			Y += self.h(p, loc) * emission
		return Y

	def all_cells(self):
		cells = []
		for container in self.containers:
			for cell in container.get_cells():
				cells.append(cell)
		return cells

	def update_sets(self, lcb_grid, ucb_grid):
		remaining_candidates = self.k - len(self.emitters)
		sorted_ucb = np.sort(ucb_grid, axis=None)
		sorted_lcb = np.sort(lcb_grid, axis=None)
		cells = self.all_cells()
		for i in range(0, len(cells)):
			cell = cells[i]
			if cell not in self.possible_sources:
				continue
			cell_pos = cell.get_pos()
			if lcb_grid[i] > sorted_ucb[(sorted_ucb.size - 2) - remaining_candidates]:
				self.emitters.append(cell)
				self.possible_sources.remove(cell)
			elif ucb_grid[i] < sorted_lcb[-remaining_candidates]:
				self.possible_sources.remove(cell)
			cell.get_confidence_bounds().set_LB(lcb_grid[i])
			cell.get_confidence_bounds().set_UB(ucb_grid[i])

	def get_sensing_config(self, raster_path, tau, iteration):
		sensing_config = []
		for point in raster_path:
			if point in self.possible_sources:
				sensing_config.append([point, int(tau * 2 ** iteration)])
			else:
				sensing_config.append([point, tau])
		return sensing_config

	def all_contains(self, start, dir):
		for container in self.containers:
			if container.contains_vector(start, dir):
				return True
		return False

		


