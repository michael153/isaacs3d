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
		if len(self.containers) == 1:
			path = self.path_planner.tsp(self.containers[0].get_observation_points(), self.containers[0].graph)
			transport_path = get_shortest_path(self.containers[0].graph, path[len(path)-1], path[0])
			path.extend(transport_path[1:len(transport_path)-1])
			return path

		ob_pts = {}
		all_pts = []
		containers = []
		c_graphs = {}
		for container in self.containers:
			center = tuple(container.get_center())
			ob_pts[center] = container.get_observation_points()
			all_pts.extend(container.get_flight_grid())
			c_graphs[center] = container.graph
			containers.append(center)

		start_pt = (-40,-40,7.5)
		start_container = min(containers, key=lambda x: self.distance(start_pt, x))
		end_container = max(containers, key=lambda x: self.distance(start_pt, x))
		path = []
		c_path = self.path_planner.tsp(containers, start=start_container, end=end_container)
		c_path = c_path[1:len(c_path)-1]
		big_graph = Graph(all_pts, self.all_contains, 100)
		for i in range(0, len(c_path)):
			curr_container = c_path[i]
			next_container = c_path[(i+1) % len(c_path)]
			o_pts = ob_pts[curr_container]
			best_start_dist = float('inf')
			best_start_point = None
			best_end_dist = float('inf')
			best_end_point = None
			for pt in o_pts:
				start_d = self.distance(pt, start_pt)
				end_d = self.distance(pt, next_container)
				if start_d < best_start_dist:
					best_start_dist = start_d
					best_start_point = pt
				if end_d < best_end_dist:
					best_end_dist = end_d
					best_end_point = pt
			if i != 0:
				transport_path = get_shortest_path(big_graph, start_pt, best_start_point)
				path.extend(transport_path[1:len(transport_path)-1])
			container_path = self.path_planner.tsp(o_pts, c_graphs[curr_container], best_start_point, best_end_point)
			path.extend(container_path)
			start_pt = best_end_point
		transport_path = get_shortest_path(big_graph, path[len(path)-1], path[0])
		path.extend(transport_path[1:len(transport_path)-1])
#		self.path_planner.display_path(path)
		return path

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
		sorted = np.sort(ucb_grid, axis=None)
		cells = self.all_cells()
		for i in range(0, len(cells)):
			cell = cells[i]
			if cell not in self.possible_sources:
				continue
			cell_pos = cell.get_pos()
			if lcb_grid[i] > sorted[(sorted.size - 2) - remaining_candidates]:
				self.emitters.append(cell)
				self.possible_sources.remove(cell)
			cell.get_confidence_bounds().set_LB(lcb_grid[i])
			cell.get_confidence_bounds().set_UB(lcb_grid[i])

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

		


