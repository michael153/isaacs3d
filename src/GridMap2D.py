from GridCell import GridCell
from ConfidenceBounds import ConfidenceBound
from GridMap import GridMap
import sys
import math
import numpy as np
sys.path.insert(1, './Utils/')
sys.path.insert(2, '../Utils/')
from Position import Position

class GridMap2D(GridMap):
	def __init__(self, x, y, resx, resy, max_rad, k):
		GridMap.__init__(self)
		self.x = x
		self.y = y
		self.resx = resx
		self.resy = resy
		self.k = k
		maxx = resx * x
		maxy = resy * y
		self.grid = []
		self.emitters = []
		self.possible_sources = []
		self.sensor_constant = 1
		self.drone_height = 2
		for i in np.arange(self.resx / 2, maxx, resx):
			col = []
			for j in np.arange(self.resy / 2, maxy, resy):
				pos = Position(i, j, -1)
				k, l = self.cartesian_to_grid((i, j))
				cb = ConfidenceBound(0, max_rad)
				cell = GridCell(pos, 1, cb)
				col.append(cell)
				self.possible_sources.append(cell)
			self.grid.append(col)

	def cartesian_to_grid(self, pos):
		x, y = pos
		i = ((x + self.resx / 2) / self.resx) - 1
		j = ((y + self.resy / 2) / self.resy) - 1
		return (int(i), int(j))

	def grid_to_cartesian(self, pos):
		i, j = pos
		x = (i + 1) * self.resx - self.resx / 2
		y = (j + 1) * self.resy - self.resy / 2
		return (x, y)

	"""
	Whether or not we have found the number of emitters we are looking for

	returns: boolean
	"""
	def solved(self):
		return len(self.emitters) >= self.k

	def distance(self, loc, drone):
		(i, j) = self.grid_to_cartesian((loc[0], loc[1]))
		(k, l) = self.grid_to_cartesian((drone[0], drone[1]))
		return np.sqrt((i - k)**2 + (j - l)**2 + self.drone_height**2)

	"""
	Sensor sensitivity function

	inputs:
	x -- position we want to evaluate sensitivity at
	z -- position of the drone at measurement time

	returns:
	float -- sensor sensitivity
	"""
	def h(self, x, z):
		return self.sensor_constant / (self.distance(x, z)**2)

	"""
	Updates the maximal emitters and possible_sources sets

	inputs:
	lcb_grid -- 2D array of low confidence bounds
	ucb_grid -- 2D array of high confidence bounds
	"""
	def update_sets(self, lcb_grid, ucb_grid):
		remaining_candidates = self.k - len(self.emitters)
		sorted = np.sort(ucb_grid, axis=None)
		for cell in list(self.possible_sources):
			cell_pos = cell.get_pos()
			i, j = self.cartesian_to_grid(cell_pos.pos())
			if lcb_grid[i][j] > sorted[(sorted.size - 2) - remaining_candidates]:
				self.emitters.append(cell)
				self.possible_sources.remove(cell)
			cell.get_confidence_bounds().set_LB(lcb_grid[i][j])
			cell.get_confidence_bounds().set_UB(lcb_grid[i][j])

	"""
	Returns the sensing config for a certain iteration

	inputs:
	raster_path -- array of path points
	tau -- basic amount of time spent at a single cell
	iteration -- the current iteration

	returns:
	sensing_config -- array of the structure [(cell, t), (cell1, t1), ...] where t is the time to spend at cell
	"""
	def get_sensing_config(self, raster_path, tau, iteration):
		sensing_config = []
		for cell in raster_path:
			if cell in self.possible_sources:
				#Time spent at each cell doubles per iteration
				sensing_config.append([cell.get_pos(), int(tau * 2 ** iteration)])
			else:
				sensing_config.append([cell.get_pos(), tau])
		return sensing_config

	"""
	Returns simple raster path over the grid

	returns:
	raster_path -- array of GridCells to be visited
	"""
	def get_raster_path(self):
		raster_path = []
		for j in np.arange(self.y):
		    for i in np.arange(self.x)[::int(math.pow(-1, j))]:
		        raster_path.append(self.grid[i][j])
		return raster_path

	"""
	Fakes a measurement from drone position loc
	"""
	def fake_measurement(self, loc):
		# Y := total observed measurement at a given time
		Y = 0
		for i in np.arange(self.x):
		    for j in np.arange(self.y):
		        env_point = (i, j)
		        emission = np.random.poisson(self.emissions_grid[i][j])
		        Y += self.h(env_point, loc) * emission
		return Y

	"""
	Generates the "true" emissions grid using background and radiation min and max
	"""
	def generate_emissions_grid(self, background_min, background_max, rad_min, rad_max):
		true_emission_grid = np.random.uniform(background_min, background_max, self.x * self.y)
		sources = np.random.choice(self.x * self.y, self.k)
		for i in sources:
		    true_emission_grid[i] = np.random.uniform(rad_min,rad_max)
		true_emission_grid = true_emission_grid.reshape((self.x,self.y))
		self.emissions_grid = true_emission_grid
		return true_emission_grid

	def set_emissions_grid(self, grid):
		self.emissions_grid = grid

	def get_possible_sources(self):
		return self.possible_sources

	def get_emitters(self):
		return self.emitters

	def is_occupied(self, pos):
		return self.grid[pos[0]][pos[1]].get_occupied()

	def is_measure(self, pos):
		return self.grid[pos[0]][pos[1]].get_measure()

	def get_cell(self, pos):
		return self.grid[pos[0]][pos[1]]

	def get_x(self):
		return self.x

	def get_y(self):
		return self.y