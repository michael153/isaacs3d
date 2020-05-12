import numpy as np
from ContainerCell import ContainerCell
from ConfidenceBounds import ConfidenceBound
from GridMap import GridMap
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys
sys.path.insert(1, './Utils')
from Graph import Graph

class Container(GridMap):
	def __init__(self, min_c, max_c):
		GridMap.__init__(self)
		self.min_corner = np.array(min_c)
		self.max_corner = np.array(max_c)
		self.center = np.add(self.min_corner,self.max_corner) / 2.0
		self.cells = []
		self.min_cell = None
		self.max_cell = None
		self.voxel_counts = []
		self.voxel_size = []
		self.cell_count = 0
		self.min_side = -1
		self.drone_offset = np.array([2,2,2]) #TO-DO: Make this a variable
		self.flight_grid = []
		self.partition()
		self.build_grid()
		self.graph = Graph(self.flight_grid, self.contains_vector, max(self.voxel_size), self.get_observation_points())

	def set_min(self, min_c):
		self.min_corner = min_c

	def set_max(self, max_c):
		self.max_corner = max_c

	def get_min(self):
		return self.min_corner

	def get_max(self):
		return self.max_corner

	def get_center(self):
		return self.center

	def get_observation_points(self):
		min_ob_pt = self.min_cell.get_observation_point()
		max_ob_pt = self.max_cell.get_observation_point()
		points = [min_ob_pt, max_ob_pt]
		for cell in self.cells:
			if cell != self.min_cell and cell != self.max_cell:
				points.append(cell.get_observation_point())
		return points

	def get_flight_grid(self):
		return self.flight_grid

	def get_cells(self):
		return self.cells
		
	"""
	Partitions the container into voxels

	Splits the container in each direction
	Tries to get as close to 4m per voxel in each direction
	"""
	def zero(self, normal):
		return sum(normal) == 0

	def distance(self, p1, p2):
		x, y, z = p1
		x2, y2, z2 = p2
		return np.sqrt((x-x2)**2+(y-y2)**2+(z-z2)**2)

	def build_grid(self):
		start = np.array(self.min_corner) + (self.voxel_size / 2.0)
		end = np.array(self.max_corner) - (self.voxel_size / 2.0)
		dist = np.linalg.norm(((self.min_corner - self.max_corner) / 2.0)[0:2])
		offset = np.linalg.norm(self.drone_offset[0:2])
		for x in [self.min_corner[0], self.max_corner[0]]:
			for y in [self.min_corner[1], self.max_corner[1]]:
				for k in np.arange(start[2], end[2]+0.1, self.voxel_size[2]):
					pos = np.array([x,y,k])
					center = self.center.copy()
					center[2] = k
					dir = pos - center
					n_dir = dir / np.linalg.norm(dir)
					p = center + (n_dir * (dist + offset))
					self.flight_grid.append(p)

	def partition(self):
		dists = np.subtract(self.max_corner, self.min_corner)
		minside = np.argmin(dists[0:2])
		divdists = dists / 4.0
		num = [round(x) if round(x) > 0 else 1 for x in divdists]
		fnum = [float(n) for n in num]
		distper = np.divide(dists, fnum)
		distper[minside] = dists[minside] / 2.0
		offset = np.array([0,0,0])
		s_offset = np.array([0,0,0])
		offset[minside] = dists[minside] / 2.0
		s_offset[1 - minside] = dists[1 - minside] / num[1 - minside]
		num[minside] = 2.0
		self.voxel_counts = num
		self.voxel_size = distper
		start = np.array(self.min_corner) + (distper / 2.0)
		end = np.array(self.max_corner) - (distper / 2.0)
		for i in np.arange(start[0], end[0]+0.1, distper[0]):
			for j in np.arange(start[1], end[1]+0.1, distper[1]):
				for k in np.arange(start[2], end[2]+0.1, distper[2]):
					cb = ConfidenceBound(-2,-1)
					normal = (np.array([i, j, k]) - self.center)
					normal /= np.linalg.norm(normal)
					proj = offset * np.dot(normal, offset) / np.dot(offset, offset)
					normal = proj / np.linalg.norm(proj)

					c_normal = (np.array([i, j, k]) - self.center)
					c_normal /= np.linalg.norm(c_normal)
					s_proj = s_offset * np.dot(c_normal, s_offset) / np.dot(s_offset, s_offset)

					s_normal = s_proj / np.linalg.norm(s_proj)
					
					#Observation Point:
					p = np.array([i, j, k])
					ob_pt = p + (normal * ((distper / 2.0) + self.drone_offset))
					s_pt = p + (s_normal * ((distper / 2.0) + self.drone_offset))

					if not self.contains_point(s_pt):
						#print(s_pt)
						self.flight_grid.append(s_pt)
					self.flight_grid.append(ob_pt)

					cell = ContainerCell((i,j,k), True, cb, normal, tuple(ob_pt))
					if self.cell_count == 0:
						self.min_cell = cell
					if self.cell_count == num[0]*num[1]*num[2] - 1:
						self.max_cell = cell
					#print(self.cell_count)
					#print(num)
					self.cell_count+=1
					self.cells.append(cell)
		#self.display_partitions()
	
	def generate_emissions_grid(self, num_sources, back_min, back_max, rad_min, rad_max, emissions_grid):
		a = np.arange(self.cell_count)
		np.random.shuffle(a)
		sources = a[:num_sources]
		count = 0
		for cell in self.cells:
			p = cell.get_pos()
			if count in sources:
				emissions_grid[p] = np.random.uniform(rad_min, rad_max)
			else:
				emissions_grid[p] = np.random.uniform(back_min, back_max)
			count+=1
		return emissions_grid

	def contains_vector(self, origin, direction):
		tmin, tmax = 0.0, 0.0
		t0x = (self.min_corner[0] - origin[0]) / direction[0]
		t1x = (self.max_corner[0] - origin[0]) / direction[0]
		txmin = min(t0x, t1x)
		txmax = max(t0x, t1x)

		t0y = (self.min_corner[1] - origin[1]) / direction[1]
		t1y = (self.max_corner[1] - origin[1]) / direction[1]
		tymin = min(t0y, t1y)
		tymax = max(t0y, t1y)

		t0z = (self.min_corner[2] - origin[2]) / direction[2]
		t1z = (self.max_corner[2] - origin[2]) / direction[2]
		tzmin = min(t0z, t1z)
		tzmax = max(t0z, t1z)

		if txmin > tymax or tymin > txmax:
			return False

		tmin = max(txmin, tymin)
		tmax = min(txmax, tymax)

		if tmin > tzmax or tzmin > tmax:
			return False

		tmin = max(tmin, tzmin)
		tmax = min(tmax, tzmax)

		if tmax <= 1 and tmin >= 0:
			return True
		return False

	def contains_point(self, point):
		if point[0] <= self.max_corner[0] and point[0] >= self.min_corner[0]:
			if point[1] <= self.max_corner[1] and point[1] >= self.min_corner[1]:
				if point[2] <= self.max_corner[2] and point[2] >= self.min_corner[2]:
					return True
		return False
		
	def display_partitions(self):
		fig = plt.figure()
		sp = fig.add_subplot(111, projection="3d")
		px = []
		py = []
		pz = []
		ox = []
		oy = []
		oz = []
		cx = []
		cy = []
		cz = []
		for cell in self.cells:
			p = cell.get_pos()
			n = cell.get_normal()
			o = cell.get_observation_point()
			px.append(p[0])
			py.append(p[1])
			pz.append(p[2])
			ox.append(o[0])
			oy.append(o[1])
			oz.append(o[2])
			tx = np.linspace(p[0], p[0] + 1 * n[0], 3)
			ty = np.linspace(p[1], p[1] + 1 * n[1], 3)
			tz = np.linspace(p[2], p[2] + 1 * n[2], 3)
			sp.plot(tx, ty, tz, color='blue')
		ex = []
		ey = []
		ez = []
		for pt in self.flight_grid:
			ex.append(pt[0])
			ey.append(pt[1])
			ez.append(pt[2])
		cx.append(self.min_corner[0])
		cx.append(self.max_corner[0])
		cy.append(self.min_corner[1])
		cy.append(self.max_corner[1])
		cz.append(self.min_corner[2])
		cz.append(self.max_corner[2])
		sp.scatter(px, py, pz)
		sp.scatter(ox, oy, oz)
		sp.scatter(cx, cy, cz)
		sp.scatter(ex, ey, ez)
		plt.show()

	def cartesian_to_grid(self, loc):
		x, y, z = loc
		i = int((x - (voxel_size[0]/2.0) - self.min_corner[0]) / float(num[0]))
		j = int((x - (voxel_size[1]/2.0) - self.min_corner[1]) / float(num[1]))
		k = int((x - (voxel_size[2]/2.0) - self.min_corner[2]) / float(num[2]))
		return (i, j, k)

	def grid_to_cartesian(self, loc):
		i, j, k = loc
		x = self.min_corner[0] + (i * voxel_size[0]) + voxel_size[0]/2.0
		y = self.min_corner[1] + (j * voxel_size[1]) + voxel_size[1]/2.0
		z = self.min_corner[2] + (k * voxel_size[2]) + voxel_size[2]/2.0
		return (x,y,z)