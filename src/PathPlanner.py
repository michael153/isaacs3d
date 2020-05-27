import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import random
import sys
import time
sys.path.insert(1, './Utils/')
from Graph import Graph
from ShortestPath import a_star_search, shortest_distance, get_shortest_path

class PathPlanner:
	def __init__(self):
		pass
	
	def get_neighbors(self, p, threshold, points):
		node = tuple(p)
		if node not in points:
			return None
		result = []
		for n in points:
			d = self.cost(node, n)
			if d <= threshold[0] or d <= threshold[1] or d <= threshold[2]:
				result.append(n)
		return result

	def cost(self, n1, n2):
		x, y, z = n1
		x2, y2, z2 = n2
		return np.sqrt((x-x2)**2+(y-y2)**2+(z-z2)**2)

	def find_closest(self, points, p, test=lambda x,y: True):
		min_dist = 1000
		closest = None
		for point in points:
			if self.cost(point,p) < min_dist and tuple(point) != tuple(p) and test(point, p):
				min_dist = self.cost(point,p)
				closest = point
		return tuple(closest)

	def tsp(self, points, graph=None, start=None, end=None, voxel_size=None, contains=None, support_points=None):
		min_p = (10000,10000,10000)
		for point in points:
			if point[0] < min_p[0] and point[1] < min_p[1] and point[2] < min_p[2]:
				min_p = point
		included = {}
		path = [min_p]
		latest = min_p
		included[min_p] = True
		while len(included) != len(points):
			neighbors = self.get_neighbors(latest, voxel_size, points)
			all_in = True
			for n in neighbors:
				if n not in included.keys():
					all_in = False
					break
			if all_in:
				neighbors = self.get_neighbors(latest, [1000]*3, points)
			closest = None
			min_dist = 10000
			for n in neighbors:
				if self.cost(n, latest) < min_dist and not n in included.keys():
					min_dist = self.cost(n, latest)
					closest = n
			path.append(closest)
			included[closest] = True
			latest = closest
		insertions = {}
		for i in range(len(path)):
			nex_t = (i+1) % len(path)
			o = path[i]
			d = np.asarray(path[nex_t]) - np.asarray(path[i])
			d = d / np.linalg.norm(d)
			if contains(o,d):
				close_start_support = self.find_closest(support_points, path[i])
				close_end_support = self.find_closest(support_points, path[nex_t])
				o2 = close_start_support
				d2 = np.asarray(close_end_support) - np.asarray(close_start_support)
				d2 = d2 / np.linalg.norm(d2)
				if not contains(o2,d2):
					insertions[i] = [close_start_support, close_end_support]
				else:
					mid_support = self.find_closest(support_points, close_start_support, lambda x,y: x[1] != y[1])
					addon = [close_start_support, mid_support]
					o3 = mid_support
					d3 = np.asarray(close_end_support) - np.asarray(mid_support)
					d3 = d3 / np.linalg.norm(d3)
					while contains(o3,d3):
						mid_support = self.find_closest(support_points, mid_support, lambda x,y: x[1] != y[1])
						o3 = mid_support
						d3 = np.asarray(close_end_support) - np.asarray(mid_support)
						d3 = d3 / np.linalg.norm(d3)
						addon.append(mid_support)
					addon.append(close_end_support)
					insertions[i] = addon
		final_path = []
		for j in range(0, len(path)):
			final_path.append(path[j])
			if j in insertions.keys():
				final_path.extend(insertions[j])
		final_path.append(final_path[0])
		return final_path

	def expand_path(self, path, graph):
		if not graph:
			return path
		expanded_path = []
		for i in range(0, len(path)-1):
			p = get_shortest_path(graph, path[i], path[i+1])
			expanded_path.extend(p[:len(p)-1])
		return expanded_path

	def display_path(self, path):
		first = path[0]
		fig = plt.figure()
		sp = fig.add_subplot(111, projection='3d')
		px = []
		py = []
		pz = []
		for i in range(0, len(path)):
			p = path[i]
			pn = path[(i+1) % len(path)]
			px.append(path[i][0])
			py.append(path[i][1])
			pz.append(path[i][2])
			tx = np.linspace(p[0], pn[0], 3)
			ty = np.linspace(p[1], pn[1], 3)
			tz = np.linspace(p[2], pn[2], 3)
			sp.plot(tx, ty, tz, color='red')
		sp.scatter(px, py, pz)
		plt.show()

	##  Helper Functions

	def diff(self, p1, p2):
		return np.abs(np.subtract(p1, p2))

	def vertical_penalty(self, p1, p2):
		diff = self.diff(p1, p2)
		print((diff[2] / sum(diff)))
		return (diff[2] / sum(diff))

	def distance(self, p1, p2, graph = None):
		if graph:
			return shortest_distance(graph, p1, p2)
		return np.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 + (p1[2]-p2[2])**2)




