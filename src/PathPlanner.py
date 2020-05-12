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

	def tsp(self, points, graph=None, start=None, end=None):
		currlen = 0
		mindist = float('inf')
		graph_copy = graph
		closest = []
		if len(points) < 2:
			return points
		path = []
		bestdist = float('inf')
		not_found_count = 0
		if start and end:
			if start not in points or end not in points:
				return -1
			path = [start, end]
		else:
			path = points[0:2]
			points = points[2:]
		distances = []
		distance = self.distance(path[0], path[1], graph)
		distances = [distance] * 2
		while points:
			if not_found_count == 10:
				graph = None
			choices = []
			if graph:
				choices = graph.custom_neighbors(random.choice(path), points)
			p = None
			if len(choices) == 0:
				p = random.choice(points)
			else:
				p = random.choice(choices)
			n = path
			if graph:
				n = graph.custom_neighbors(p)
			minnewlen = distance
			bestgain = float('inf')
			index = -1
			found = False
			predist = -1
			postdist = -1
			dist = -1
			for i in range(0, len(path)):
				s = time.time()
				if path[i] not in n:
					continue
				found = True
				#Check adding before this node
				point = path[i]
				dist = self.distance(p, point, graph)
				if point != start:
					prev = path[(i-1) % len(path)]
					predist = distances[(i-1) % len(distances)]
					newdist = self.distance(prev, p, graph) + dist
					gain = newdist - predist
					if gain < bestgain:
						bestgain = gain
						predist = newdist - dist
						index = i
				if point != end:
					post = path[(i+1) % len(path)]
					postdist = distances[(i+1) % len(distances)]
					newdist = dist + self.distance(p, post, graph)
					gain = newdist - postdist
					if gain < bestgain:
						bestgain = gain
						postdist = newdist - dist
						index = (i+1) % len(path)
				e = time.time()
				#print(e - s)
			if found:
				not_found_count = 0
				path.insert(index, p)
				distances.insert(index, postdist)
				distances[(index-1) % len(distances)] = predist
				points.remove(p)
				minnewlen -= dist
				minnewlen += predist + postdist
			else:
				not_found_count+=1
		return self.expand_path(path, graph_copy)

	"""
	def two_opt(self, path, currlen, graph):
		for _ in range(0, 1000):
			i = random.randrange(0, len(path))
			j = random.randrange(0, len(path))
			while i == j:
				j = random.randrange(0, len(path))
			if graph.line_intersection(path[i], path[(j+1) % len(path)]) or graph.line_intersection(path[j], path[(i+1) % len(path)]):
				continue
			newlen = currlen - self.distance(path[i], path[(i+1) % len(path)], graph) - self.distance(path[j], path[(j+1) % len(path)], graph)
			newlen += self.distance(path[i], path[(j+1) % len(path)], graph) + self.distance(path[j], path[(i+1) % len(path)], graph)
			if newlen < currlen:
				temp = path[(i+1) % len(path)]
				path[(i+1) % len(path)] = path[(j+1) % len(path)]
				path[(j+1) % len(path)] = temp
				currlen = newlen
		return path
	"""

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




