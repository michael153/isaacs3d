import numpy as np

class Graph:
    def __init__(self, points, contains, threshold, observation_pts = None):
        self.nodes = set()
        for point in points:
            self.nodes.add(tuple(point))
        self.contains = contains
        self.threshold = threshold
        self.ob_pts = observation_pts

    def neighbors(self, node, threshold=None):
        if not threshold:
            threshold = self.threshold
        node = tuple(node)
        if node not in self.nodes:
            return None
        result = []
        for n in self.nodes:
            d = self.cost(node, n)
            if d <= self.threshold and d != 0:
                if not self.contains(node, np.subtract(n, node)):
                    result.append(n)
        return result

    def custom_neighbors(self, node, points = None):
        if not points:
            points = self.ob_pts
        node = tuple(node)
        if node not in self.nodes:
            return None
        result = []
        for p in points:
            d = self.cost(node, p)
            if d <= self.threshold and d != 0:
                if not self.contains(node, np.subtract(p, node)):
                    result.append(p)
        return result

    def line_intersection(self, start, end):
        start = np.array(start)
        end = np.array(end)
        dir = (end - start)
        if self.contains(start, dir):
            return True
        return False

    def cost(self, p1, p2):
        x, y, z = p1
        x2, y2, z2 = p2
        return np.sqrt((x-x2)**2+(y-y2)**2+(z-z2)**2)