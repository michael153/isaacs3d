import queue

class Path:
	def __init__(self):
		self.flightpoints = queue.Queue(maxsize=50)
		self.seenpoints = set()

	def add_point(self, point):
		if not point in self.seenpoints:
			self.flightpoints.put(point)
			self.seenpoints.add(point)

	def get_next_point(self, point):
		return self.flightpoints.get()

	def size(self):
		return self.flightpoints.qsize()


