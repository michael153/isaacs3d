class FlightPoint:
	def __init__(self, pos, s):
		self.pos = pos
		self.status = s

	def get_pos(self):
		return self.pos

	def get_flight_point(self):
		return self

	def get_status(self):
		return self.status