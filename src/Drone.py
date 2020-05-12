class Drone:
	def __init__(self, drone_type, pos, vel):
		self.type = drone_type
		self.pos = pos
		self.vel = vel

	def set_pos(self, p):
		self.pos = p

	def get_pos(self):
		return self.pos

	def get_vel(self):
		return self.vel

	def set_type(self, t):
		self.type = t

	def get_type(self):
		return self.type
