class Position:
	def __init__(self, x=0, y=0, z=0):
		self.x = x
		self.y = y
		self.z = z

	def pos(self):
		if self.z == -1:
			return (self.x, self.y)
		return (self.x, self.y, self.z)

	def X(self):
		return self.x

	def Y(self):
		return self.y

	def Z(self):
		return self.z

	def __hash__(self):
		return ((self.x * 47) + (self.y * 61) + (self.z * 31)) % 100