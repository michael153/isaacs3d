from GridCell import GridCell

class ContainerCell(GridCell):
	def __init__(self, pos, ostatus, cb, normal, ob_pt):
		GridCell.__init__(self, pos, ostatus, cb)
		self.normal = normal
		self.observation_point = ob_pt

	def get_normal(self):
		return self.normal

	def set_normal(self, n):
		self.normal = n

	def get_observation_point(self):
		return self.observation_point