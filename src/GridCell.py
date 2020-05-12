class GridCell:
	def __init__(self, pos, ostatus, cb):
		self.pos = pos
		self.occupied = ostatus
		self.cbounds = cb

	def get_pos(self):
		return self.pos

	def get_occupied(self):
		return self.occupied

	def set_pos(self, p):
		self.pos = p

	def set_occupied(self, o):
		self.occupied = o

	def get_confidence_bounds(self):
		return self.cbounds

	def set_confidence_bounds(self, cb):
		self.cbounds = cb