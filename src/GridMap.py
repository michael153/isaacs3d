class GridMap:
	def __init__(self):
		pass

	def cartesian_to_grid(self, pos):
		raise Exception("cartesian_to_grid() not implemented")

	def grid_to_cartesian(self, pos):
		raise Exception("grid_to_cartesian() not implemented")

	def solved(self):
		raise Exception("solved() not implemented")

	def distance(self):
		raise Exception("distance() not implemented")

	def h(self, x, z):
		raise Exception("h() not implemented")

	def update_sets(self, lcb_grid, ucb_grip):
		raise Exception("update_sets() not implemented")

	def get_sensing_config(self, raster_path, tau, iteration):
		raise Exception("get_sensing_config() not implemented")

	def get_possible_sources(self):
		raise Exception("get_possible_sources() not implemented")

	def get_emitters(self):
		raise Exception("get_emitters() not implemented")

	def is_occupied(self, pos):
		raise Exception("is_occupied() not implemented")

	def is_measure(self, pos):
		raise Exception("is_measure() not implemented")

	def get_cell(self, pos):
		raise Exception("get_cell() not implemented")