class ConfidenceBound:
	def __init__(self, lcb, ucb):
		self.lb = lcb
		self.ub = ucb

	def get_UB(self):
		return self.ub

	def get_LB(self):
		return self.lb

	def set_LB(self, l):
		self.lb = l

	def set_UB(self, u):
		self.ub = u
