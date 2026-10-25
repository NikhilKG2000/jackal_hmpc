import numpy as np

class Jackal:
	def __init__(self):
		self.name = "Jackal"
		
		self.lower_bound = {"x": -15.0, "y": -15.0, "psi": -1.0e3, "v": -1.5, "a": -2.0, "omega": -2.0,}
		
		self.upper_bound = {"x":  15.0, "y":  15.0, "psi":  1.0e3, "v":  1.5, "a":  2.0, "omega":  2.0,}
		
	def __str__(self):
		return self.name
		
