import sys
import numpy as np
sys.path.insert(1, '../')
from Search import Search

def read_emissions_grid(filename):
	true_emissions_grid = []
	sources = []
	gridsize = []
	with open(filename, 'r') as file:
		line = file.readline()
		gridx, gridy = [int(x) for x in line.split(" ")]
		gridsize = [gridx, gridy]
		for i in range(0, gridx):
			line = file.readline()
			row = [float(x) for x in line.split(" ")]
			true_emissions_grid.append(row)
		num_sources = int(file.readline())
		for i in range(0, num_sources):
			x, y = [int(k) for k in file.readline().split(" ")]
			sources.append((x, y))
	return (true_emissions_grid, sources, gridsize)

def failed(expected, actual, reason):
	print("FAILED")
	print(reason)
	print("Expected: "+str(expected))
	print("Actual: "+str(actual))

def passed():
	print("PASSED")

for i in range(1,5):
	filename = "./test"+str(i)+".txt"
	emissions_grid, sources, gridsize = read_emissions_grid(filename)
	s = Search(gridsize[0], gridsize[1], 4, 4, 4, 4, len(sources), 0, 400, 800, 1000, 2, False)
	emitters = s.start(emissions_grid)
	print("Test "+str(i))
	print("--------------------------")
	if len(emitters) != len(sources):
		failed(sources, emitters, "Did not detect correct amount of sources")
		continue
	if not np.array_equal(sources, emitters):
		failed(sources, emitters, "Incorrect sources")
		continue
	passed()
	print("\n")

