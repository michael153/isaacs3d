import numpy as np
#import pandas as pd
import matplotlib.pyplot as plt
#import seaborn as sns
from mpl_toolkits.mplot3d import Axes3D
from SearchAlgorithm import SearchAlgorithm
from ConfidenceBounds import ConfidenceBound

class Adasearch2_5D(SearchAlgorithm):
    def __init__(self, experiment_number, k, min_background_emmision_rate, max_background_emmision_rate, radiation_source_min_rate, radiation_source_max_rate, gridmap):
        self.experiment_number = experiment_number
        self.k = k
        self.min_background_emmision_rate = min_background_emmision_rate
        self.max_background_emmision_rate = max_background_emmision_rate
        self.radiation_source_min_rate = radiation_source_min_rate
        self.radiation_source_max_rate = radiation_source_max_rate

        # constant c in sensor model
        self.sensor_constant = 1

        # height of drone in cartesian (meters)
        self.drone_height = 2

        # number of standard deviations to use in confidence bounds (i think this is alpha(delta_i))
        self.num_stddevs = 3.72

        # "bias b introduced for numerical stability"
        self.b = 0.1

        self.grid_map = gridmap
        self.num_cells = len(self.grid_map.all_cells())

        self.i = np.zeros(self.num_cells)
        self.I = 1e-6 * np.eye(self.num_cells)

    
    """
    Callback for a new_measurement. Applies least squares to find the updated means for each grid cell

    inputs:
    drone_location -- tuple representing 2D location of drone
    measurement -- float representing on the measurement

    returns:
    tuple (mean, lcb, ucb)

    mean -- 2D grid with estimated mean at each cell
    lcb -- 2D grid with lower bound for radiation levels at each cell
    ucb -- 2D grid with upper bound for radiation levels at each cell
    """
    def new_measurement(self, drone_location, measurement):
        #H is the sensitivity matrix for the radiation sensor
        all_cells = self.grid_map.all_cells()
        H = np.zeros(self.num_cells)
        for i in range(0, self.num_cells):
            H[i] = self.grid_map.h(self.gridmap.get_position(i), drone_location)
        self.i += H * measurement / (measurement + 10.0)
        self.I += np.outer(H, H) / (measurement + 10.0)
        #Compute the new mean values using least squares
        mean = np.linalg.lstsq(self.I, self.i)[0]#.reshape((self.grid_size_i, self.grid_size_j))
        
        I_inv = np.linalg.inv(self.I)
        var = np.diag(I_inv)#.reshape(self.grid_size_i, self.grid_size_j)
        std_dev = np.sqrt(var)
        #Compute lower and upper bounds
        lcb = mean - self.num_stddevs * std_dev
        ucb = mean + self.num_stddevs * std_dev
        #for i in range(0, self.num_cells):
        #    cell = all_cells[i]
        #    cb = ConfidenceBound(lcb[i], ucb[i])
        #    cell.set_confidence_bounds(cb)
        return (mean, lcb, ucb)

    """
    Saves a figure displaying the current upper and lower bounds

    inputs:
    ucb_grid -- 2D array with upper bounds
    lcb_grid -- 2D array with lower bounds
    S_i -- set of cells that can still be emitters
    maximal_emitters -- set of cells that are emitters
    """
    def display_lcb_ucb(self, ucb_grid, lcb_grid, S_i, maximal_emitters):
        ### LCB UCB plot
        fig = plt.figure(figsize=(15, 15))
        ax1 = fig.add_subplot(111, projection='3d')
        _x = np.arange(self.grid_size_i)
        _y = np.arange(self.grid_size_j)
        _xx, _yy = np.meshgrid(_x, _y)
        x, y = _xx.ravel(), _yy.ravel()
        
        #print(x)
        #print(y)

        top = ucb_grid.T.ravel()
        bottom = lcb_grid.T.ravel()
        height = top - bottom
        width = depth = 1
        
        color_array = []
        for _j in np.arange(self.grid_size_j):
            for _i in np.arange(self.grid_size_i):
                if (_i, _j) in S_i:
                #if _i+_j == 3:
                    color_array.append((61/255, 219/255, 159/255, 0.5))
                elif (_i, _j) in maximal_emitters:
                    color_array.append((196/255, 36/255, 12/255, 0.5))
                else:
                    color_array.append((91/255, 115/255, 106/255, 0.5))


        ax1.bar3d(x, y, bottom, width, depth, height, shade=True, color = color_array)
        ax1.set_title('Confidence Bounds')
        ax1.set_xlabel("i")
        ax1.set_ylabel("j")
        ax1.set_zlabel("Emmision Rate")
        #plt.show()
        plt.savefig("figures/confidencebounds{}.png".format(self.fignum))
        self.fignum += 1


