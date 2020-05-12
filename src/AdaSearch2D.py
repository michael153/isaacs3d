import numpy as np
#import pandas as pd
import matplotlib.pyplot as plt
#import seaborn as sns
from mpl_toolkits.mplot3d import Axes3D
from SearchAlgorithm import SearchAlgorithm
from ConfidenceBounds import ConfidenceBound

class Adasearch2D(SearchAlgorithm):
    def __init__(self, experiment_number, k, min_background_emmision_rate, max_background_emmision_rate, radiation_source_min_rate, radiation_source_max_rate, grid_size_i, grid_size_j, x_resolution, y_resolution, gridmap):
        self.experiment_number = experiment_number
        self.k = k
        self.min_background_emmision_rate = min_background_emmision_rate
        self.max_background_emmision_rate = max_background_emmision_rate
        self.radiation_source_min_rate = radiation_source_min_rate
        self.radiation_source_max_rate = radiation_source_max_rate
        self.grid_size_i = grid_size_i
        self.grid_size_j = grid_size_j
        self.x_resolution = x_resolution
        self.y_resolution = y_resolution

        # cartesian coordinate space (meters)
        self.x_min = 0
        self.x_max = x_resolution * grid_size_i + self.x_min
        self.y_min = 0
        self.y_max = y_resolution * grid_size_j + self.y_min

        # constant c in sensor model
        self.sensor_constant = 1

        # height of drone in cartesian (meters)
        self.drone_height = 2

        # number of standard deviations to use in confidence bounds (i think this is alpha(delta_i))
        self.num_stddevs = 3.72

        # "bias b introduced for numerical stability"
        self.b = 0.1

        self.grid_map = gridmap

        self.i = np.zeros(self.grid_size_i * self.grid_size_j)
        self.I = 1e-6 * np.eye(self.grid_size_i * self.grid_size_j)

    
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
        H = np.zeros(self.grid_size_i * self.grid_size_j)
        for i in np.arange(self.grid_size_i):
            for j in np.arange(self.grid_size_j):
                x = (i, j)
                H[i * self.grid_size_j + j] = self.grid_map.h(x, drone_location)
        self.i += H * measurement / (measurement + 10.0)
        self.I += np.outer(H, H) / (measurement + 10.0)

        #Compute the new mean values using least squares
        mean = np.linalg.lstsq(self.I, self.i)[0].reshape((self.grid_size_i, self.grid_size_j))

        I_inv = np.linalg.inv(self.I)
        var = np.diag(I_inv).reshape(self.grid_size_i, self.grid_size_j)
        std_dev = np.sqrt(var)
        #Compute lower and upper bounds
        lcb = mean - self.num_stddevs * std_dev
        ucb = mean + self.num_stddevs * std_dev
        for i in np.arange(self.grid_size_i):
            for j in np.arange(self.grid_size_j):
                cell = self.grid_map.get_cell((i, j))
                cb = ConfidenceBound(lcb[i][j], ucb[i][j])
                cell.set_confidence_bounds(cb)
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
"""
    # convert point in grid coordinates to cartesian coordinates
    def grid_to_cartesian(self, i, j):
        x = self.x_min + (i + 1) * self.x_resolution - self.x_resolution / 2
        y = self.y_min + (j + 1) * self.y_resolution - self.y_resolution / 2
        return (x, y)

    # list of grid locations and the time to spend at each (# of measurements to take at each point)
    # returns: [[(x1,y1), tau1], [(x2,y2), tau2], ...]
    def get_sensing_config(self, raster_path, S_i, tau, iteration):
        sensing_config = []
        for point in raster_path:
            if point in S_i:
                sensing_config.append([point, int(tau * 2**iteration)])
            else:
                sensing_config.append([point, tau])
        return sensing_config
"""

"""
    def update_sets(self, ucb_grid, lcb_grid, S_i, maximal_emmiters):
        number_of_remaining_candidates = self.k - len(maximal_emmiters)
        ucb_sorted = np.sort(ucb_grid, axis=None) 
        #print("\n ucb_sorted:")
        #print(ucb_sorted)
        for point in S_i.copy():
            if lcb_grid[point[0], point[1]] > ucb_sorted[(ucb_sorted.size - 2) - number_of_remaining_candidates]:
                print("\n point:")
                print(point)
                print("\n lcb_grid[point[0], point[1]]:")
                print(lcb_grid[point[0], point[1]])
                maximal_emmiters.add(point)
                S_i.remove(point)
    

    def collect_measurement(self, drone_loc, true_emmision_grid):
        # Y := total observed measurement at a given time
        Y = 0
        for i in np.arange(self.grid_size_i):
            for j in np.arange(self.grid_size_j):
                env_point = (i, j)
                emission = np.random.poisson(true_emmision_grid[i][j])
                Y += self.h(env_point, drone_loc) * emission
        return Y
"""
"""
    def adasearch(self, true_emmision_grid):
        lcb_grid = np.zeros([self.grid_size_i, self.grid_size_j])
        ucb_grid = np.tile(np.array([self.radiation_source_max_rate]), (self.grid_size_i, self.grid_size_j))
    
        grid_size = self.grid_size_i * self.grid_size_j


        # sets: {0=pruned, 1=S_i, 2=S_top}
        #sets_grid = np.ones([grid_size_i, grid_size_j])

        ### Algorithm

        # S^top
        maximal_emmiters = set()

        # round
        iteration = 0

        # for teleportation model just treat as number of samples
        tau = 1

        raster_path = []
        for j in np.arange(self.grid_size_j):
            for i in np.arange(self.grid_size_i)[::int((-1)**j)]:
                raster_path.append((i,j))

        # points still in consideration (i.e. need to spend more time at these points)
        S_i = set(raster_path)

        i = np.zeros(self.grid_size_i * self.grid_size_j)
        I = 1e-6 * np.eye(self.grid_size_i * self.grid_size_j)

        plt.figure()
        plt.title("Actual Emission Grid")
        sns.heatmap(true_emmision_grid.T, cmap="coolwarm", annot=True, fmt='.1f', square=True)
        plt.xlim(0, true_emmision_grid.shape[0])
        plt.ylim(0, true_emmision_grid.shape[1])

        self.fignum = 0

        while len(maximal_emmiters) < self.k:
        #for scan_round in np.arange(2):
            # list of grid locations and the time to spend at each (# of measurements to take at each point)
            #sensing_config = self.get_sensing_config(raster_path, S_i, tau, iteration)
            sensing_config = self.get_sensing_config(raster_path, tau, iteration)
            print("\nsensing config:")
            print(sensing_config)
            print("\n S_i:")
            print(S_i)
            
            # config: [[(x1,y1), tau1], [(x2,y2), tau2], ...]
            for config in sensing_config:
                for _ in np.arange(config[1]): # sensorCallback

                    drone_location = config[0]
                    measurement = self.collect_measurement(drone_location, true_emmision_grid)

                    H = np.zeros(self.grid_size_i * self.grid_size_j)
                    for env_i in np.arange(self.grid_size_i):
                        for env_j in np.arange(self.grid_size_j):
                            x = (env_i, env_j)
                            H[env_i * self.grid_size_j + env_j] = self.h(x, drone_location)
                    print(measurement)
                    print(H)
                    i += H * measurement / (measurement + 10.0)
                    I += np.outer(H, H) / (measurement + 10.0)
                    print(I)
                    #Ix = i
                    #I is the H~ -> sensitivity matrix
                    #i is the observered number per area
                    mean = np.linalg.lstsq(I, i)[0].reshape((self.grid_size_i, self.grid_size_j))

                    #print("\n mean: ")
                    #print(mean)
                    u_grid = mean
                    
                    I_inverse = np.linalg.inv(I)
                    var = np.diag(I_inverse)
                    var_grid = var.reshape(self.grid_size_i, self.grid_size_j)
                    std_dev_grid = np.sqrt(var_grid)
                    lcb_grid = u_grid - self.num_stddevs * std_dev_grid
                    ucb_grid = u_grid + self.num_stddevs * std_dev_grid
                    
                    #print("\n lcb_grid: ")
                    #print(lcb_grid)
                    #print("\n ucb_grid: ")
                    #print(ucb_grid)
                    
                    plt.figure()
                    plt.title("Least Squares Estimate")
                    sns.heatmap(u_grid.T, cmap="coolwarm", annot=True, fmt='.1f', square=True)
                    plt.xlim(0, u_grid.shape[0])
                    plt.ylim(0, u_grid.shape[1])
                    plt.savefig("figures/leastSqauresEstimate{}.png".format(self.fignum))

                    self.update_sets(ucb_grid, lcb_grid)#, S_i, maximal_emmiters)
                    self.display_lcb_ucb(ucb_grid, lcb_grid, S_i, maximal_emmiters)

            
            plt.figure()
            plt.title("Round {} Least Squares Estimate".format(iteration))
            sns.heatmap(u_grid.T, cmap="coolwarm", annot=True, fmt='.1f', square=True)
            plt.xlim(0, u_grid.shape[0])
            plt.ylim(0, u_grid.shape[1])
            plt.savefig("figures/Round {} LeastSqauresEstimate.png".format(iteration))
            print("Error Norm: {}".format(np.linalg.norm(u_grid.T - true_emmision_grid.T)))
            
            self.update_sets(ucb_grid, lcb_grid, S_i, maximal_emmiters)
            iteration += 1

        plt.figure()
        plt.title("Error Grid")
        sns.heatmap(u_grid.T - true_emmision_grid.T, cmap="coolwarm", annot=True, fmt='.1f', square=True)
        plt.xlim(0, true_emmision_grid.shape[0])
        plt.ylim(0, true_emmision_grid.shape[1])

        plt.savefig("figures/errorGrid.png")
            
        self.display_lcb_ucb(ucb_grid, lcb_grid, S_i, maximal_emmiters)
"""
"""
search = Adasearch(0, 1, 0, 400, 800, 1000, 4, 4, 4, 4)
true_emmision_grid = np.random.uniform(0, 400, 16)
sources = np.random.choice(16, 2)
for i in sources:
    true_emmision_grid[i] = np.random.uniform(800,1000)
true_emmision_grid = true_emmision_grid.reshape((4,4))
search.adasearch(true_emmision_grid)
"""


