import numpy as np
import math
import scipy.io 
from get_grid import Grid

# Assuming self.grid_points is a list of numpy arrays representing grid points
def get_data(Data):
    ##extract data from matlab
    workspace = scipy.io.loadmat(Data)
    grid_size = workspace['grid_size'][0] ## arrAY OF LIST FOR SUM REASON
    grid_size = np.array(grid_size)
    l =  np.array(workspace["grid_min"]).flatten()
    u = np.array(workspace["grid_max"]).flatten()
    Values = workspace["Values"]
    Values = np.array(Values)
    ##reproduce grid for value Function look up
    g = Grid(l,u,5, grid_size, [2])
    
    return g, Values

g, Values = get_data("data13.mat")
state = [10,2,0,5,6]
grid_points = g.grid_points
V = Values[...,-1] ##cponverged values

# Combined lambda function to get the value of the state from the value function matrix V
# get_value_lambda = lambda V, state, grid_points: V[
#     tuple(
#         np.searchsorted(points, s) - 1 if np.searchsorted(points, s) > 0 and (
#             np.searchsorted(points, s) == len(points) or
#             math.fabs(s - points[np.searchsorted(points, s) - 1]) < math.fabs(s - points[np.searchsorted(points, s)]))
#         else np.searchsorted(points, s) for s, points in zip(state, grid_points)
#     )
# ]

# Define lambda functions
get_value = lambda V, state, grid_points: tuple(
    np.searchsorted(points, s) - 1 if np.searchsorted(points, s) > 0 and (
        np.searchsorted(points, s) == len(points) or
        math.fabs(s - points[np.searchsorted(points, s) - 1]) < math.fabs(s - points[np.searchsorted(points, s)]))
    else np.searchsorted(points, s) for s, points in zip(state, grid_points))



print(get_value(V, state,grid_points))

