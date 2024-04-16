import numpy as np
import math
import scipy.io 
from get_grid import Grid

## To do 
##improve approxiamation helperOc eval_u = 6.8 , mine =  7.15
##better on improving grid_size
def get_data(Data):
    ##extract data from matlab
    workspace = scipy.io.loadmat(Data)
    grid_size = workspace['grid_size'][0] ## arrAY OF LIST FOR SUM REASON
    grid_size = np.array(grid_size)
    print(grid_size)
    l =  np.array(workspace["grid_min"]).flatten()
    u = np.array(workspace["grid_max"]).flatten()
    Values = workspace["Values"]
    Values = np.array(Values)
    ##reproduce grid for value Function look up
    g = Grid(l,u,5, grid_size, [2])
    #np.array([-4.0, -4.0, -math.pi]), np.array([4.0, 4.0, math.pi]), 3, np.array([40, 40, 40]), [2])


    # ##get value
    # value = g.get_value(Values[...,-1], state)
    return g , Values
    
# state = [15,3,0,7,6] ##dummy state
g, Values = get_data("data13.mat")
print(Values[...,-1].shape[-1])