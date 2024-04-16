import casadi as ca
import numpy as np
from eval_u import get_data
import pickle
##ToDO
##improve grid size eval_u = 10 ; my_value = 

# Define some data points
g, Values = get_data("data13.mat")
grid_points = g.grid_points  ##in required format
Values_converged = Values[...,-1].ravel(order='F') ##convert to 1 d as in docs (column major)
# Define a CasADi variable representing the independent variable
x = ca.MX.sym('x')

# Create a CasADi interpolant for the lookup table
lookup_table = ca.interpolant("lookup_table2", "bspline", grid_points, Values_converged)

# Save the lookup table to a file
with open("lookup_table2.pkl", "wb") as f:
    pickle.dump(lookup_table, f)

print(lookup_table([15,3,0,5,6]))
