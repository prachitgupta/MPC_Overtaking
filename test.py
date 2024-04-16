import casadi as ca
import pickle
import numpy as np
#Load the lookup table from the file
with open("lookup_table2.pkl", "rb") as f:
    lookup_table = pickle.load(f)

# Define symbolic variables
x1 = ca.MX.sym('x')
y = ca.MX.sym('y')
z = ca.MX.sym('z')
w = ca.MX.sym('w')
u = ca.MX.sym('u')
X1 = ca.vcat([x1, y, z,w,u])
x = ca.MX.sym('x', 5)  # Assuming a 5-dimensional state vector
a = np.array([1,2,2,2,2])
aca = ca.MX(a)
# Evaluate the lookup table at the specified point
result = X1 + aca
lookup_value = lookup_table(result)