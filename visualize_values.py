import casadi as ca
import pickle
import numpy as np
import matplotlib.pyplot as plt
#Load the lookup table from the file
with open("lookup_table2.pkl", "rb") as f:
    lookup_table = pickle.load(f)
##load trajectories
data = np.load('trajectories.npz')
robot = data['robot']
human = data['human']

def compute_xrel(Xr,Xh):
    xrel = np.cos(Xr[2]) * (-Xh[0] + Xr[0]) + np.sin(Xr[2])*(-Xh[1] + Xr[1])
    yrel = -np.sin(Xr[2]) * (-Xh[0] + Xr[0]) + np.cos(Xr[2])*(-Xh[1] + Xr[1])
    yawrel = Xh[2] - Xr[2]
    vr = Xr[3]
    vh = Xh[3]
    return np.array([xrel,yrel,yawrel,vr,vh])

values = []
for Xr,Xh in zip(robot,human):
    #compute xrel
    statesRel = compute_xrel(Xr, Xh)
    ##compute value
    value = lookup_table(statesRel)
    values.append(value)
values = np.array(values)

# Create a 3D plot
#fig = plt.figure(figsize=(12, 8))
#ax = fig.add_subplot(111, projection='3d')
fig, ax = plt.subplots(figsize=(12, 8))
# Plot the trajectory with color coding based on the value function
# sc = ax.scatter(robot[:, 0], robot[:, 1], robot[:, 2], c=values, cmap='jet')
sc = ax.scatter(robot[:, 0], robot[:, 1], c=values, cmap='jet')

# Add a colorbar to show the values
cbar = plt.colorbar(sc)
cbar.set_label('Value Function')

# Set labels
ax.set_xlabel('X')
ax.set_ylabel('Y')
#ax.set_zlabel('phi')
ax.set_title('Robot Trajectory with Value Function Coloring')

plt.show()