import casadi as ca
import pickle

#Load the lookup table from the file
with open("lookup_table2.pkl", "rb") as f:
    lookup_table = pickle.load(f)


def simulate_human_motion(states, dt):
    """
    Simulate the motion of a human-driven car with constant velocity.

    Arguments:
    states : MX
        Current state of the human-driven car (x, y, theta, v).
    dt : float
        Time step for integration.

    Returns:
    MX
        Updated state of the human-driven car after simulation.
    """
    # Define symbolic variables
    x, y, theta, v , p = states[0], states[1], states[2], states[3],states[4]

    # Constant velocity (no acceleration)
    a = 0.0

    # Extended unicycle model dynamics
    L = 1.735  # Wheelbase length
    dxdt = v * ca.cos(theta)
    dydt = v * ca.sin(theta)
    dthetadt = 0.0  # No change in heading angle
    dvdt = 0.0  # Constant velocity

    # Update the state using Euler integration
    x_new = x + dxdt * dt
    y_new = y + dydt * dt
    theta_new = theta + dthetadt * dt
    v_new = v + dvdt * dt
    p_new  = p
    # Return the updated state
    return ca.vertcat(x_new, y_new, theta_new, v_new,p_new)

# Example usage
# Define initial state
x0 = 0.0
y0 = 0.0
theta0 = 0.0
v0 = 3  # Initial velocity (m/s)
p0 = 0
states0 = ca.vertcat(0, 0, theta0, v0,p0)

# Time parameters
dt = 0.1  # Time step
total_time = 10.0  # Total simulation time
num_steps = int(total_time / dt)  # Number of time steps

# Simulate human motion with constant velocity
states = states0
for _ in range(num_steps):
    states = simulate_human_motion(states, dt)
    value = lookup_table(states)
    print(states)

print("Final state after simulation:", states)
