import casadi as ca
import casadi.tools as ca_tools

import numpy as np
from draw import Draw_MPC_point_stabilization_v1
import time
from constraint_limits import PCC_parameters



def TemporalDynamics():
    """Vehicle model"""
    A = ca.DM([[0, 1, 0],
               [0, 0, 0],
               [0, 0, 0]])

    B = ca.DM([[0, 0],
               [1, 0],
               [0, 1]])

    return A, B


def HumanDynamics(params, t):
    return ca.DM([params.vl * t, params.vl, 0])


def UpdateState(z, u, dt):
    # A, B = Temporaldynamics()
    # return np.eye(3) @ z + params.dt*(A @ z + B @ u)
    A, B = TemporalDynamics()
    return ca.mtimes(np.eye(3),z) + (ca.mtimes(A, z) + ca.mtimes(B, u)) * dt

def get_ref(x_tilde,params,N): ##params imported created a reference
    
    x   = np.array([0.0]*N)
    vr  = np.array([0.0]*N)
    y   = np.array([0.0]*N)

    vr[:] = params.vr - params.vl
    for i in range(N):
        x[i] = x[i-1] + vr[i]*params.dt
        if params.XL0 - params.lLF <= x_tilde + params.x_d*i and x_tilde + params.x_d*i <= params.XL0 + params.lLr:
            y[i] = 3*params.wl/2
        else:
            y[i] = params.wl/2

    z_ref = np.array([x,vr,y])
    return z_ref


def MPC(params, z_initial, U_prev, z_r=None):
    """MPC solver"""
    N = int(params.x_f / params.x_d)
    z_e = ca.MX.sym("z_e", 3, N)
    u_e = ca.MX.sym("u_e", 2, N+1)
    cost = 0
    constraints = []
    ymin = ca.DM.zeros(N)
    ymax = ca.DM.zeros(N)

    if z_r is None:
        z_r = get_ref(0, params, N)
        
    Plt = HumanDynamics(params,0)
    constraints.append(z_e[:,0] == z_initial.flatten() -Plt)

    for i in range(N):
        cost += ca.dot(z_e[:, i] - z_r[:, i], params.Q3 @ (z_e[:, i] - z_r[:, i]))
        cost += ca.dot(u_e[:, i], params.R @ u_e[:, i])

        ymin[i] = ca.if_else(z_r[0, i] >= params.XL0 - params.lLF and z_r[0, i] <= params.XL0 + params.lLr,
                             params.w + params.wl, params.w)

        ymax[i] = ca.if_else(z_r[0, i] > params.XL0 - params.ls and z_r[0, i] < params.XL0 + params.le,
                             2 * params.wl - params.w, params.wl - params.w)

    
    for i in range(N - 1):      
        constraints.append(z_e[:, i + 1] == UpdateState(z_e[:, i], u_e[:, i], params.dt))

    for i in range(N):
        Plt = HumanDynamics(params,i*params.dt)
        constraints.append(z_e[0,i]  >= 0 - Plt[0])
        constraints.append(z_e[1,i]  >= params.vl + params.epsilon - Plt[1])
        constraints.append(z_e[1,i] <= params.vxmax - Plt[1])
        constraints.append(z_e[2,i] >= ymin[i] - Plt[2])
        constraints.append(z_e[2,i] <= ymax[i] - Plt[2])
        constraints.append(z_e[0,i]  >= 0 - Plt[0])
        
        constraints.append(u_e[1,i] >= params.smin*(z_e[1,:] + Plt[1]))
        constraints.append(u_e[1,i] <= params.smax*(z_e[1,:] + Plt[1]))
        
        constraints.append(u_e[0,i] >= params.axmin)
        constraints.append(u_e[0,i] <= params.axmax)
        constraints.append(u_e[1,i] >= params.vymin)
        constraints.append(u_e[1,i] <= params.vymax)
        
    opt_variables = ca.vertcat( ca.reshape(u_e, -1, 1), ca.reshape(z_e, -1, 1))
    qp = {"x": opt_variables, "f": cost, "g": ca.vertcat(*constraints)}
    solver_opts = {"print_time": False, "ipopt": {"print_level": 0}}

    solver = ca.nlpsol("solver", "ipopt", qp, solver_opts)
    #sol = solver(x0=ca.vertcat(z_initial.flatten(), U_prev.flatten()))

    if solver.stats()["success"]:
        z_opt = sol["x"][:3 * N].full().reshape(3, N)
        u_opt = sol["x"][3 * N:].full().reshape(2, N)
    else:
        z_opt, u_opt = None, None

    return z_opt, u_opt, ymin, ymax


if __name__ == "__main__":
    params = PCC_parameters()
    Nc = 5  # Control horizon

    z_initial = np.array([0, 20 / 3.6, 2.5])
    U_prev = np.array([[0], [0]])

    for i in range(int(180 / Nc)):  # Iteratively call
        z, u, ymin, ymax = MPC(params, z_initial, U_prev)
        if z is not None:
            print(z)
            print(u)
        else:
            print("Optimization failed")

        z_initial = UpdateState(z_initial, u[:, 0], params.dt * Nc)
