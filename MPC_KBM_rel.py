#!/usr/bin/env python
# -*- coding: utf-8 -*-

import casadi as ca
import casadi.tools as ca_tools
import pickle
import numpy as np
import time
from draw import Draw_MPC_point_stabilization_v1


def shift_movement(T, t0, x0, u, x_f, f): ##time step, cureent time,current pos, control prediction,state prediction and dynamics
    f_value = f(x0, u[0, :]) ##Xdot  ##exected only once
    st = x0 + T*f_value.full() ##update
    t = t0 + T ##update time
    # print(u[:,0])
    # u_end = np.concatenate((u[:, 1:], u[:, -1:]), axis=1)
    u_end = np.concatenate((u[1:], u[-1:])) #shifts all elements of u one position to the left(as first executed), and the last element wraps around to the beginning.
    # x_f = np.concatenate((x_f[:, 1:], x_f[:, -1:]), axis=1)
    x_f = np.concatenate((x_f[1:], x_f[-1:]), axis=0)

    return t, st, u_end, x_f ##st is precisly what i needed


if __name__ == '__main__':
    T = 0.2  # sampling time [s]
    N = 100  # prediction horizon
    rob_diam = 0.3  # [m]
    ##control limits
    v_max = 0.6
    omega_max = np.pi/4.0
    lf,lr = 1.105,1.738
    
    ##load lookup table
    with open("lookup_table2.pkl", "rb") as f:
        lookup_table_loaded = pickle.load(f)
        
                                ##define state
    ##KBM for robot, curvature for human
    xrel = ca.SX.sym('xrel')
    yrel = ca.SX.sym('yrel')
    yawrel = ca.SX.sym('yawrel')
    vrobot = ca.SX.sym('vrobot')
    vhuman  = ca.SX.sym('vhuman')
    
    
    states = ca.vertcat(xrel, yrel)
    states = ca.vertcat(states, yawrel)
    states = ca.vertcat(states, vrobot)
    states = ca.vertcat(states, vhuman)
    states = ca.vcat([xrel,yrel,yawrel,vrobot,vhuman])
    
    ##DIMENSION
    n_states = states.size()[0]

                                ##define control robot
    ar = ca.SX.sym('ar')
    delfr = ca.SX.sym('delfr')
    Robotcontrols = ca.vertcat(ar, delfr)
    n_Robotcontrols = Robotcontrols.size()[0]
                                ##slip angle
    Br =  ca.atan((lr/(lf+lr))*ca.tan(delfr)) 
                                ##human
    ah = ca.SX.sym('ah')
    omegah = ca.SX.sym('omegah')
    Robotcontrols = ca.vertcat(ah, omegah)
    n_Humancontrols = HUmancontrols.size()[0]
    
    ###dynamics how they will propgate
    ##update px py
    rhs = ca.vertcat((vrobot*yrel/lr)*ca.sin(Br)+ vhuman*ca.cos(yawrel) - vrobot*ca.cos(Br),
                    (-vrobot*xrel/lr)*ca.sin(Br)+ vhuman*ca.sin(yawrel) - vrobot*ca.sin(Br))
    ##update 
    rhs = ca.vertcat(rhs, omegah- vrobot*ca.sin(Br)/lr)
    rhs = ca.vertcat(rhs, ar)
    rhs = ca.vertcat(rhs, ah)

    # function
    ##f(x,u) can be non linear
    f = ca.Function('f', [states, controls], [rhs], [
                    'input_state', 'control_input'], ['rhs'])

    # for MPC
    U = ca.SX.sym('U', n_controls, N)

    X = ca.SX.sym('X', n_states, N+1)

    P = ca.SX.sym('P', n_states+n_states) ##initial state and final state param 

    # define
    Q = np.array([[1.0, 0.0, 0.0], [0.0, 5.0, 0.0], [0.0, 0.0, .1]])
    R = np.array([[0.5, 0.0], [0.0, 0.05]])
    # cost function
    obj = 0  # cost
    g = []  # equal constrains
    
    g.append(X[:, 0]-P[:3]) ##initial condition
    for i in range(N):
        ##import value function somehow
        
        ##obj penalizes deviation from final state (soften) and control
        obj = obj + ca.mtimes([(X[:, i]-P[3:]).T, Q, X[:, i]-P[3:]]
                              ) + ca.mtimes([U[:, i].T, R, U[:, i]])
        ##dynamics
        x_next_ = f(X[:, i], U[:, i])*T + X[:, i] ##xt+1 = xdot * T + xt
        g.append(X[:, i+1]-x_next_)
    
    ##define solver
    opt_variables = ca.vertcat(ca.reshape(U, -1, 1), ca.reshape(X, -1, 1))

    nlp_prob = {'f': obj, 'x': opt_variables, 'p': P, 'g': ca.vertcat(*g)}
    opts_setting = {'ipopt.max_iter': 100, 'ipopt.print_level': 5   , 'print_time': 0,
                    'ipopt.acceptable_tol': 1e-8, 'ipopt.acceptable_obj_change_tol': 1e-6}

    solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts_setting)

    lbg = 0.0
    ubg = 0.0
    lbx = []
    ubx = []

    ## bound constarinst on u
    for _ in range(N):
        lbx.append(-v_max)
        lbx.append(-omega_max)
        ubx.append(v_max)
        ubx.append(omega_max)
        
    ##bound constarints on x
    for _ in range(N+1):  # note that this is different with the method using structure
        lbx.append(-2.0)
        lbx.append(-2.0)
        lbx.append(-np.inf)
        ubx.append(2.0)
        ubx.append(2.0)
        ubx.append(np.inf)

    # Simulation define params
    t0 = 0.0
    ##initial state
    x0 = np.array([0.0, 0.0, 0.0]).reshape(-1, 1)  # initial state
    x0_ = x0.copy() ##fixed
    ##store next states 
    x_m = np.zeros((n_states, N+1))
    next_states = x_m.copy().T
    
    ##destination soft constraint
    xs = np.array([1.5, 1.5, 0.0]).reshape(-1, 1)  # final state
    ##idk maybe initial control
    u0 = np.array([1, 2]*N).reshape(-1, 2).T  # np.ones((N, 2)) # controls
    x_c = []  # contains for the history of the state
    u_c = []
    t_c = []  # for the time
    xx = [] ##robot state
    sim_time = 20.0

    # start MPC
    mpciter = 0
    start_time = time.time()
    index_t = []
    # inital test

    while(np.linalg.norm(x0-xs) > 1e-2 and mpciter-sim_time/T < 0.0): ##how much accuracy in reaching goal
        # set parameter
        c_p = np.concatenate((x0, xs)) ##parameter storing initial and final state (initial updates and final fixed)
        init_control = np.concatenate((u0.reshape(-1, 1), next_states.reshape(-1, 1)))
        t_ = time.time()
        
        ##called MPC solver 
        ##Todo this is prediction horizon
        ##x0 and hence c_p updates
        res = solver(x0=init_control, p=c_p, lbg=lbg,
                     lbx=lbx, ubg=ubg, ubx=ubx)
        index_t.append(time.time() - t_)
        # the feedback is in the series [u0, x0, u1, x1, ...]
        estimated_opt = res['x'].full() ##all otimal variables
        ##entire horizon optimal control sequence and states prediction returned  (check shapes)
        u0 = estimated_opt[:200].reshape(N, n_controls)  # (N, n_controls)
        x_m = estimated_opt[200:].reshape(N+1, n_states)  # (N+1, n_states)
        
        ##history upddates
        x_c.append(x_m.T) ##transpose state sorted, entire predicted trajectory stored
        u_c.append(u0[0, :])  ##only first control stored
        t_c.append(t0)
        
        ##todo add control horizon
        ##update state
        t0, x0, u0, next_states = shift_movement(T, t0, x0, u0, x_m, f) ##entire prediction horizon for contol
        ##u0 , next states are predictions excluding first
        ##update initial states which are actually executed
        x0 = ca.reshape(x0, -1, 1)
        x0 = x0.full()
        xx.append(x0) ##this is precisley how he moves
        # print(u0[0])
        mpciter = mpciter + 1
    t_v = np.array(index_t)
    print(t_v.mean())
    print((time.time() - start_time)/(mpciter))
    
    ##to do add plots for visulation (best case trajecory on a heat map)

    draw_result = Draw_MPC_point_stabilization_v1(
        rob_diam=0.3, init_state=x0_, target_state=xs, robot_states=xx)
