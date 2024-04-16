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

def simulate_human_motion(states, dt):
    # Define symbolic variables
    x, y, theta, v = states[0], states[1], states[2], states[3]
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
    # Return the updated state
    return ca.vertcat(x_new, y_new, theta_new, v_new)

if __name__ == '__main__':
    T = 0.2  # sampling time [s]
    N = 100  # prediction horizon
    rob_diam = 0.3  # [m]
    ##control limits
    a_max = 4
    delF_max = np.pi/4
    lf,lr = 1.105,1.738
    
    ##load lookup table
    with open("lookup_table2.pkl", "rb") as f:
        lookup_table = pickle.load(f)
        
                                ##define state
    x = ca.MX.sym('x')
    y = ca.MX.sym('y')
    theta = ca.MX.sym('theta')
    v = ca.MX.sym('v')
    
    states = ca.vcat([x, y,theta,v])

    ##DIMENSION
    n_states = states.size()[0]

                                ##define control
    a = ca.MX.sym('a')
    delF = ca.MX.sym('delF')
    controls = ca.vcat([a, delF])
    n_controls = controls.size()[0]
    
                               ##SLIP ANGLE
    Br =  ca.atan((lr/(lf+lr))*ca.tan(delF)) 

    ###dynamics how they will propgate 4d kbm
    rhs = ca.vertcat(v*ca.cos(theta + Br), v*ca.sin(theta + Br))
    rhs = ca.vertcat(rhs, v*ca.tan(Br)/lr)
    rhs = ca.vertcat(rhs, a)

    # function
    ##f(x,u) can be non linear
    f = ca.Function('f', [states, controls], [rhs], [
                    'input_state', 'control_input'], ['rhs'])
    
    
    # Define initial state
    xh0 = 0.0
    yh0 = 0.0
    thetah0 = 0.0
    vh0 = 5.0  # Initial velocity (m/s)
    statesh0 = ca.vertcat(xh0, yh0, thetah0, vh0)
    
                    ##define relative state 
    xrel= ca.MX.sym('xrel')
    yrel = ca.MX.sym('yrel')
    thetarel = ca.MX.sym('thetarel')
    vr = ca.MX.sym('vr')
    vh = ca.MX.sym('vh')
    
  
    
    # for MPC
    U = ca.MX.sym('U', n_controls, N)

    X = ca.MX.sym('X', n_states, N+1)

    P = ca.MX.sym('P', n_states+n_states) ##initial state and final state param 
    

    # define
    Q = np.array([[1.0, 0.0], [0.0, .1]])
    R = np.array([[10, 0.0, 0.0, 0.0], [0.0, 10.0, 0.0, 0.0], [0.0, 0.0, 10.0, 0.0], [0.0, 0.0, 0.0, 10.0]])
    # cost function
    obj = 0  # cost
    g = []  # equal constrains
    
    g.append(X[:, 0]-P[:4]) ##initial condition
    
    statesH = statesh0
    for i in range(N):
        ##human motion simulate
        statesH = simulate_human_motion(statesH,T)
        
        ##calculate relative dynamics
        ##rotation transformation
        xrel = ca.cos(X[2,i]) * (statesH[0] - X[0,i]) + ca.sin(X[2,i])*(statesH[1] - X[1,i])
        yrel = -ca.sin(X[2,i]) * (statesH[0] - X[0,i]) + ca.cos(X[2,i])*(statesH[1] - X[1,i])        
        ##append omega rel , vr and vh
        thetarel = statesH[2] - X[2,i]
        vr = X[3,i]
        vh = statesH[3]
        statesRel = ca.vcat([xrel,yrel,thetarel,vr,vh])
        #print(statesRel.size()[0])
        ##value at xrel
        value = lookup_table(statesRel)
        
        ##obj penalizes deviation from final state (soften) and control
        obj = obj + ca.mtimes([(X[:, i]-P[4:]).T, R, X[:, i]-P[4:]]
                              ) + ca.mtimes([U[:, i].T, Q, U[:, i]])  
        ##value function appended
        obj = obj + value + 1/value 
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
        lbx.append(-a_max)
        lbx.append(-delF_max)
        ubx.append(a_max)
        ubx.append(delF_max)
        
    ##bound constarints on x
    for _ in range(N+1):  # note that this is different with the method using structure
        lbx.append(-np.inf)
        lbx.append(-5.0)
        lbx.append(-np.pi)
        lbx.append(0.0)
        ubx.append(np.inf)
        ubx.append(5.0)
        ubx.append(np.pi)
        ubx.append(20)
        

    # Simulation define params
    t0 = 0.0
    ##initial state
    x0 = np.array([-15.0, 0.0, 0.0, 0.0]).reshape(-1, 1)  # initial state
    x0_ = x0.copy() ##fixed
    ##store next states 
    x_m = np.zeros((n_states, N+1))
    next_states = x_m.copy().T
    
    ##destination soft constraint
    xs = np.array([15, 0.5, 0.0,0.0]).reshape(-1, 1)  # final state
    ##idk maybe initial control
    u0 = np.array([1, 0]*N).reshape(-1, 2).T  # np.ones((N, 2)) # controls
    x_c = []  # contains for the history of the state
    u_c = []
    t_c = []  # for the time
    xx = [] ##robot state
    sim_time = 200.0

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
        rob_diam=2, init_state=x0_, target_state=xs, robot_states=xx)
