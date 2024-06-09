# !usr/bin/env python3
# Copyright (c) @ RugvedKatole
#
# Author: Rugved Katole
# Affliation: Indian Institute of Bombay


##TO DO
##slip angle less than 30 deg not working
##improve x refernce trajectory more practical
##better visulaization if possible
### not working for all velocities
##human dynamics

import cvxpy
import numpy as np
import matplotlib.pyplot as plt
from constraint_limits import PCC_parameters
from eval_u import eval_u
from PIL import Image


def Temporaldynamics():
    """vehicle model"""
    A = np.array([[0,1,0],
                  [0,0,0],
                  [0,0,0]])

    B = np.array([[0,0],
                  [1,0],
                  [0,1]])

    return A,B

def humanDynamics(params,t):
    return np.array([params.vl*t,params.vl,0])

def update_z(z,u): 
    ##mpc execute only first input
    A, B = Temporaldynamics()
    return np.eye(3) @ z + params.dt*(A @ z + B @ u)

# def vehicle_model(z,u,x_d=1): 
#     ##mpc execute only first input
#     v = z[0] + u[0]*x_d ## z = state = [v, y] updated (executed)
#     y = z[1] + u[1]*x_d

#     return np.array([v,y])


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

def MPC(z_initial,U_prev,Q,R,S,params,z_r = None): ##z initila updated iteratively and passed
    """MPC solver"""
    N = int(params.x_f/params.x_d)
    z_e = cvxpy.Variable((3,N),"z") ## along columns state updated
    u_e = cvxpy.Variable((2,N),"u")
    t_dash = cvxpy.Variable((1,N),"t_dash") ##time horizon
    cost = 0

    ymin = np.array([0.0]*N)
    ymax = np.array([0.0]*N)
    
    ## initial human dynamics
    Plt = humanDynamics(params,0)
    constraints = [z_e[:,0] == z_initial.flatten() -Plt] # 10e
    
    

    vr_tilde = params.vr - params.vl ##robot - lead speed

    if z_r is None:
        z_r = get_ref(0,params,N) ##reference trajectory

    for i in range(N-1):

        if i != 0:
            cost += cvxpy.quad_form(u_e[:,i+1]-u_e[:,i],S) # ponist control increment
        else:
            cost += cvxpy.quad_form(u_e[:,i]-U_prev[:,i],S)
        
        ## GET VALUE FUNCTION
        # def callback_function(x_value):
        #     u_value = eval_u(x_value)  # Call eval_u with the value of x
        #     # Do something with u_value
        #     print("Value of u:", u_value)
        ##dynamics   
        A, B = Temporaldynamics()       
        constraints += [z_e[:,i+1] == np.eye(3) @ z_e[:,i] + params.dt*(A @ z_e[:,i] + B @ u_e[:,i])]  #10 b  rel simple dynmaics
    
    # cost += params.epsilon*t_dash[0,-1]*N
    
    for i in range(N): ##i column iterate over entire horizon
        
        cost += cvxpy.quad_form(z_e[:,i]-z_r[:,i], Q)
        cost += cvxpy.quad_form(u_e[:,i], R)

        # constraints += [t_dash[0,i] >= cvxpy.inv_pos(z_e[0,i])]
        
        #lateral limits in time domain
        if  z_r[0,i] >= params.XL0 - params.lLF and z_r[0,i] <= params.XL0 + params.lLr:
            ymin[i] = params.w + params.wl
        else:##corridor constraints
            ymin[i] = params.w

        if  z_r[0,i] > params.XL0 - params.ls and z_r[0,i] < params.XL0 + params.le:
            ymax[i] = 2*params.wl - params.w
        else:
            ymax[i] = params.wl - params.w ##wl - w
        
            
            # constraints += [(params.x_d*i -params.XO0 -(params.vo-params.vl)*t_dash[0,i]*i*params.x_d)/params.lOf + (z_e[1,i] - params.YO0)/params.wl <= -1]
            # constraints += [(params.x_d*i -params.XA0 -(params.va-params.vl)*t_dash[0,i]*i*params.x_d)/params.lAr - (z_e[1,i] - params.YA0)/params.wl >= 1]

        ##human dynamics
        Plt = humanDynamics(params,i*params.dt)
        ##constraint state  
        constraints += [z_e[0,i]  >= 0 - Plt[0]] #constraint x
        
        constraints += [z_e[1,i]  >= params.vl + params.epsilon - Plt[1]  ]           #10c
        constraints += [z_e[1,i] <= params.vxmax - Plt[1]]    #10c
        
        constraints += [z_e[2,i] >= ymin[i] - Plt[2]]                      #10c
        constraints += [z_e[2,i] <= ymax[i] - Plt[2]]                       #10c
        
        ##velocity constraint to be imposed
        constraints += [u_e[1,i] >= params.smin*(z_e[1,:] + Plt[1])]  #10f
        constraints += [u_e[1,i] <= params.smax*(z_e[1,:] + Plt[1])]  #10f
        
        ##control limits
        # constraints += [u_e[0,i] >= params.axmin*(2 - z_e[0,i]/vr_tilde)/vr_tilde]  #10d
        # constraints += [u_e[0,i] <= params.axmax*(2 - z_e[0,i]/vr_tilde)/vr_tilde]  #10d
        # constraints += [u_e[1,i] >= params.vymin*(2 - z_e[0,i]/vr_tilde)/vr_tilde]  #10d
        # constraints += [u_e[1,i] <= params.vymax*(2 - z_e[0,i]/vr_tilde)/vr_tilde]  #10d
        
        ##doubt control limits very with time how to incorporate that
        constraints += [u_e[0,i] >= params.axmin]  #10d
        constraints += [u_e[0,i] <= params.axmax]  #10d
        constraints += [u_e[1,i] >= params.vymin]  #10d
        constraints += [u_e[1,i] <= params.vymax]  #10d

    def callback_function(z_e_value):
        u_value = eval_u(z_e_value)  # Call eval_u with the value of x
        # Do something with u_value
        print("Value of u:", u_value)

    qp = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
    qp.solve(solver=cvxpy.ECOS, verbose=False)
   
    print(qp.status)
    
    if qp.status == cvxpy.OPTIMAL or qp.status == cvxpy.OPTIMAL_INACCURATE:
        x = np.array(z_e.value[0,:]).flatten()
        v = np.array(z_e.value[1,:]).flatten()
        y = np.array(z_e.value[2,:]).flatten()
        
        v_dash = np.array(u_e.value[0,:]).flatten()
        y_dash = np.array(u_e.value[1,:]).flatten()
        print(qp.status)
    else:
        x, y, v, v_dash, y_dash = None, None, None, None, None

    return np.array([x,v,y]) , np.array([v_dash, y_dash]) , ymin, ymax

if __name__ == "__main__":
    """whatever"""
    
    params = PCC_parameters()

    N = int(params.x_f/params.x_d)   # Prediction horizon
    Nc = 5 #control horizon

    z_initial = np.array([0, 20/3.6,2.5])
    U_prev    = np.array([[0],[0]])
    z_r       = get_ref(0,params=params,N=int(params.x_f/params.x_d))
    y = np.array([5]*N)
    log_z = []

    for i in range(int(180/Nc)):  ##iteratively call
        z, u, ymin, ymax = MPC(z_initial,U_prev,params.Q3,params.R,params.S,params,z_r)  # z,u shape = (2,180)
        ##optimzed sequence of z and control input
        ##thats how real time plot obtained
        Plt = humanDynamics(params,params.dt*i)
        if(z[1] is not None):
            plt.clf()
            plt.plot(list(range(N)),y,"--",color="0.9") ## dashed line at 5
            plt.plot(list(range(N)),z_r[2,:],"-.",color = "red",linewidth=0.8) ##my reference y sucks
            plt.plot(list(range(N)),ymin,"-",color = "b",linewidth=0.8) ##corridor constraints calculated
            plt.plot(list(range(N)),ymax,"-",color = "b",linewidth=0.8)
            plt.plot(list(range(N)),z[2],color="black") ## lateral y with coordinates
            plt.xlim(0,N*2)
            plt.ylim(0,10)
            plt.ylabel("lateral position(m)") ##spatial domain
            plt.xlabel("Relative Longitudinal position(m)")
            plt.plot(params.XL0,2.5,"s",color="r",label = "lead vehicle") ##obstacle to overtake
            plt.plot(params.XE0,params.YE0,"s",color="green",label = "Ego vehicle relative")  ##car visual
            plt.plot(params.X,params.Y,"s",color="orange",label = "Ego vehicle")  ##car visual absolute
            plt.plot(params.XL0+Plt[0],Plt[2]+2.5,"s",color="maroon",label = "lead vehicle moving")  ##car visual absolute
            plt.plot(z[0],z[2],color="purple") ## yrel vs xrel how car moves on leading reference frame
            plt.legend()
            #plt.pause(0.001)
            
            # Iterate over frames, saving each one
            for i in range(NUM_FRAMES):
                # Plot your frame
                # (Make sure to update data if necessary)
                plt.pause(0.001)  # Pause to let the plot update
                # Save the plot as an image
                filename = f"frame_{i:03d}.png"
                plt.savefig(filename)
                # Open the saved image and append to frames list
                frames.append(Image.open(filename))

            # Save frames as GIF
            frames[0].save("animation.gif", save_all=True, append_images=frames[1:], optimize=False, duration=100, loop=0)
        else:
            print("AH oHHH")
            continue

        for j in range(Nc): ##iterate over control horizon (here not 1) first 5
            z_initial = update_z(z_initial,u[:,j]) ## zinitial updated to be passed to MPC 
            log_z.append(z_initial)
            params.XL0 -=  params.x_d
        print(params.XL0)
        U_prev = u[:,Nc-1:Nc] ##update control uprev as already executed

        params.YE0 = z_initial[2] ##update car visual
        params.XE0 = z_initial[0]
        ## absolute dynamics
        params.X = params.XE0 + Plt[0]
        params.Y = params.YE0 + Plt[2]
        ##absolute lead
        z_r       = get_ref(params.XE0,params=params,N=int(params.x_f/params.x_d))

        
        print(z_initial)
        print(i)


    log_z = np.array(log_z).T
    print(log_z.shape)
    plt.plot(log_z[2,:])
    plt.show()