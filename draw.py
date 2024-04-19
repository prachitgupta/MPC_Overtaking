#!/usr/bin/env python
# coding=utf-8

import numpy as np
from matplotlib import pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as mpatches
import matplotlib as mpl


class Draw_MPC_point_stabilization_v1(object):
    def __init__(self, robot_states: list, init_state: np.array, target_state: np.array, rob_diam=0.3,
                 export_fig=True):
        self.robot_states = robot_states
        self.init_state = init_state
        self.target_state = target_state
        self.rob_radius = rob_diam / 2.0
        self.fig = plt.figure()
        self.ax = plt.axes(xlim=(-25, 25), ylim=(-5, 5))
        # self.fig.set_dpi(400)
        self.fig.set_size_inches(7, 6.5)
        # init for plot
        self.animation_init()

        self.ani = animation.FuncAnimation(self.fig, self.animation_loop, range(len(self.robot_states)),
                                           init_func=self.animation_init, interval=100, repeat=False)

        plt.grid('--')
        if export_fig:
            self.ani.save('./v1.gif', writer='imagemagick', fps=100)
        plt.show()

    def animation_init(self):
        # plot target state
        self.target_circle = plt.Circle(self.target_state[:2], self.rob_radius, color='b', fill=False)
        self.ax.add_artist(self.target_circle)
        self.target_arr = mpatches.Arrow(self.target_state[0], self.target_state[1],
                                         self.rob_radius * np.cos(self.target_state[2]),
                                         self.rob_radius * np.sin(self.target_state[2]), width=0.2)
        self.ax.add_patch(self.target_arr)
        self.robot_body = plt.Circle(self.init_state[:2], self.rob_radius, color='r', fill=False)
        self.ax.add_artist(self.robot_body)
        self.robot_arr = mpatches.Arrow(self.init_state[0], self.init_state[1],
                                        self.rob_radius * np.cos(self.init_state[2]),
                                        self.rob_radius * np.sin(self.init_state[2]), width=0.2, color='r')
        self.ax.add_patch(self.robot_arr)
        return self.target_circle, self.target_arr, self.robot_body, self.robot_arr

    def animation_loop(self, indx):
        position = self.robot_states[indx][:2]
        orientation = self.robot_states[indx][2]
        self.robot_body.center = position
        # self.ax.add_artist(self.robot_body)
        self.robot_arr.remove()
        self.robot_arr = mpatches.Arrow(position[0], position[1], self.rob_radius * np.cos(orientation),
                                        self.rob_radius * np.sin(orientation), width=0.2, color='r')
        self.ax.add_patch(self.robot_arr)
        return self.robot_arr, self.robot_body
    
    
##custom function to animate lead vehicle
class Draw_MPC_Overtaking(object):
    def __init__(self, robot_states: list,human_states: list ,init_stateR: np.array,init_stateH: np.array, target_state: np.array, rob_diam=0.3,lead_diam=0.3,show_heatmap = True,
                 export_fig =True):
        self.robot_states = robot_states
        self.human_states = human_states
        self.init_stateR = init_stateR
        self.init_stateH = init_stateH
        self.target_state = target_state
        self.rob_radius = rob_diam / 2.0
        self.hum_radius = lead_diam/ 2.0
        self.fig = plt.figure()
        self.is_heatMap = show_heatmap
        ##animation horizon
        self.ax = plt.axes(xlim=(-30, 32), ylim=(-5, 5))
        # self.fig.set_dpi(400)
        self.fig.set_size_inches(10, 8)
        # init for plot
        self.animation_init()
        
        self.ani = animation.FuncAnimation(self.fig, self.animation_loop, range(len(self.robot_states)),
                                           init_func=self.animation_init, interval=100, repeat=False)

        plt.grid('--')
        if export_fig:
            self.ani.save('./v1.gif', writer='imagemagick', fps=100)
        plt.show()
        if show_heatmap:
            self.heatMap()

    def animation_init(self):
        # plot target state
        self.target_circle = plt.Circle(self.target_state[:2], self.rob_radius, color='b', fill=False)
        self.ax.add_artist(self.target_circle)
        self.target_arr = mpatches.Arrow(self.target_state[0], self.target_state[1],
                                         self.rob_radius * np.cos(self.target_state[2]),
                                         self.rob_radius * np.sin(self.target_state[2]), width=0.2)
        self.ax.add_patch(self.target_arr)
        
        self.robot_body = plt.Circle(self.init_stateR[:2], self.rob_radius, color='r', fill=False)
        self.ax.add_artist(self.robot_body)
        self.robot_arr = mpatches.Arrow(self.init_stateR[0], self.init_stateR[1],
                                        self.rob_radius * np.cos(self.init_stateR[2]),
                                        self.rob_radius * np.sin(self.init_stateR[2]), width=0.2, color='r')
        self.ax.add_patch(self.robot_arr)
        
        self.human_body = plt.Circle(self.init_stateH[:2], self.hum_radius, color='g', fill=False)
        self.ax.add_artist(self.human_body)
        self.human_arr = mpatches.Arrow(self.init_stateH[0], self.init_stateH[1], self.hum_radius * np.cos(self.init_stateH[2]), self.hum_radius * np.sin(self.init_stateH[2]), width=0.2, color='g')
        self.ax.add_patch(self.human_arr)
        
        return self.target_circle, self.target_arr, self.robot_body, self.robot_arr, self.human_body, self.human_arr


    def animation_loop(self, indx):
        robot_position = self.robot_states[indx][:2]
        robot_orientation = self.robot_states[indx][2]
        human_position = self.human_states[indx][:2]
        human_orientation = self.human_states[indx][2]
        self.robot_body.center = robot_position
        self.robot_arr.remove()
        self.robot_arr = mpatches.Arrow(robot_position[0], robot_position[1], self.rob_radius * np.cos(robot_orientation), self.rob_radius * np.sin(robot_orientation), width=0.2, color='r')
        self.ax.add_patch(self.robot_arr)
        
        self.human_body.center = human_position
        self.human_arr.remove()
        self.human_arr = mpatches.Arrow(human_position[0], human_position[1], self.hum_radius * np.cos(human_orientation), self.hum_radius * np.sin(human_orientation), width=0.2, color='g')
        self.ax.add_patch(self.human_arr)
        
        return self.robot_arr, self.robot_body, self.human_arr, self.human_body
    
    def heatMap(self):
        ##compute xrel
        human = self.human_states
        robot = self.robot_states
        ##compute xrel
        
        xrel = np.cos(robot[:][2]) * (-human[:][0] + robot[:][0]) + np.sin(robot[:][2])*(-human[:][1] + robot[:][1])
        yrel = -np.sin(robot[:][2]) * (-human[:][0] + robot[:][0]) + np.cos(robot[:][2])*(-human[:][1] + robot[:][1])
        
        plt.figure(figsize=(13, 8))
        
        ##to do use eval_u to get data for heat map

        # plt.jet()
        
        # # Plot the heat map
        # plt.contourf(grid.coordinate_vectors[0], grid.coordinate_vectors[1], smallValues[:, :, 0,8,5].T)
        # plt.colorbar()

        # # Plot the contour line
        # plt.contour(grid.coordinate_vectors[0],
        #             grid.coordinate_vectors[1],
        #             smallValues[:, :, 0,8,5].T,
        #             levels=0,
        #             colors="black",
        #             linewidths=3)

        # Plot the trajectory
        plt.plot(xrel, yrel, 'k--')
        plt.xlim(-20, 20)
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Heat Map with relative Trajectory')
        plt.show()
            
