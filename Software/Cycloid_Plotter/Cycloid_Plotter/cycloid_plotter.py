# coding: utf-8

######
# code created by Jascha Wallmeier
# based on the research done by Junrong Li, Ke An, Xiaozhong Deng and Jinfan Li
# "A New Tooth Profile Modification Method of Cycloidal Gears in Precision Reducers for Robots"
# DOI:10.3390/app10041266
######

import math as m
import numpy as np
import csv
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt 
#from Cycloid_Plotter import R_c0_X, R_c0_Y
from config import * # import all varaiables from config.py

## basic paramerts and equations ##
alpha_tip_rad = np.deg2rad(90)                              #alpha value at tip (and root) of disc teeth
phi = np.linspace(0,2*np.pi, num_inital_points)             #angle, from 0 to 2*PI
k_1 = a * (z_p/r_p)                                         #short width coefficient
S = np.sqrt(1 + (k_1**2) - 2 * k_1 * np.cos(z_c * phi))     #S, just used as shortcut

def calculate_Rc(phi_val, delta_L_val, S_val):
    return (r_p-(r_rp + delta_L_val)/S_val)*np.sin(phi_val)-(a-k_1*(r_rp + delta_L_val)/S_val)*np.sin(z_p*phi_val), \
           (r_p-(r_rp + delta_L_val)/S_val)*np.cos(phi_val)-(a-k_1*(r_rp + delta_L_val)/S_val)*np.cos(z_p*phi_val)

def calculate_Rc0(phi_val):
    return (r_p - r_rp / S) * np.sin(phi_val) - (a - k_1 * r_rp / S) * np.sin(z_p * phi_val), \
           (r_p - r_rp / S) * np.cos(phi_val) - (a - k_1 * r_rp / S) * np.cos(z_p * phi_val)

def calculate_nc(phi_val):
    return (k_1 * np.sin(z_p * phi_val) - np.sin(phi_val)) / S,\
           (k_1 * np.cos(z_p * phi_val) - np.cos(phi_val)) / S

def calculate_alpha(phi, r_p, r_rp, a, z_p, S):
    R_c0_X, R_c0_Y = calculate_Rc0(phi)
    R_c0_3D = np.stack((R_c0_X, R_c0_Y, np.zeros_like(phi)), axis=-1)
    k = np.array([0, 0, 1])
    v_c = np.cross(R_c0_3D, k)
    n_c_X, n_c_Y = calculate_nc(phi)
    n_c_3D = np.stack((n_c_X, n_c_Y, np.zeros_like(phi)), axis=-1)
    normalized_v_c = v_c / np.linalg.norm(v_c, axis=1, keepdims=True)
    scalar_product = np.sum(normalized_v_c * n_c_3D, axis=1)
    scalar_product = np.clip(scalar_product, -1.0, 1.0)
    alpha = np.arccos(scalar_product)
    return alpha

def calculate_delta_L(alpha, alpha_tip_rad, delta_L_max, delta_L_0):
    alpha_0 = np.min(alpha)
    return ((delta_L_max - delta_L_0) / 2) * (1 - np.cos(((alpha - alpha_0) / (alpha_tip_rad - alpha_0)) * np.pi)) + delta_L_0

## calculate alpha, delta_l, Rc and Rc0
alpha = calculate_alpha(phi, r_p, r_rp, a, z_p, S)
delta_L = calculate_delta_L(alpha, alpha_tip_rad, delta_L_max, delta_L_0)
R_c_X, R_c_Y = calculate_Rc(phi, delta_L, S)
R_c0_X, R_c0_Y = calculate_Rc0(phi)

## create resampled points, needed for a CSV or to plot them ##
if create_csv or plot_resampled:

    # calculate the cumulative arc length
    arc_length = np.cumsum(np.sqrt(np.diff(R_c_X)**2 + np.diff(R_c_Y)**2))
    arc_length = np.insert(arc_length, 0, 0)  # Start from 0

    # resample the curve with uniform arc length parameterization
    desired_arc_length = np.linspace(0, arc_length[-1], num_resampled_points)
    
    # create interpolation functions for X and Y
    x_interp = interp1d(arc_length, R_c_X, kind='cubic')
    y_interp = interp1d(arc_length, R_c_Y, kind='cubic')

    # resample X and Y
    R_c_X_resampled = x_interp(desired_arc_length)
    R_c_Y_resampled = y_interp(desired_arc_length)

    R_c_resampled = np.vstack((R_c_X_resampled, R_c_Y_resampled)).T
    R_c_3D_resampled = np.hstack((R_c_resampled, np.zeros((R_c_resampled.shape[0], 1))))

## export the resampled points as a CSV file ##
if create_csv:
    # export R_c coordinates to a CSV file
    output_file = 'cycloid_points_resampled.csv'
    with open(output_file, 'w', newline='', encoding='utf-8') as csvfile:
        writer = csv.writer(csvfile)
        # append the first point to the end to close the spline
        R_c_3D_resampled_closed = np.vstack((R_c_3D_resampled, R_c_3D_resampled[0]))
        writer.writerows(R_c_3D_resampled_closed)      # write the 3D coordinates (x, y, 0)

    print(f"R_c coordinates were saved to '{output_file}' ")

## find minimum presuure angle ##
if plot_alpha:
    # create filter
    alpha_min_filter = (phi >= 0) & (phi <= np.pi / z_c)    # area between start (0) and first tooth tip (np.pi/z_c)
    alpha_0 = alpha[np.argmin(alpha[alpha_min_filter])]     # find minimum in filter range (returns pos in array), get alpha value at that pos
    phi_0 = phi[np.argmin(alpha[alpha_min_filter])]         # find minimum in filter range (returns pos in array), get phi value at that pos

## plot all the data ##

num_plots = sum([plot_disc|plot_modified|plot_resampled,plot_alpha])    # calculate number of plots
fig = plt.figure(1,figsize=(12,5), tight_layout=True)   #create figure
index = 1 

# Plot R_c0 and R_c
if(plot_disc | plot_modified | plot_resampled):
    ax = fig.add_subplot(1,num_plots,index)
    if(plot_disc):
        ax.plot(R_c0_X, R_c0_Y, label='R_c0(phi)')
    if(plot_modified):
        ax.plot(R_c_X, R_c_Y, label='R_c(phi)')
    if(plot_resampled):
        ax.plot(R_c_X, R_c_Y, 'b.', label='Original Points')        # original points in blue
        ax.plot(R_c_X_resampled, R_c_Y_resampled, 'r.', label='Resampled Points') # resampled points in red
    ax.set(title='Cycloid disc (mm)')
    ax.set(xlabel='X')
    ax.set(ylabel='Y')
    ax.grid(True)
    ax.legend()
    ax.axis('equal') # Ensure the aspect ratio is equal
    index += 1              # increment index for next plot

if(plot_alpha):
    ax = fig.add_subplot(1,num_plots,index)

    ax.plot(np.degrees(phi[alpha_min_filter]), np.degrees(alpha[alpha_min_filter]), label='alpha(phi)')      # plot alpha over filtered range
    ax.plot(np.degrees(phi_0),np.degrees(alpha_0), 'go', label='alpha_0')    # plot alpha_0 as a single point
    text = str(round(np.degrees(alpha_0),2))    # create label text for point
    text += " deg"                  # add unit to label text
    # label point alpha_0:
    ax.annotate(text,        
                       (np.degrees(phi_0),np.degrees(alpha_0)), # position of point
                       textcoords="offset points", 
                       xytext=(3,15),       # text position relativ to point
                       ha='center',         # text alignment
                       )
    ax.set(xlabel='phi [deg]')   # label x-axis
    ax.set(ylabel='Pressure Angle [deg]')    # label y-axis
    ax.set(title=f'Pressure Angle (Alpha) as a function of phi') # title plot
    ax.legend()
    ax.grid(True)
    index += 1          # increment index for next plot

plt.show()