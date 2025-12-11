#
# Cycloid Gear Profile Plotter and DXF Exporter
# Code created by Jascha Wallmeier with the help of Google Gemini
#
# Based on the research by Junrong Li et al.:
# "A New Tooth Profile Modification Method of Cycloidal Gears in Precision Reducers for Robots"
# DOI:10.3390/app10041266
#

import math as m
import numpy as np
import csv
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt 
from config import * # Import all variables from config.py

# Import for DXF export - Requires 'ezdxf' library to be installed (e.g., pip install ezdxf)
if create_dxf:
    try:
        import ezdxf
    except ImportError:
        print("Warning: 'ezdxf' is not installed. DXF export function will be skipped. Please install it with 'pip install ezdxf'.")
        create_dxf = False # Disable DXF export if the library is missing


## BASIC PARAMETERS AND EQUATIONS ##
alpha_tip_rad = np.deg2rad(90)                              # Alpha value at tip (and root) of disc teeth (90 degrees)
phi = np.linspace(0,2*np.pi, num_inital_points)             # Angle for calculation, from 0 to 2*PI
k_1 = a * (z_p/r_p)                                         # Short width coefficient
S = np.sqrt(1 + (k_1**2) - 2 * k_1 * np.cos(z_c * phi))     # S shortcut term

# Calculates the modified cycloid profile coordinates R_c (X, Y)
def calculate_Rc(phi_val, delta_L_val, S_val):
    return (r_p-(r_rp + delta_L_val)/S_val)*np.sin(phi_val)-(a-k_1*(r_rp + delta_L_val)/S_val)*np.sin(z_p*phi_val), \
           (r_p-(r_rp + delta_L_val)/S_val)*np.cos(phi_val)-(a-k_1*(r_rp + delta_L_val)/S_val)*np.cos(z_p*phi_val)

# Calculates the unmodified cycloid profile coordinates R_c0 (X, Y)
def calculate_Rc0(phi_val):
    return (r_p - r_rp / S) * np.sin(phi_val) - (a - k_1 * r_rp / S) * np.sin(z_p * phi_val), \
           (r_p - r_rp / S) * np.cos(phi_val) - (a - k_1 * r_rp / S) * np.cos(z_p * phi_val)

# Calculates the normal vector n_c (X, Y) for the pressure angle
def calculate_nc(phi_val):
    return (k_1 * np.sin(z_p * phi_val) - np.sin(phi_val)) / S,\
           (k_1 * np.cos(z_p * phi_val) - np.cos(phi_val)) / S

# Calculates the pressure angle alpha
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

# Calculates the profile modification offset delta_L based on the pressure angle
def calculate_delta_L(alpha, alpha_tip_rad, delta_L_max, delta_L_0):
    alpha_0 = np.min(alpha)
    return ((delta_L_max - delta_L_0) / 2) * (1 - np.cos(((alpha - alpha_0) / (alpha_tip_rad - alpha_0)) * np.pi)) + delta_L_0

# DXF FUNCTION: EXPORT R_c0 AS NURBS SPLINE (Unmodified)
def export_Rc0_to_dxf_nurbs(R_c0_X, R_c0_Y, filename='cycloid_Rc0_unmodified.dxf'):
    """Creates a NURBS Spline for the unmodified R_c0 contour."""
    doc = ezdxf.new(dxfversion="R2018")
    msp = doc.modelspace()
    
    # Set drawing units to Millimeters (4)
    doc.header['$INSUNITS'] = 4 
    doc.header['$LUNITS'] = 4   
    
    # Points as a list of (x, y) tuples
    points = list(zip(R_c0_X, R_c0_Y))
    
    # Create the NURBS spline (degree 3 is cubic)
    msp.add_spline(
        points,  
        degree=3 
        # 'closed=True' parameter removed due to incompatibility with some ezdxf versions
    )
    
    try:
        doc.saveas(filename)
        print(f"R_c0 Contour (unmodified) saved as smooth NURBS SPLINE in '{filename}'.")
        print(f"INFO: DXF file explicitly scaled to millimeters (4).")
    except IOError as e:
        print(f"Error saving DXF file: {e}")

# DXF FUNCTION: EXPORT R_c AS NURBS SPLINE (Modified / Production)
def export_Rc_to_dxf_nurbs(R_c_X, R_c_Y, filename='cycloid_Rc_modified.dxf'):
    """Creates a NURBS Spline for the modified R_c contour (production profile)."""
    doc = ezdxf.new(dxfversion="R2018")
    msp = doc.modelspace()
    
    # Set drawing units to Millimeters (4)
    doc.header['$INSUNITS'] = 4 
    doc.header['$LUNITS'] = 4   
    
    # Points as a list of (x, y) tuples
    points = list(zip(R_c_X, R_c_Y))
    
    # Create the NURBS spline (degree 3 is cubic)
    msp.add_spline(
        points,  
        degree=3 
        # 'closed=True' parameter removed due to incompatibility with some ezdxf versions
    )
    
    try:
        doc.saveas(filename)
        print(f"R_c Contour (modified/production) saved as smooth NURBS SPLINE in '{filename}'.")
        print(f"INFO: DXF file explicitly scaled to millimeters (4).")
    except IOError as e:
        print(f"Error saving DXF file: {e}")


## CALCULATE CONTOURS ##
alpha = calculate_alpha(phi, r_p, r_rp, a, z_p, S)
delta_L = calculate_delta_L(alpha, alpha_tip_rad, delta_L_max, delta_L_0)
R_c_X, R_c_Y = calculate_Rc(phi, delta_L, S)   # R_c is the modified contour (production)
R_c0_X, R_c0_Y = calculate_Rc0(phi)           # R_c0 is the unmodified contour

## EXPORT DXF FILES (NURBS SPLINE) ##
if create_dxf:
    if 'ezdxf' in globals() and ezdxf: 
        # Export unmodified contour for reference
        export_Rc0_to_dxf_nurbs(R_c0_X, R_c0_Y)
        # Export modified contour (R_c) for manufacturing
        export_Rc_to_dxf_nurbs(R_c_X, R_c_Y)


## CREATE RESAMPLED POINTS (Used for CSV export and resampling plot) ##
# Resampling is performed on the modified R_c contour, which is relevant for production.
R_c_X_resampled = []
R_c_Y_resampled = []
if create_csv or plot_resampled:
    # Calculate the cumulative arc length for R_c
    arc_length = np.cumsum(np.sqrt(np.diff(R_c_X)**2 + np.diff(R_c_Y)**2))
    arc_length = np.insert(arc_length, 0, 0)  # Start from 0

    # Resample the curve with uniform arc length parameterization
    desired_arc_length = np.linspace(0, arc_length[-1], num_resampled_points)
    
    # Create interpolation functions for X and Y for R_c
    x_interp = interp1d(arc_length, R_c_X, kind='cubic')
    y_interp = interp1d(arc_length, R_c_Y, kind='cubic')

    # Resample X and Y
    R_c_X_resampled = x_interp(desired_arc_length)
    R_c_Y_resampled = y_interp(desired_arc_length)

    R_c_resampled = np.vstack((R_c_X_resampled, R_c_Y_resampled)).T
    R_c_3D_resampled = np.hstack((R_c_resampled, np.zeros((R_c_resampled.shape[0], 1))))

    

## EXPORT RESAMPLED POINTS AS A CSV FILE ##
if create_csv:
    # Export R_c coordinates to a CSV file (Uses R_c_3D_resampled)
    output_file = 'cycloid_points_resampled.csv'
    with open(output_file, 'w', newline='', encoding='utf-8') as csvfile:
        writer = csv.writer(csvfile)

        # Append the first point to the end to close the spline/polyline
        R_c_3D_resampled_closed = np.vstack((R_c_3D_resampled, R_c_3D_resampled[0]))
        writer.writerows(R_c_3D_resampled_closed)      # Write the 3D coordinates (x, y, 0)

    print(f"R_c coordinates saved to '{output_file}' ")

## FIND MINIMUM PRESSURE ANGLE ##
if plot_alpha:
    # Create filter to focus on one tooth segment (0 to tip)
    alpha_min_filter = (phi >= 0) & (phi <= np.pi / z_c)    
    # Find minimum alpha value in the filtered range
    alpha_0 = alpha[np.argmin(alpha[alpha_min_filter])]     
    # Find the corresponding phi angle
    phi_0 = phi[np.argmin(alpha[alpha_min_filter])]         

## PLOT ALL DATA ##

num_plots = sum([plot_disc|plot_modified|plot_resampled,plot_alpha])    # Calculate number of required subplots
fig = plt.figure(1,figsize=(12,5), tight_layout=True)   # Create figure
index = 1 

# Plot R_c0 and R_c
if(plot_disc | plot_modified | plot_resampled):
    ax = fig.add_subplot(1,num_plots,index)
    if(plot_disc):
        ax.plot(R_c0_X, R_c0_Y, label='R_c0 (Unmodified)') # Unmodified contour
    if(plot_modified):
        ax.plot(R_c_X, R_c_Y, label='R_c (Modified)') # Modified contour (production)
    if(plot_resampled):
        # Plot original R_c points and resampled R_c points
        ax.plot(R_c_X, R_c_Y, 'b.', label='R_c Original Points')        
        ax.plot(R_c_X_resampled, R_c_Y_resampled, 'r.', label='R_c Resampled Points') 
    ax.set(title='Cycloid Disc Profile (mm)') # Corrected title
    ax.set(xlabel='X')
    ax.set(ylabel='Y')
    ax.grid(True)
    ax.legend()
    ax.axis('equal') # Ensure the aspect ratio is equal
    index += 1              # Increment index for next plot

if(plot_alpha):
    ax = fig.add_subplot(1,num_plots,index)

    ax.plot(np.degrees(phi[alpha_min_filter]), np.degrees(alpha[alpha_min_filter]), label='Alpha(phi)')      # Plot alpha over filtered range
    ax.plot(np.degrees(phi_0),np.degrees(alpha_0), 'go', label='Alpha_0 (Minimum)')    # Plot alpha_0 as a single point
    text = str(round(np.degrees(alpha_0),2))    # Create label text for point
    text += " deg"                  # Add unit to label text
    # Label point alpha_0:
    ax.annotate(text,        
                       (np.degrees(phi_0),np.degrees(alpha_0)), # Position of point
                       textcoords="offset points", 
                       xytext=(3,15),       # Text position relative to point
                       ha='center',         # Text alignment
                       )
    ax.set(xlabel='Phi [deg]')   # Label x-axis
    ax.set(ylabel='Pressure Angle [deg]')    # Label y-axis
    ax.set(title='Pressure Angle (Alpha) as a function of Phi') # Title plot
    ax.legend()
    ax.grid(True)
    index += 1          # Increment index for next plot

plt.show()