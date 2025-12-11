# config.py

# PARAMETERS
#-------------------------------------------------------------------------------
# Disc Geometry Parameters
r_p = 38    #[mm] <- Roller position radius (Pin circle radius)
r_rp = 3    #[mm] <- Roller radius
a = 2       #[mm] <- Eccentricity
z_p = 16    # Number of pins (Pins in the housing)
z_c = 15    # Number of disc teeth (usually z_p - 1)

# Modification Parameters (Profile Shift)
delta_L_max = 0.3   #[mm] <- Maximum offset in the non-working segment
delta_L_0 = 0.1     #[mm] <- Minimum offset in the working segment

# Data Point Control
num_inital_points = 5000   # Number of initial points for high-precision calculation and DXF export
num_resampled_points = 750  # Number of points after resampling (relevant for CSV export/lower-precision plots)

# FEATURE TOGGLES
#-------------------------------------------------------------------------------
create_csv = False      # Resamples the R_c data and outputs it to a .csv
create_dxf = True       # Exports R_c0 (unmodified) and R_c (modified/production) as DXF NURBS splines (requires 'ezdxf' package)
plot_disc = True        # Plots the R_c0 (unmodified) cycloid disc
plot_modified = True    # Plots the R_c (modified) cycloid disc
plot_resampled = False  # Plots the resampled data vs original R_c data for troubleshooting
plot_alpha = True       # Plots the pressure angle (alpha)