# config.py

# Paramters for the disc
r_p = 38    #[mm] <- Roller position radius
r_rp = 2    #[mm] <- Roller radius
a = 2       #[mm] <- Eccentricity
z_p = 16    #Number of pins
z_c = 15    #Number of disc teeth (usually z_p - 1)

# Parameters to modify the disc 
delta_L_max = 0.3   #[mm] <- Disc Offset in non working segment (aka maximum)
delta_L_0 = 0.1     #[mm] <- Disc Offset in working segment (aka minimum)

# Parameters for the resampling (relevant for .csv export)
num_inital_points = 10000   # Number of initial points, used for the base plot. More points = more accurate resampling, but more processing time
num_resampled_points = 750  # Number of points after resampling. Limited by how many points your CAD package can handle

# Enable Functions
create_csv = False      # Resamples the data and outputs it to a .csv
plot_disc = True        # Plots the original cycloid disc
plot_modified = True    # Plots the modified cycloid disc
plot_resampled = True   # Plots the resampled data vs original data, useful to troubleshoot
plot_alpha = True       # Plots alpha, including alpha_0 (minimum)
