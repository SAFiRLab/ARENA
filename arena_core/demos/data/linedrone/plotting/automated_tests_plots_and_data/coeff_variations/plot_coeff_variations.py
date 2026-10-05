import csv
import os
import sys

import numpy as np
from matplotlib import pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.colors as mcolors


def extract_costs_mean_and_coeffs_from_nb_of_iter(data, header, nb_of_iter):
    out_dict = {
        'time_mean': [],
        'security_mean': [],
        'energy_mean': [],
        'time_coeff': [],
        'security_coeff': [],
        'energy_coeff': [],
    }

    temp_time_costs = []
    temp_security_costs = []
    temp_energy_costs = []

    current_nb_of_iter = 0
    old_id = -1
    sigma = 2
    for row in data:
        if current_nb_of_iter == nb_of_iter:
            # Clean outliers from the data before applying the mean
            mean_time = np.mean(temp_time_costs)
            std_time = np.std(temp_time_costs)
            threshold_high_time = mean_time + std_time
            temp_time_costs = [i for i in temp_time_costs if i <= threshold_high_time]

            mean_security = np.mean(temp_security_costs)
            std_security = np.std(temp_security_costs)
            threshold_high_security = mean_security + std_security
            temp_security_costs = [i for i in temp_security_costs if i <= threshold_high_security]

            mean_energy = np.mean(temp_energy_costs)
            std_energy = np.std(temp_energy_costs)
            threshold_high_energy = mean_energy + std_energy
            temp_energy_costs = [i for i in temp_energy_costs if i <= threshold_high_energy]

            out_dict['time_mean'].append(np.mean(temp_time_costs))
            out_dict['security_mean'].append(np.mean(temp_security_costs))
            out_dict['energy_mean'].append(np.mean(temp_energy_costs))
            out_dict['time_coeff'].append(float(row[header.index('Time coefficient')]))
            out_dict['security_coeff'].append(float(row[header.index('Security coefficient')]))
            out_dict['energy_coeff'].append(float(row[header.index('Energy coefficient')]))
            # Get median instead of mean
            """out_dict['time_mean'].append(np.median(temp_time_costs))
            out_dict['security_mean'].append(np.median(temp_security_costs))
            out_dict['energy_mean'].append(np.median(temp_energy_costs))
            out_dict['time_coeff'].append(float(row[header.index('Time coefficient')]))
            out_dict['security_coeff'].append(float(row[header.index('Security coefficient')]))
            out_dict['energy_coeff'].append(float(row[header.index('Energy coefficient')]))"""
            temp_time_costs = []
            temp_security_costs = []
            temp_energy_costs = []
            current_nb_of_iter = 0
        
        if row[header.index('Id')] != old_id:
            temp_time_costs.append(float(row[header.index('Chosen time cost')]))
            temp_security_costs.append(float(row[header.index('Chosen security cost')]))
            temp_energy_costs.append(float(row[header.index('Chosen energy cost')]))
            old_id = row[header.index('Id')]
            current_nb_of_iter += 1

    return out_dict

def plot_heatmap(X, Y, Z, x_label, y_label, z_label, cmap_label, title):
    # Define outlier threshold
    sigma = 3
    mean_Z = np.mean(Z)
    std_Z = np.std(Z)
    threshold_high = mean_Z + sigma * std_Z

    # Mask outliers for colorbar scaling
    Z_clipped = np.ma.masked_where(Z > threshold_high, Z)
    
    fig, ax = plt.subplots()
    
    # Plot heatmap with outliers masked for colormap scaling
    c = ax.pcolormesh(X, Y, Z_clipped, shading='auto', cmap='PuBuGn', 
                      norm=mcolors.Normalize(vmin=np.min(Z_clipped), vmax=np.max(Z_clipped)))
    
    # Plot outliers in red
    Z_outliers = np.ma.masked_where(Z <= threshold_high, Z)
    from matplotlib.colors import ListedColormap
    single_color_cmap = ListedColormap(['red'])
    ax.pcolormesh(X, Y, Z_outliers, shading='auto', cmap=single_color_cmap)  # 'autumn' cmap gives a red color

    # If there is any outliers we add a text box at the top center of the plot to show the max outlier value
    """if np.any(Z_outliers.mask):
        max_outlier = np.max(Z_outliers)
        ax.text(
            0.5, 
            0.85, 
            f'Max outlier value: {max_outlier:.2f}' + '', 
            color="black",
            ha="center",
            va="center",
            bbox=dict(facecolor=(1, 1, 1, 0.5), edgecolor=(1, 0, 0, 1), linewidth=3), fontsize=20, transform=ax.transAxes)"""
        
    ax.text(
        0.75, 
        0.75, 
        f'Optimal cost', 
        color="white",
        ha="center",
        va="center",
        bbox=dict(facecolor=(0.67, 0.70, 0.77, 1.0), edgecolor=(0.67, 0.70, 0.77, 1.0)), fontsize=20, transform=ax.transAxes
    )

    ax.text(
        0.75, 
        0.69, 
        f'       {optimal_cost:.2f}' + '      ', 
        color=(0.67, 0.70, 0.77, 1.0),
        ha="center",
        va="center",
        bbox=dict(facecolor="white", edgecolor=(0.67, 0.70, 0.77, 1.0)), fontsize=20, transform=ax.transAxes
    )
    

    # Add colorbar without considering outliers
    cbar = plt.colorbar(c, ax=ax)
    cbar.ax.tick_params(labelsize=16)
    cbar.set_label(cmap_label, fontsize=18)

    # Plot the diagonal line
    ax.plot([0, 0.5], [0, 0.5], color='black', lw=3)

    # Add the diagonal line label at the end of the line and oriented with the line
    ax.text(0.5 + 0.075, 0.5 + 0.075, r'$k_{' + z_label + r'} / \Sigma k_i$', color='black', 
            ha='center', va='center', fontsize=18, rotation=45)

    # Define positions for annotations
    positions = [(0.1, 0.1, '0.8'), (0.2, 0.2, '0.6'), (0.3, 0.3, '0.4'), (0.4, 0.4, '0.2')]

    # Draw perpendicular markers and annotations
    for x, y, label in positions:
        ax.plot([x - 0.025, x + 0.025], [y + 0.025, y - 0.025], color='black', lw=3)
        ax.text(x - 0.05, y + 0.05, label, color='black', ha='center', va='center', fontsize=18)

    # Set axis labels
    ax.set_xlabel(r'$k_{' + x_label + r'} / \Sigma k_i$', fontsize=18)
    ax.set_ylabel(r'$k_{' + y_label + r'} / \Sigma k_i$', fontsize=18)

    # Limit axes to the triangular region
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)

    # Set axis tick parameters
    ax.tick_params(axis='both', which='major', labelsize=16)

    # Add title
    ax.set_title(title, fontsize=18)

    # Make the plot bigger and square
    fig.set_size_inches(10, 8)

    plt.show()

def get_cost_coverage(X, Y, Z, optimal_cost, x_threshold, x_threshold2, y_threshold, y_threshold2, z_threshold, z_threshold2):
    # Get the mean value of the cost in the zone defined by the thresholds and compare it to the optimal cost
    # Get the indices of the values in the meshgrid that are in the zone defined by the thresholds
    indices = np.where((X >= x_threshold) & (X <= x_threshold2) & (Y >= y_threshold) & (Y <= y_threshold2))
    # Get the values of the cost in the zone defined by the thresholds
    values = Z[indices]
    # Get the mean value of the cost in the zone defined by the thresholds
    mean_value = np.mean(values)
    # Get the percentage of the optimal cost that the mean value of the cost in the zone defined by the thresholds represents
    percentage = ((mean_value / optimal_cost) * 100) - 100

    return percentage

def open_csv_1_iter_per_set(file):
    with open(file, 'r', newline='\n') as f:
        reader = csv.reader(f, delimiter=',')
        data = []
        header = next(reader)
        for row in reader:
            push = True
            for i in row:
                if float(i) > 1.0e20 or float(i) < -1.0e20:
                    push = False
            if push:
                data.append(row)
    return header, data

if __name__ == '__main__':
    # What do we want to plot:
    #plot_id = 'Time'
    #plot_id = 'Safety'
    plot_id = 'Energy'

    if plot_id == 'Time':
        unit_id = '(s)'
        optimal_cost = 22.976809
    elif plot_id == 'Safety':
        unit_id = ''
        optimal_cost = 0.0
    elif plot_id == 'Energy':
        unit_id = '(J)'
        optimal_cost = 71421.400855

    nb_of_iter = 3

    # Get the path of the current file
    current_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'ksl_airport_1', 'reports2')

    dataset = []
    headers = []
    x = []
    y = []
    z_time = []
    z_security = []
    z_energy = []
    
    for a_file in os.listdir(current_path):
        if a_file.endswith('.csv'):
            file = a_file
            break
    
    header, data = open_csv_1_iter_per_set(os.path.join(current_path, file))

    out_dict = extract_costs_mean_and_coeffs_from_nb_of_iter(data, header, nb_of_iter)
    x = out_dict['security_coeff']
    y = out_dict['time_coeff']

    # Normalize the z values
    z_time = out_dict['time_mean']
    z_security = out_dict['security_mean']
    z_energy = out_dict['energy_mean']

    X, Y = np.mgrid[0:1:2000j, 0:1:2000j]

    # Generates Z with the same shape as X and Y, but with the z_values on each row
    Z_dict = {
        'Time': z_time,
        'Safety': z_security,
        'Energy': z_energy
    }

    from scipy.interpolate import griddata
    # Interpolate Z values over the meshgrid
    Z1 = griddata((x, y), Z_dict[plot_id], (X, Y), method='linear')
    Z2 = griddata((x, y), Z_dict[plot_id], (X, Y), method='nearest')
    Z3 = griddata((x, y), Z_dict[plot_id], (X, Y), method='cubic')

    # Only take the Z values where X + Y <= 1
    Z_masked1 = np.ma.masked_where(X + Y > 1, Z1)
    Z_masked2 = np.ma.masked_where(X + Y > 1, Z2)
    Z_masked3 = np.ma.masked_where(X + Y > 1, Z3)

    if plot_id == 'Safety':
        coverage = get_cost_coverage(X, Y, Z_masked2, optimal_cost, 0.6, 1.0, 0.0, 0.4, 0.0, 0.4)
    elif plot_id == 'Time':
        coverage = get_cost_coverage(X, Y, Z_masked2, optimal_cost, 0.0, 0.2, 0.0, 1.0, 0.0, 1.0)
    elif plot_id == 'Energy':
        coverage = get_cost_coverage(X, Y, Z_masked2, optimal_cost, 0.0, 0.2, 0.0, 1.0, 0.0, 1.0)
    print(f'Coverage: {coverage:.2f}%')
    
    plot_heatmap(X, Y, Z_masked2, 'S', 'T', 'E', plot_id + ' cost '+ unit_id, plot_id + ' cost for different set of coefficients')

    plt.show()



