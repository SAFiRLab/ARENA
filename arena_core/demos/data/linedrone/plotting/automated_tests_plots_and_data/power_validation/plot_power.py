import csv
import os
import sys
import yaml

import numpy as np
from matplotlib import pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.colors as mcolors
import matplotlib.patches as mpatches
from sympy import symbols, Eq, solve, N
from scipy.optimize import curve_fit as scipy_curve_fit

from distance_to_quadric_surface import distance_to_empirical_quadric_surface


def extract_csv_data(file):
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

def extract_yaml_data(file):
    with open(file, 'r') as f:
        yaml_data = yaml.safe_load(f)
    return yaml_data

def fit_curve(data, x_row, y_row):
    def func(x, a, b, c, d):
        return a * x**3 + b * x**2 + c * x + d
    
    x = np.array([float(row[header.index(x_row)]) for row in data])
    y = np.array([float(row[header.index(y_row)]) for row in data])

    popt, pcov = scipy_curve_fit(func, x, y)

    # Compute the curve at every point
    curve = func(x, *popt)

    # Switch the curve's y values with the data's y values to compensate for the battery discharge
    # The first point is the last point of the current curve, the second points is the point before the last point of the current curve and so on
    curve = curve[::-1]
    # Scale this back to 0
    curve = curve - curve[0]
    
    # Plot
    #plt.plot(x, y, 'o')
    #plt.plot(x, curve, label='fit')
    #plt.show()

    return curve

def define_tolerance_ranges(data, header, start_time_stamp, end_time_stamp):
    # Define the tolerance ranges for the different attitude rates
    # This will help us to determine the acceptable rates values to be considered in steady state

    out_dict = {
        'pitch_rate_tolerance': 0.0,
        'roll_rate_tolerance': 0.0,
        'z_rate_tolerance': 0.0
    }

    pitch_rates = []
    roll_rates = []
    z_rates = []
    
    # Analyse the noise level when the drone is not moving
    for row in data:
        if float(row[header.index('timestamp(ms)')]) < start_time_stamp:
            continue
        if float(row[header.index('timestamp(ms)')]) > end_time_stamp:
            break
        pitch_rates.append(float(row[header.index('RATE.P')]))
        roll_rates.append(float(row[header.index('RATE.R')]))
        z_rates.append(float(row[header.index('RATE.A')]))

    # Compute the standard deviation of the rates to define the tolerance ranges at 3 sigma for acceleration rates and 4 sigma for angular rates
    #accel_sigma = 3
    sigma = 2.0
    out_dict['pitch_rate_tolerance'] = sigma * np.std(pitch_rates)
    out_dict['roll_rate_tolerance'] = sigma * np.std(roll_rates)
    out_dict['z_rate_tolerance'] = sigma * np.std(z_rates)

    return out_dict

def extract_time_stamps_from_steady_state(data, header, period, disrupting_period, tolerance_ranges, first_time_stamp, end_time_stamp):
    # Extract the time stamps of the steady state periods
    # The steady state period is defined as a period where the attitude rates are within the tolerance ranges

    steady_state_stamps = []
    steady_state_indexes = []

    pitch_rate_tolerance = tolerance_ranges['pitch_rate_tolerance']
    roll_rate_tolerance = tolerance_ranges['roll_rate_tolerance']
    z_rate_tolerance = tolerance_ranges['z_rate_tolerance']

    old_time_stamp = first_time_stamp
    period_counter = 0    # ms
    disrupting_period_counter = 0   # ms
    was_in_steady_state = False
    for row in data:
        if float(row[header.index('timestamp(ms)')]) > end_time_stamp - 1000:
            break
        if float(row[header.index('timestamp(ms)')]) < first_time_stamp + 1000:
            continue

        if abs(float(row[header.index('RATE.P')])) < pitch_rate_tolerance and \
           abs(float(row[header.index('RATE.R')])) < roll_rate_tolerance and \
           abs(float(row[header.index('RATE.A')])) < z_rate_tolerance:
            period_counter += float(row[header.index('timestamp(ms)')]) - old_time_stamp
            old_time_stamp = float(row[header.index('timestamp(ms)')])

            if period_counter >= period:
                steady_state_stamps.append(old_time_stamp)
                steady_state_indexes.append(data.index(row))
                was_in_steady_state = True
                disrupting_period_counter = 0
        else:
            if was_in_steady_state:
                period_counter += float(row[header.index('timestamp(ms)')]) - old_time_stamp
                disrupting_period_counter += float(row[header.index('timestamp(ms)')]) - old_time_stamp

                if disrupting_period_counter >= disrupting_period:
                    if period_counter >= period:
                        steady_state_stamps.append(float(row[header.index('timestamp(ms)')]))
                        steady_state_indexes.append(data.index(row))

                    was_in_steady_state = False
                    disrupting_period_counter = 0
                    period_counter = 0
            
            old_time_stamp = float(row[header.index('timestamp(ms)')])
            if period_counter >= period:
                steady_state_stamps.append(old_time_stamp)
                steady_state_indexes.append(data.index(row))
            
            period_counter = 0

    return steady_state_stamps, steady_state_indexes

def get_steady_state_powers(data, header, tolerance_ranges, steady_state_periods, steady_state_disrupting_periods, start_timestamps, end_timestamps, plot_bool=False):
    # Start a plot for each unit vector showing the period's data vs the steady state data
    # Each subplots will show different data: pitch_rate, roll_rate, z_rate
    if plot_bool:
        fig = plt.figure()
        gs = gridspec.GridSpec(3, 1)
        ax1 = fig.add_subplot(gs[0])
        ax2 = fig.add_subplot(gs[1])
        ax3 = fig.add_subplot(gs[2])

    # Fit a logarithmic curve to the voltage and current data to compensate for the battery discharge
    volt_discharge_curve = fit_curve(data, 'timestamp(ms)', 'BAT.Volt')

    steady_state_stamps = []
    steady_state_indexes = []
    for iter in range(len(steady_state_periods)):
        temp_stamps, temp_indexes = extract_time_stamps_from_steady_state(data, header, steady_state_periods[iter], steady_state_disrupting_periods[iter], 
                                                                          tolerance_ranges, start_timestamps[iter], end_timestamps[iter])
        steady_state_stamps.append(temp_stamps)
        steady_state_indexes.append(temp_indexes)

    if plot_bool:
        # Plot the data
        ax1.plot([float(row[header.index('timestamp(ms)')]) for row in data], [float(row[header.index('RATE.P')]) for row in data], label='pitch_rate')
        ax2.plot([float(row[header.index('timestamp(ms)')]) for row in data], [float(row[header.index('RATE.R')]) for row in data], label='roll_rate')
        ax3.plot([float(row[header.index('timestamp(ms)')]) for row in data], [float(row[header.index('RATE.A')]) for row in data], label='z_rate')

    temp_instant_powers = []
    # Plot the steady state periods
    for i in range(len(steady_state_indexes)):
        if plot_bool:
            ax1.axvline(x=start_timestamps[i], color='g', linestyle='--')
            ax1.axvline(x=end_timestamps[i], color='r', linestyle='--')
            ax2.axvline(x=start_timestamps[i], color='g', linestyle='--')
            ax2.axvline(x=end_timestamps[i], color='r', linestyle='--')
            ax3.axvline(x=start_timestamps[i], color='g', linestyle='--')
            ax3.axvline(x=end_timestamps[i], color='r', linestyle='--')
        
        # Sort the indices to ensure they are in order
        steady_state_segment = sorted(steady_state_indexes[i])

        # Split into consecutive segments
        segments = []
        current_segment = [steady_state_segment[0]]

        for j in range(1, len(steady_state_segment)):
            # Check if the current index is consecutive to the previous one
            if steady_state_segment[j] == steady_state_segment[j - 1] + 1:
                current_segment.append(steady_state_segment[j])
            else:
                # If not consecutive, add the current segment to segments and start a new segment
                segments.append(current_segment)
                current_segment = [steady_state_segment[j]]
        segments.append(current_segment)  # Add the last segment

        # Plot each consecutive segment separately
        temp_powers = []
        for segment in segments:
            #temp = ([(float(data[idx][header.index('BAT.Volt')]) + volt_discharge_curve[idx]) * float(data[idx][header.index('BAT.Curr')]) for idx in segment])
            temp = ([float(data[idx][header.index('BAT.Volt')]) * float(data[idx][header.index('BAT.Curr')]) for idx in segment])
            for i in temp:
                temp_powers.append(i)

            if plot_bool:
                ax1.plot([float(data[idx][header.index('timestamp(ms)')]) for idx in segment], [float(data[idx][header.index('RATE.P')]) for idx in segment], label='pitch_rate_steady_state', color='r')
                ax2.plot([float(data[idx][header.index('timestamp(ms)')]) for idx in segment], [float(data[idx][header.index('RATE.R')]) for idx in segment], label='roll_rate_steady_state', color='r')
                ax3.plot([float(data[idx][header.index('timestamp(ms)')]) for idx in segment], [float(data[idx][header.index('RATE.A')]) for idx in segment], label='z_rate_steady_state', color='r')

        if plot_bool:
            # Plot the tolerance ranges
            ax1.axhline(y=tolerance_ranges['pitch_rate_tolerance'], color='b', linestyle='--')
            ax2.axhline(y=tolerance_ranges['roll_rate_tolerance'], color='b', linestyle='--')
            ax3.axhline(y=tolerance_ranges['z_rate_tolerance'], color='b', linestyle='--')
            ax1.axhline(y=-tolerance_ranges['pitch_rate_tolerance'], color='b', linestyle='--')
            ax2.axhline(y=-tolerance_ranges['roll_rate_tolerance'], color='b', linestyle='--')
            ax3.axhline(y=-tolerance_ranges['z_rate_tolerance'], color='b', linestyle='--')

        temp_instant_powers.append(temp_powers)

    # Remove outliers from the instant powers
    instant_powers = []
    for temp in temp_instant_powers:
        instant_powers.append(remove_outliers(temp))
        
    if plot_bool:
        # Set the titles
        ax1.set_title('Pitch rate')
        ax2.set_title('Roll rate')
        ax3.set_title('Z rate')

        # Set the labels roll and pitch are in degrees per second, z is in m/s^2
        ax1.set_xlabel('Time (ms)')
        ax1.set_ylabel('Pitch rate (deg/s)')
        ax2.set_xlabel('Time (ms)')
        ax2.set_ylabel('Roll rate (deg/s)')
        ax3.set_xlabel('Time (ms)')
        ax3.set_ylabel('Z rate (m/s^2)')

        # Set spaces between subplots
        plt.tight_layout()

        # Upscale the plot
        fig.set_size_inches(18.5, 10.5)

        # Add the legend manually and place it in the upper left corner outside the plot so it doesn't hide the data
        ax1.legend(['pitch_rate', 'start_time_stamp', 'end_time_stamp', 'pitch_rate_steady_state'], loc='upper right')
        ax2.legend(['roll_rate', 'start_time_stamp', 'end_time_stamp', 'roll_rate_steady_state'], loc='upper right')
        ax3.legend(['z_rate', 'start_time_stamp', 'end_time_stamp', 'z_rate_steady_state'], loc='upper right')

        plt.show()

    return instant_powers

def remove_outliers(data):
    # Remove outliers from the data
    # Outliers are defined as values that are 3 sigma away from the mean
    # data is a list of floats
    data = np.asarray(data).flatten()
    mean = np.mean(data)
    std_dev = np.std(data)
    sigma = 3
    return [x for x in data if x < mean + sigma * std_dev and x > mean - sigma * std_dev]

def plot_quadric_surface(full_power_data, unit_vectors, step=1, plot_bool=False):
    # Define the range of the powers
    min_max_range = abs(np.max(full_power_data) - np.min(full_power_data))
    max_value = np.max(full_power_data).astype(int)

    # Define indices and points
    indices = [0, 1, 8, 9, 10, 12, 16, 40, 41, 2, 10, 12, 16, 20, 21]
    #indices = [0, 1, 8, 9, 40, 41]
    points = [
        [full_power_data[i] * unit_vectors[i][j] for j in range(3)]
        for i in indices
    ]

    # Create the matrix M
    M = np.array([
        [
            p[0]**2, p[1]**2, p[2]**2, p[0], p[1], p[2]
        ]
        for p in points
    ])

    # Right-hand side of the equation is a -1 vector
    """b = -np.ones(M.shape[0])

    A, B, C, G, H, I = symbols('A B C G H I')
    unknowns = [A, B, C, G, H, I]

    # Define the six equations based on the user's points
    equations = [Eq(M[i] @ unknowns, b[i]) for i in range(M.shape[0])]

    # Solve the system of equations
    solution = solve(equations, unknowns)

    # Convert the symbolyc solution to numerical values
    solution = {key: float(N(value)) for key, value in solution.items()}

    print(solution)"""

    from scipy.optimize import least_squares
    
    # Define the quadric surface equation
    def quadric_equation(coeffs, point):
        x, y, z = point
        A, B, C, G, H, I = coeffs
        return (
            A * x**2 + B * y**2 + C * z**2 +
            G * x + H * y + I * z + 1.0
        )

    # Residual function: Evaluate Q(x, y, z) for all points
    def residuals_function(coeffs):
        return np.array([quadric_equation(coeffs, p) for p in points])

    # Initial guess for the coefficients
    initial_guess = np.ones(M.shape[1])

    # Perform least squares optimization
    result = least_squares(residuals_function, initial_guess)

    # Extract optimized coefficients
    solution = result.x

    # Print the solution
    print(solution)
    
    if plot_bool:
        # 3D plot of the quadractic surface based on the solution's coefficients
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

    x = range(-max_value, max_value, step)
    y = range(-max_value, max_value, step)
    X, Y = np.meshgrid(x, y)

    # quadratic formula
    a = solution[2]
    b = solution[5]
    c = solution[0] * X**2.0 + solution[1] * Y**2.0 + solution[3] * X + solution[4] * Y + 1.0
    delta = b**2 - 4 * a * c
    Z1 = (-b + np.sqrt(delta)) / (2.0 * a)
    Z2 = (-b - np.sqrt(delta)) / (2.0 * a)

    if plot_bool:
        # Plot the surface with a transparency of 0.5 and different colors for the two surfaces
        ax.plot_surface(X, Y, Z1, alpha=0.5, color='blue')
        ax.plot_surface(X, Y, Z2, alpha=0.5, color='green')

    distances = []
    closest_points = []
    iter = 0
    #iter_to_pass = [0, 1, 8, 9, 40, 41]
    # Use the unit vectors to plot the full power data
    for unit_vector in unit_vectors:
        # Make sure the unit vector is normalized
        unit_vector = np.array(unit_vector) / np.linalg.norm(unit_vector)
            
        # Plot the full power data with a scatter plot with size 10
        point = np.array([unit_vector[0] * full_power_data[iter], unit_vector[1] * full_power_data[iter], unit_vector[2] * full_power_data[iter]])

        if plot_bool:
            ax.scatter(point[0], point[1], point[2], s=50, color='black')

        #if iter not in iter_to_pass:
        # Plot the distance from the quadric surface to the point with a black line
        distance1 = distance_to_empirical_quadric_surface(point, Z1, X, Y)
        distance2 = distance_to_empirical_quadric_surface(point, Z2, X, Y)

        # get the closest point from the two surfaces
        if distance1['distance'] < distance2['distance']:
            distance = distance1
            closest_points.append(distance1['closest_point'])
            # Get sign of the distance to know if the point is above or below the quadric surface
            if distance != 0:
                if np.linalg.norm(point) <= np.linalg.norm(distance['closest_point']):
                    distance['distance'] = -abs(distance['distance'])
                else:
                    distance['distance'] = abs(distance['distance'])
        else:
            distance = distance2
            closest_points.append(distance2['closest_point'])
            # Get sign of the distance to know if the point is above or below the quadric surface
            if distance != 0:
                if np.linalg.norm(point) <= np.linalg.norm(distance['closest_point']):
                    distance['distance'] = -abs(distance['distance'])
                else:
                    distance['distance'] = abs(distance['distance'])
        
        print("Distance to quadric surface from point  (" + str(point[0]) + ", " + str(point[1]) + ", " + str(point[2]) + ") test number " + str(iter) + " :")
        print(distance)
        distances.append(distance)

        if plot_bool:
            ax.plot([point[0], distance['closest_point'][0]], [point[1], distance['closest_point'][1]], [point[2], distance['closest_point'][2]], color='black')
        #else:
        #    closest_points.append(point)

        iter += 1

    # Get mean distance
    mean_distance = np.mean([abs(distance['distance']) for distance in distances])
    print("Mean distance to quadric surface: " + str(mean_distance))

    # Represent with a metric how bad is the mean distance against the min power value of the quadric surface
    print("Mean distance to quadric surface against min power value: " + str(mean_distance / np.min(full_power_data) * 100) + "%")

    if plot_bool:
        # Set the labels
        ax.set_xlabel('Steady-state power roll axis', fontsize=18, labelpad=20)
        ax.set_ylabel('Steady-state power pitch axis', fontsize=18, labelpad=20)
        ax.set_zlabel('Steady-state power Z axis', fontsize=18, labelpad=20)

        # Set the title
        ax.set_title('Power consumption model and empirical data visualization', fontsize=18)

        # Custom legend patches
        red_patch = mpatches.Patch(color='blue', label='Descending quadric surface')
        blue_patch = mpatches.Patch(color='green', label='Ascending quadric surface')
        black_patch = mpatches.Patch(color='black', label='Empirical data')

        # Add custom legend with colored patches
        ax.legend(handles=[blue_patch, red_patch, black_patch], loc='upper left', fontsize=16)

        # Add mean distance value and the ratio against the min power value of the quadric surface to the plot informations
        # The text is placed in the upper right corner
        # 2 decimals are used for the mean distance and 4 decimals for the ratio
        ax.text2D(0.60, 0.95, "Mean distance to quadric surface: " + str(round(mean_distance, 2)) + "W", transform=ax.transAxes, fontsize=16)
        ax.text2D(0.60, 0.87, "Max distance to quadric surface against min/max\npower difference: " + 
                  str(round(np.max([abs(dist['distance']) for dist in distances]) / abs(np.min(full_power_data) - np.max(full_power_data)) * 100, 2)) + 
                  "%", transform=ax.transAxes, fontsize=16)

        # Resizing the plot to make it bigger
        fig.set_size_inches(18.5, 10.5)

        # Make axis values bigger
        ax.tick_params(axis='both', which='major', labelsize=16)

        plt.show()

    return distances, min_max_range, closest_points

def plot_bland_altman_graph(errors, min_max_range, full_power_data, quadric_surface_closest_points_data, plot_bool=False):
    #iter_to_pass = [0, 1, 8, 9, 40, 41]
    temp_full_power_data = []
    #temp_full_power_data2 = []
    temp_quadric_surface_closest_points_data = []
    #temp_quadric_surface_closest_points_data2 = []
    for i in range(len(full_power_data)):
        #if i not in iter_to_pass:
        temp_full_power_data.append(full_power_data[i])
        temp_quadric_surface_closest_points_data.append(quadric_surface_closest_points_data[i])
        #else:
        #    temp_full_power_data2.append(full_power_data[i])
        #    temp_quadric_surface_closest_points_data2.append(quadric_surface_closest_points_data[i])
    
    x = (np.add(temp_full_power_data, [np.linalg.norm(closest_point) for closest_point in temp_quadric_surface_closest_points_data])) / 2
    y = np.subtract(temp_full_power_data, [np.linalg.norm(closest_point) for closest_point in temp_quadric_surface_closest_points_data])

    ascending_power = x[0]
    descending_power = x[1]
    pitch_roll_powers = [x[8], x[9], x[40], x[41]]
    pitch_roll_ascending_powers = [x[2], x[4], x[6], x[8], x[10], x[12], x[14], x[16], x[18], x[20], x[22], x[24], x[26], x[28], x[30], x[32], x[34], x[36], x[38]]
    pitch_roll_descending_powers = [x[3], x[5], x[7], x[9], x[11], x[13], x[15], x[17], x[19], x[21], x[23], x[25], x[27], x[29], x[31], x[33], x[35], x[37], x[39]]
    pitch_roll_ascending_power_mean = np.mean(pitch_roll_ascending_powers)
    pitch_roll_descending_power_mean = np.mean(pitch_roll_descending_powers)
    pitch_roll_ascending_power_std_dev = np.std(pitch_roll_ascending_powers)
    pitch_roll_descending_power_std_dev = np.std(pitch_roll_descending_powers)

    #x2 = (np.add(temp_full_power_data2, [np.linalg.norm(closest_point) for closest_point in temp_quadric_surface_closest_points_data2])) / 2
    #y2 = np.subtract(temp_full_power_data2, [np.linalg.norm(closest_point) for closest_point in temp_quadric_surface_closest_points_data2])

    #mean_power = np.mean(temp_full_power_data2)
    mean_power = np.mean(temp_full_power_data)
    empirical_error_in_function_of_mean_power = np.subtract(temp_full_power_data, mean_power)
    #empirical_error_in_function_of_mean_power2 = np.subtract(temp_full_power_data2, mean_power)

    # Compute the mean error and the standard deviation of the error
    mean_error = np.mean([error['distance'] for error in errors])
    std_dev_error = np.std([error['distance'] for error in errors])

    if plot_bool:
        # Plot the Bland-Altman graph
        fig, ax = plt.subplots()
        scatter_plot = ax.scatter(x, y, color='black')

    #ax.scatter(x, empirical_error_in_function_of_mean_power, color='blue')
    #ax.scatter(x2, empirical_error_in_function_of_mean_power2, color='blue')

    mean_2 = np.mean([abs(power) for power in empirical_error_in_function_of_mean_power])
    print("Mean error 2: " + str(mean_2))
    #ax.axhline(y=mean_2, color='orange', linestyle='--')

    limit_of_agreement = 1.645 * std_dev_error
    if plot_bool:
        # Plot the mean error
        mean_line = ax.axhline(y=mean_error, color='r', linestyle='--', linewidth=3)  # Mean line

        # Put std dev in shades of grey around the mean error
        #x_all = np.concatenate((x, x2))
        x_all = np.sort(x)
        std_fill = ax.fill_between(x_all, mean_error - std_dev_error, mean_error + std_dev_error, color='grey', alpha=0.5)  # Std dev fill

        # Plot the limit of agreement
        #ax.axhline(y=mean_error + limit_of_agreement, color='g', linestyle='--', linewidth=3)

        # Plot the min and max empirical power consumptions
        min_max_line = ax.axhline(y=abs(min_max_range / 2.0), color='black', linestyle='--', linewidth=3)  # Min/Max line
        #ax.axhline(y=max(full_power_data), color='b', linestyle='--')

        # Complete plots with things we don's want to be in the legend
        #ax.scatter(x2, y2, color='red')
        #ax.axhline(y=mean_error - limit_of_agreement, color='g', linestyle='--', linewidth=3)
        ax.axhline(y=-abs(min_max_range / 2.0), color='black', linestyle='--', linewidth=3)
        #ax.axhline(y=-min(full_power_data), color='b', linestyle='--')
        ax.axvline(x=ascending_power, color='r', linewidth=3)
        ax.axvline(x=descending_power, color='r', linewidth=3)
        for pitch_roll_power in pitch_roll_powers:
            ax.axvline(x=pitch_roll_power, color='g', linewidth=3)
        ax.axvspan(pitch_roll_ascending_power_mean - pitch_roll_ascending_power_std_dev, pitch_roll_ascending_power_mean + pitch_roll_ascending_power_std_dev, color='g', alpha=0.1, label="Horizontal maneuvers with ascending")
        ax.axvspan(pitch_roll_descending_power_mean - pitch_roll_descending_power_std_dev, pitch_roll_descending_power_mean + pitch_roll_descending_power_std_dev, color='b', alpha=0.1, label="Horizontal maneuvers with descending")
        ax.text(
            pitch_roll_ascending_power_mean,
            750,  # Adjust Y position for better placement
            "Pitch/Roll with +Z \n mean and std dev",
            color="green",
            ha="center",
            va="center",
            bbox=dict(facecolor='white', alpha=0.7), fontsize=24
        )

        ax.text(
            pitch_roll_descending_power_mean,
            750,
            "Pitch/Roll with -Z \n mean and std dev",
            color="blue",
            ha="center",
            va="center",
            bbox=dict(facecolor='white', alpha=0.7), fontsize=24
        )

        line_text = "Pitch/Roll maneuvers"
        ax.text(
            sum(pitch_roll_powers) / len(pitch_roll_powers),  # Average x position
            -775,  # Y position
            line_text,
            color="green",
            ha="center",
            bbox=dict(facecolor='white', alpha=0.7), fontsize=24
        )

        # Draw connecting lines to the label
        for power in pitch_roll_powers:
            ax.plot([power, sum(pitch_roll_powers) / len(pitch_roll_powers)], [-600, -675], color='green', lw=3)

        # Explain that for the first pitch/roll maneuver, we had the wind in the back
        line_text = "Same direction\nas the wind"
        ax.text(
            pitch_roll_powers[3] - 25,
            -400,
            line_text,
            color="green",
            ha="right",
            bbox=dict(facecolor='white', alpha=0.7), fontsize=24
        )
        # Connect the text to the first pitch/roll maneuver
        ax.plot([pitch_roll_powers[3], pitch_roll_powers[3] - 20], [-300, -310], color='green', lw=3)

        # Add text for red lines
        ax.text(
            ascending_power,
            1000,
            "Ascending maneuver",
            color="red",
            ha="center",
            bbox=dict(facecolor='white', alpha=0.7), fontsize=24
        )
        ax.text(
            descending_power,
            1000,
            "Descending maneuver",
            color="red",
            ha="center",
            bbox=dict(facecolor='white', alpha=0.7), fontsize=24
        )


    min_max_range_diff = abs(min_max_range)
    loa_diff = abs(mean_error + limit_of_agreement) + abs(mean_error - limit_of_agreement)
    #print("Limit of agreement: " + str(loa_diff))
    print("Min and max empirical power consumptions: " + str(min_max_range_diff))
    #print("Ratio of the limit of agreement against the min and max empirical power consumptions: " + str(loa_diff / min_max_range_diff * 100) + "%")

    if plot_bool:
        # Set the labels
        ax.set_xlabel('(Empirical data - Model closest point data) / 2 (W)', fontsize=24)
        ax.set_ylabel('Empirical data - Model closest point data (W)', fontsize=24)
        # Set axis tick parameters
        ax.tick_params(axis='both', which='major', labelsize=24)

        # Set the title
        ax.set_title('Bland-Altman graph of the power consumption model and empirical data', fontsize=24)

        # Add the legend explicitly with handles and labels
        ax.legend(
            handles=[
                scatter_plot,  # Handle for scatter plot
                mean_line,     # Handle for mean line
                plt.Line2D([], [], color='grey', alpha=0.5, linewidth=10),  # Custom handle for std dev fill
                min_max_line   # Handle for min/max line
            ],
            labels=[
                'Empirical - Model data',
                'Mean error',
                'Std dev',
                'Min/Max Empirical range'
            ],
            loc='upper center',
            fontsize=24,
            ncol=2
        )

        # Set limits for the x and y axis
        ax.set_ylim(-800, 1500)

        # Make the plot bigger
        fig.set_size_inches(18.5, 10.5)

        plt.show()


if __name__ == '__main__':
    # Get the path of the current file
    current_path1 = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'data', 'test3')
    current_path2 = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'data', 'test4')
    
    for a_file in os.listdir(current_path1):
        if a_file.endswith('.csv'):
            csv_file1 = a_file
        if a_file.endswith('.yaml'):
            yaml_file1 = a_file
    
    for a_file in os.listdir(current_path2):
        if a_file.endswith('.csv'):
            csv_file2 = a_file
        if a_file.endswith('.yaml'):
            yaml_file2 = a_file
    
    header1, data1 = extract_csv_data(os.path.join(current_path1, csv_file1))
    header2, data2 = extract_csv_data(os.path.join(current_path2, csv_file2))

    # Make sure data2 starts at 0 ms
    data2 = [[str(float(row[header2.index('timestamp(ms)')]) - float(data2[0][header2.index('timestamp(ms)')]))] + row[1:] for row in data2]

    # Move the data2's timestamp to the end of the data1's last timestamp
    data2 = [[str(float(row[header2.index('timestamp(ms)')]) + float(data1[-1][header1.index('timestamp(ms)')]))] + row[1:] for row in data2]

    data = data1 + data2
    header = header1

    # Get the yaml data
    yaml_data1 = extract_yaml_data(os.path.join(current_path1, yaml_file1))
    yaml_data2 = extract_yaml_data(os.path.join(current_path2, yaml_file2))

    unit_vectors = yaml_data1['unit_vectors'] + yaml_data2['unit_vectors']
    # Make sure the unit vectors are normalized
    unit_vectors = [np.array(unit_vector) / np.linalg.norm(unit_vector) for unit_vector in unit_vectors]

    test_time1 = yaml_data1['test_time']
    test_time2 = yaml_data2['test_time']
    # Make sure the test times are the same
    if test_time1 != test_time2:
        print("The test times are different")
        sys.exit(1)
    test_time = test_time1
    
    nb_of_tests = len(unit_vectors)
    
    start_time_stamps1 = yaml_data1['start_test_timestamps']
    start_time_stamps2 = yaml_data2['start_test_timestamps']
    first_timestamp2 = 285156.0
    start_time_stamps2 = [start_time_stamps2[i] - first_timestamp2 for i in range(len(start_time_stamps2))]

    # Add, data2's first timestamp to all the start_time_stamps
    start_time_stamps = start_time_stamps1 + [start_time_stamps2[i] + float(data2[0][header2.index('timestamp(ms)')]) for i in range(len(start_time_stamps2))]
    # For end_time_stamp, add test_time to start_time_stamp
    end_time_stamps = [start_time_stamps[i] + test_time for i in range(len(start_time_stamps))]

    hovering_end_timestamp = yaml_data1['hovering_end_timestamp']
    hovering_start_timestamp = yaml_data1['hovering_start_timestamp']
    tolerance_ranges = define_tolerance_ranges(data, header, hovering_start_timestamp, hovering_end_timestamp)
    #print(tolerance_ranges)

    steady_state_periods = yaml_data1['steady_state_periods'] + yaml_data2['steady_state_periods']
    steady_state_disrupting_periods = yaml_data1['steady_state_disrupting_periods'] + yaml_data2['steady_state_disrupting_periods']

    instant_powers = get_steady_state_powers(data, header, tolerance_ranges, steady_state_periods, steady_state_disrupting_periods, start_time_stamps, end_time_stamps, plot_bool=False)

    # Plot the quadric surface
    errors, min_max_ranges, closest_points = plot_quadric_surface([np.mean(powers) for powers in instant_powers], unit_vectors, step=10, plot_bool=False)
    
    # Plot the Bland-Altman graph
    plot_bland_altman_graph(errors, min_max_ranges, [np.mean(powers) for powers in instant_powers], closest_points, True)
