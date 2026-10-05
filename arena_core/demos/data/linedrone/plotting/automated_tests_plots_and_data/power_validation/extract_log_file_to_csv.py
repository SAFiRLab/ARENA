## This file is used to extract logs from ardupilot log files and create csv files for further analysis
# This ise usefull when the binary file is corrupted and we need to extract the data from the log file

import csv
import os
import sys

import numpy as np
from matplotlib import pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.colors as mcolors
import matplotlib.patches as mpatches


def extract_log_file_to_csv(log_file_path, csv_file_path, header):
    # Create a csv file
    csv_file = open(csv_file_path, 'w')

    # Write the header
    csv_file.write(header + '\n')

    # Open the log file
    log_file = open(log_file_path, 'r')

    timestamp = -1
    battery_voltage = -1
    battery_current = -1
    rate_r_buffer = []
    rate_p_buffer = []
    rate_a_buffer = []
    had_battery_data = False
    # Read the log file
    for line in log_file:
        # Split the line by ','
        line = line.split(',')

        # Check if the line starts with 'BAT, '
        if line[0] == 'BAT':
            # Battery data is way slower than RATE data, so we need to wait for the next battery data to finish the csv line
            if had_battery_data:
                # Write the csv line
                csv_file.write(str(timestamp) + ',' + str(battery_voltage) + ',' + str(battery_current) + ',' + str(np.mean(rate_r_buffer)) + ',' + str(np.mean(rate_p_buffer)) + ',' + str(np.mean(rate_a_buffer)) + '\n')
                # Reset the buffers
                rate_r_buffer = []
                rate_p_buffer = []
                rate_a_buffer = []
                had_battery_data = False
            
            # Lines of BAT looks like this:
            # 'BAT',TimeUS,Instance,Volt,VoltR,Curr,CurrTot,EnrgTot,Temp,Res,RemPct
            # We're interested in the following fields:
            # TimeUS, Volt, Curr
            # Get the time in milliseconds
            timestamp = float(line[1]) / 1000.0
            # Get the battery voltage
            battery_voltage = float(line[3])
            # Get the battery current
            battery_current = float(line[5])
            had_battery_data = True

        if line[0] == 'RATE':
            # Lines of RATE looks like this:
            # 'RATE',T,RDes,R,ROut,PDes,P,POut,YDes,Y,YOut,ADes,A,AOut,AOuS,LOut,YSOt
            # We're interested in the following fields:
            # T, R, P, A
            # Get the roll rate
            # Check if the text is not NaN
            if line[3] != 'NaN':
                rate_r_buffer.append(float(line[3]))
            # Get the pitch rate
            # Check if the text is not NaN
            if line[6] != 'NaN':
                rate_p_buffer.append(float(line[6]))
            # Get the acceleration rate
            # Check if the text is not NaN
            if line[12] != 'NaN':
                rate_a_buffer.append(float(line[12]))

def extract_mode_from_log_file(log_file_path):
    # Open the log file
    log_file = open(log_file_path, 'r')

    modes_dict = {
        'timestamp': [],
        'mode': []
    }
    # Read the log file
    for line in log_file:
        # Split the line by ','
        line = line.split(',')

        # Check if the line starts with 'MODE'
        if line[0] == 'MODE':
            # Lines of MODE looks like this:
            # 'MODE',TimeUS,Mode,ModeNum
            # We're interested in the following fields:
            # Mode
            # Get the mode
            modes_dict['timestamp'].append(float(line[1]) / 1000.0)
            modes_dict['mode'].append(line[2])

    return modes_dict

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


if __name__ == '__main__':
    # Get the path of the current file
    current_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'data', 'test4')

    # Get the path of the log file
    log_file_path = os.path.join(current_path, '2024-11-21 13-42-04.log')

    # Get the path of the csv file
    csv_file_path = os.path.join(current_path, 'extracted_data.csv')

    # Define the header
    header = 'timestamp(ms),BAT.Volt,BAT.Curr,RATE.R,RATE.P,RATE.A'

    # Extract the log file to csv
    #extract_log_file_to_csv(log_file_path, csv_file_path, header)
    modes_dic = extract_mode_from_log_file(log_file_path)

    # Plot the data
    # Open the csv file
    csv_file = open(csv_file_path, 'r')

    # Extract the data
    header, data = extract_csv_data(csv_file_path)

    # Get the time data
    time = [float(row[header.index('timestamp(ms)')]) for row in data]

    # Get the battery voltage data
    battery_voltage = [float(row[header.index('BAT.Volt')]) for row in data]

    # Get the battery current data
    battery_current = [float(row[header.index('BAT.Curr')]) for row in data]

    # Get the roll rate data
    rate_r = [float(row[header.index('RATE.R')]) for row in data]

    # Get the pitch rate data
    rate_p = [float(row[header.index('RATE.P')]) for row in data]

    # Get the acceleration rate data
    rate_a = [float(row[header.index('RATE.A')]) for row in data]

    # Create a figure
    fig = plt.figure(figsize=(15, 10))
    ax = fig.add_subplot(111)

    # Plot everything in the same plot but with different colors and different y-axis
    ax.plot(time, battery_voltage, color='blue', label='Battery Voltage')
    ax.plot(time, battery_current, color='red', label='Battery Current')
    ax.plot(time, rate_r, color='green', label='Roll Rate')
    ax.plot(time, rate_p, color='orange', label='Pitch Rate')
    ax.plot(time, rate_a, color='purple', label='Acceleration Rate')

    # Plot the modes as vertical lines
    for i in range(1, len(modes_dic['timestamp'])):
        ax.axvline(x=modes_dic['timestamp'][i], color='black', linestyle='--', label='Mode: ' + modes_dic['mode'][i])

    # Set the labels
    ax.set_xlabel('Time (ms)')

    plt.legend()
    plt.show()


