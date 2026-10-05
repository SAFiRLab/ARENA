import csv
import os
import sys

import numpy as np
from matplotlib import pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.colors as mcolors


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
    # cost_id = 'Time'
    cost_id = 'Safety'
    #cost_id = 'Energy'

    current_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'ksl_airport_1', 'reports5')
    
    for a_file in os.listdir(current_path):
        if a_file.endswith('.csv'):
            file = a_file
            break
    
    header, data = open_csv_1_iter_per_set(os.path.join(current_path, file))

    if cost_id == 'Time':
        min_cost = np.min([float(row[header.index("Best time cost")]) for row in data])
    elif cost_id == 'Safety':
        min_cost = np.min([float(row[header.index("Best security cost")]) for row in data])
    elif cost_id == 'Energy':
        min_cost = np.min([float(row[header.index("Best energy cost")]) for row in data])
    print(min_cost)
