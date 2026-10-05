import csv
import os
import sys

import numpy as np
from matplotlib import pyplot as plt


# Metrics that can be plotted: name -> (label, unit, scale applied to the values for display)
PLOT_METRICS = {
    'feasibility': ('Feasibility rate', '', 1.0),
    'planning_time': ('Planning time', 's', 1.0e-9),  # Stored in nanoseconds in the reports
    'time_cost': ('Time cost', 's', 1.0),
    'security_cost': ('Security cost', '', 1.0),
    'energy_cost': ('Energy cost', 'J', 1.0),
}


def extract_data(data, header):
    out_dict = {
        'feasibility': [],
        'planning_time': [],
        'nb_of_generations': [],
        'population_size': [],
        'nurbs_sample_size': [],
        'time_cost': [],
        'security_cost': [],
        'energy_cost': []
    }

    # Extract the data according to ID
    old_id = -1
    for row in data:
        if row[header.index('Id')] != old_id:
            out_dict['planning_time'].append(float(row[header.index('Planing Time')]))
            #out_dict['final_cost'].append(float(row[header.index('Final cost')]))
            if row[header.index('Time cost')] >= 1e6:
                out_dict['feasibility'].append(0)
            else:
                out_dict['feasibility'].append(1)
            
            out_dict['nb_of_generations'].append(float(row[header.index('Number of generations')]))
            out_dict['population_size'].append(float(row[header.index('Population size')]))
            out_dict['nurbs_sample_size'].append(float(row[header.index('Nurbs sample size')]))
            out_dict['time_cost'].append(float(row[header.index('Time cost')]))
            out_dict['security_cost'].append(float(row[header.index('Security cost')]))
            out_dict['energy_cost'].append(float(row[header.index('Energy cost')]))

            old_id = row[header.index('Id')]

    return out_dict

def find_report(folder):
    # The testbench writes the Pareto fronts next to the report, they don't have the report columns
    for a_file in sorted(os.listdir(folder)):
        if a_file.startswith('report_3d') and a_file.endswith('.csv') and not a_file.endswith('_pareto_front.csv'):
            return a_file
    sys.exit('No report found in {} (only the *_pareto_front.csv file?)'.format(folder))

def open_csv(file):
    with open(file, 'r', newline='\n') as f:
        reader = csv.reader(f, delimiter=',')
        data = []
        header = next(reader)
        for row in reader:
            data.append(row)
    return header, data

def get_data_from_nb_of_solution(data):
    # nb of solutions is the number of generations * population size
    out_dict = {
        'feasibility_mean': [],
        'feasibility_std_dev': [],
        'planning_time_mean': [],
        'planning_time_std_dev': [],
        'time_cost_mean': [],
        'time_cost_std_dev': [],
        'security_cost_mean': [],
        'security_cost_std_dev': [],
        'energy_cost_mean': [],
        'energy_cost_std_dev': [],
    }

    # Extract the data according to old nb of solution
    old_nb_of_points = -1
    nb_of_points_data = []

    feasibility_data = []
    temp_feasibility_data = []

    planning_time_data = []
    temp_planning_time_data = []

    time_cost_data = []
    temp_time_cost_data = []

    security_cost_data = []
    temp_security_cost_data = []

    energy_cost_data = []
    temp_energy_cost_data = []

    for i in range(len(data['planning_time'])):
        if data['nb_of_generations'][i] * data['population_size'][i] * data['nurbs_sample_size'][i] == old_nb_of_points:
            temp_feasibility_data.append(data['feasibility'][i])
            temp_planning_time_data.append(data['planning_time'][i])
            temp_time_cost_data.append(data['time_cost'][i])
            temp_security_cost_data.append(data['security_cost'][i])
            temp_energy_cost_data.append(data['energy_cost'][i])
        else:
            old_nb_of_points = data['nb_of_generations'][i] * data['population_size'][i] * data['nurbs_sample_size'][i]
            nb_of_points_data.append(old_nb_of_points)

            if len(temp_planning_time_data) > 0:
                feasibility_data.append(temp_feasibility_data)
                temp_feasibility_data = []
                planning_time_data.append(temp_planning_time_data)
                temp_planning_time_data = []
                time_cost_data.append(temp_time_cost_data)
                temp_time_cost_data = []
                security_cost_data.append(temp_security_cost_data)
                temp_security_cost_data = []
                energy_cost_data.append(temp_energy_cost_data)
                temp_energy_cost_data = []

            temp_feasibility_data.append(data['feasibility'][i])
            temp_planning_time_data.append(data['planning_time'][i])
            temp_time_cost_data.append(data['time_cost'][i])
            temp_security_cost_data.append(data['security_cost'][i])
            temp_energy_cost_data.append(data['energy_cost'][i])

    feasibility_data.append(temp_feasibility_data)
    temp_feasibility_data = []
    planning_time_data.append(temp_planning_time_data)
    temp_planning_time_data = []
    time_cost_data.append(temp_time_cost_data)
    temp_time_cost_data = []
    security_cost_data.append(temp_security_cost_data)
    temp_security_cost_data = []
    energy_cost_data.append(temp_energy_cost_data)
    temp_energy_cost_data = []

    # Calculate the mean and standard deviation for every value 
    for i in range(len(planning_time_data)):
        out_dict['feasibility_mean'].append(np.mean(feasibility_data[i]))
        out_dict['feasibility_std_dev'].append(np.std(feasibility_data[i]))
        out_dict['planning_time_mean'].append(np.mean(planning_time_data[i]))
        out_dict['planning_time_std_dev'].append(np.std(planning_time_data[i]))
        out_dict['time_cost_mean'].append(np.mean(time_cost_data[i]))
        out_dict['time_cost_std_dev'].append(np.std(time_cost_data[i]))
        out_dict['security_cost_mean'].append(np.mean(security_cost_data[i]))
        out_dict['security_cost_std_dev'].append(np.std(security_cost_data[i]))
        out_dict['energy_cost_mean'].append(np.mean(energy_cost_data[i]))
        out_dict['energy_cost_std_dev'].append(np.std(energy_cost_data[i]))

    return out_dict, nb_of_points_data


if __name__ == '__main__':
    # What do we want to plot:
    # TODO

    # Get the path of the current file
    nb_of_gen_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'CL_map_3', 'report1')
    sample_size_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'CL_map_3', 'report3')
    population_size_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'CL_map_3', 'report2')
    rrt_range_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'CL_map_3', 'report4')

    file = find_report(nb_of_gen_path)
    
    header1, data1 = open_csv(os.path.join(nb_of_gen_path, file))
    data1 = np.array(data1).astype(float)
    data1 = extract_data(data1, header1)

    file = find_report(sample_size_path)
    
    header2, data2 = open_csv(os.path.join(sample_size_path, file))
    data2 = np.array(data2).astype(float)
    data2 = extract_data(data2, header2)

    file = find_report(population_size_path)

    header3, data3 = open_csv(os.path.join(population_size_path, file))
    data3 = np.array(data3).astype(float)
    data3 = extract_data(data3, header3)

    file = find_report(rrt_range_path)

    header4, data4 = open_csv(os.path.join(rrt_range_path, file))
    data4 = np.array(data4).astype(float)
    data4 = extract_data(data4, header4)

    out_dict1, nb_of_solutions_data1 = get_data_from_nb_of_solution(data1)
    out_dict2, nb_of_solutions_data2 = get_data_from_nb_of_solution(data2)
    out_dict3, nb_of_solutions_data3 = get_data_from_nb_of_solution(data3)
    out_dict4, nb_of_solutions_data4 = get_data_from_nb_of_solution(data4)

    # Plot the data
    # Metric shown on the y axis, one of PLOT_METRICS (can also be given as first argument of the script)
    metric = 'planning_time'
    if len(sys.argv) > 1:
        metric = sys.argv[1]
    if metric not in PLOT_METRICS:
        sys.exit('Unknown metric "{}", choose one of: {}'.format(metric, ', '.join(PLOT_METRICS)))

    label, unit, scale = PLOT_METRICS[metric]
    mean = metric + '_mean'
    std_dev = metric + '_std_dev'

    series = [
        (out_dict1, nb_of_solutions_data1, 'Number of generation variation', 'blue'),
        (out_dict2, nb_of_solutions_data2, 'Population size variation', 'green'),
        (out_dict3, nb_of_solutions_data3, 'Nurbs sample size variation', 'red'),
        (out_dict4, nb_of_solutions_data4, 'Number of control points variation', 'purple'),
    ]

    # Plot the data with shaded standard deviation
    for out_dict, nb_of_solutions_data, series_label, color in series:
        out_mean = np.array(out_dict[mean]) * scale
        out_std_dev = np.array(out_dict[std_dev]) * scale

        plt.plot(nb_of_solutions_data, out_mean, label=series_label, color=color)
        plt.fill_between(nb_of_solutions_data,
                         out_mean - out_std_dev,
                         out_mean + out_std_dev,
                         color=color, alpha=0.2)

    # Labels and title
    plt.xlabel('Number of points')
    plt.ylabel('{} ({})'.format(label, unit) if unit else label)
    plt.title('Mean and standard deviation of {}'.format(label.lower()))
    plt.legend()
    plt.show()
