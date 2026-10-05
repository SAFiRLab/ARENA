import csv
import os
import sys

import numpy as np
from matplotlib import pyplot as plt


def open_csv(file):
    with open(file, 'r', newline='\n') as f:
        reader = csv.reader(f, delimiter=',')
        data = []
        header = next(reader)
        for row in reader:
            data.append(row)
    return header, data

def compute_energy_consumption(voltages, currents, timestamps):
    # Compute the energy consumption as the area under the power over time curve
    energy_consumption = 0
    for i in range(1, len(voltages)):
        # Compute the power
        power = voltages[i] * currents[i]
        # Compute the time difference
        time_diff = (timestamps[i] - timestamps[i-1]) / 1000
        # Compute the energy consumption
        energy_consumption += power * time_diff
    return energy_consumption

if __name__ == '__main__':
    # Get the path of the current file
    data_folder_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'data', 'csv')

    header, dataset = open_csv(os.path.join(data_folder_path, '3-Test_loit_4ms.csv'))

    # Evaluate the energy consumption at different timestamps and specific actions during the flight
    # The actions are:
    # - Hovering to ascending
    # - Hovering to roll
    # - Hovering to pitch
    # - Hovering to descending

    """
    ############## For 1-Test_loit_0p5ms.csv: ##############
    # - Hovering to ascending: 338153.184ms (indice: 363) to 343653.184ms (indice: 418) for the whole ascending phase
    # Transient state: 338153.184ms to 339953.184ms (indice: 363 to 381)
    # Steady state: 339953.184ms to 343653.184ms (indice: 381 to 418)
    transient_state_ascending_voltage = [float(dataset[i][header.index('BAT.Volt')]) for i in range(363, 381)]
    transient_state_ascending_current = [float(dataset[i][header.index('BAT.Curr')]) for i in range(363, 381)]
    transient_state_ascending_timestamp = [float(dataset[i][header.index('timestamp(ms)')]) for i in range(363, 381)]
    steady_state_ascending_voltage = [float(dataset[i][header.index('BAT.Volt')]) for i in range(381, 418)]
    steady_state_ascending_current = [float(dataset[i][header.index('BAT.Curr')]) for i in range(381, 418)]
    steady_state_ascending_timestamp = [float(dataset[i][header.index('timestamp(ms)')]) for i in range(381, 418)]

    transient_state_ascending_energy_consumption = compute_energy_consumption(transient_state_ascending_voltage, transient_state_ascending_current, transient_state_ascending_timestamp)
    steady_state_ascending_energy_consumption = compute_energy_consumption(steady_state_ascending_voltage, steady_state_ascending_current, steady_state_ascending_timestamp)
    print(f'Transient state ascending energy consumption: {transient_state_ascending_energy_consumption} J')
    print(f'Transient state ascending delta time: {(transient_state_ascending_timestamp[-1] - transient_state_ascending_timestamp[0]) / 1000} s')
    print(f'Transient state ascending power consumption: {transient_state_ascending_energy_consumption / (transient_state_ascending_timestamp[-1] - transient_state_ascending_timestamp[0])} W')
    print(f'Steady state ascending energy consumption: {steady_state_ascending_energy_consumption} J')
    print(f'Steady state ascending delta time: {(steady_state_ascending_timestamp[-1] - steady_state_ascending_timestamp[0]) / 1000} s')
    total_ascending_energy_consumption = transient_state_ascending_energy_consumption + steady_state_ascending_energy_consumption
    print(f'Total ascending energy consumption: {total_ascending_energy_consumption} J')
    print(f'For a delta time of {(steady_state_ascending_timestamp[-1] - transient_state_ascending_timestamp[0]) / 1000 } s')
    print(f'Power consumption: {total_ascending_energy_consumption / (steady_state_ascending_timestamp[-1] - transient_state_ascending_timestamp[0])} W')
    print('--------------------------------------------------')

    # - Hovering to roll: 347753.184ms (indice: 459) to 361153.184ms (indice: 593) for the whole roll phase
    # Transient state: 347753.184ms to 351253.184ms (indice: 459 to 494)
    # Steady state: 351253.184ms to 361153.184ms (indice: 494 to 593)
    transient_state_roll_voltage = [float(dataset[i][header.index('BAT.Volt')]) for i in range(459, 494)]
    transient_state_roll_current = [float(dataset[i][header.index('BAT.Curr')]) for i in range(459, 494)]
    transient_state_roll_timestamp = [float(dataset[i][header.index('timestamp(ms)')]) for i in range(459, 494)]
    steady_state_roll_voltage = [float(dataset[i][header.index('BAT.Volt')]) for i in range(494, 593)]
    steady_state_roll_current = [float(dataset[i][header.index('BAT.Curr')]) for i in range(494, 593)]
    steady_state_roll_timestamp = [float(dataset[i][header.index('timestamp(ms)')]) for i in range(494, 593)]

    transient_state_roll_energy_consumption = compute_energy_consumption(transient_state_roll_voltage, transient_state_roll_current, transient_state_roll_timestamp)
    steady_state_roll_energy_consumption = compute_energy_consumption(steady_state_roll_voltage, steady_state_roll_current, steady_state_roll_timestamp)
    print(f'Transient state roll energy consumption: {transient_state_roll_energy_consumption} J')
    print(f'Transient state roll delta time: {(transient_state_roll_timestamp[-1] - transient_state_roll_timestamp[0]) / 1000} s')
    print(f'Transient state roll power consumption: {transient_state_roll_energy_consumption / (transient_state_roll_timestamp[-1] - transient_state_roll_timestamp[0])} W')
    print(f'Steady state roll energy consumption: {steady_state_roll_energy_consumption} J')
    print(f'Steady state roll delta time: {(steady_state_roll_timestamp[-1] - steady_state_roll_timestamp[0]) / 1000} s')
    total_roll_energy_consumption = transient_state_roll_energy_consumption + steady_state_roll_energy_consumption
    print(f'Total roll energy consumption: {total_roll_energy_consumption} J')
    print(f'For a delta time of {(steady_state_roll_timestamp[-1] - transient_state_roll_timestamp[0]) / 1000 } s')
    print(f'Power consumption: {total_roll_energy_consumption / (steady_state_roll_timestamp[-1] - transient_state_roll_timestamp[0])} W')
    print('--------------------------------------------------')

    # - Hovering to pitch: 390453.184ms (indice: 886) to 401553.184ms (indice: 997) for the whole pitch phase
    # Transient state: 390453.184ms to 393953.184ms (indice: 886 to 921)
    # Steady state: 393953.184ms to 401553.184ms (indice: 921 to 997)
    transient_state_pitch_voltage = [float(dataset[i][header.index('BAT.Volt')]) for i in range(886, 921)]
    transient_state_pitch_current = [float(dataset[i][header.index('BAT.Curr')]) for i in range(886, 921)]
    transient_state_pitch_timestamp = [float(dataset[i][header.index('timestamp(ms)')]) for i in range(886, 921)]
    steady_state_pitch_voltage = [float(dataset[i][header.index('BAT.Volt')]) for i in range(921, 997)]
    steady_state_pitch_current = [float(dataset[i][header.index('BAT.Curr')]) for i in range(921, 997)]
    steady_state_pitch_timestamp = [float(dataset[i][header.index('timestamp(ms)')]) for i in range(921, 997)]

    transient_state_pitch_energy_consumption = compute_energy_consumption(transient_state_pitch_voltage, transient_state_pitch_current, transient_state_pitch_timestamp)
    steady_state_pitch_energy_consumption = compute_energy_consumption(steady_state_pitch_voltage, steady_state_pitch_current, steady_state_pitch_timestamp)
    print(f'Transient state pitch energy consumption: {transient_state_pitch_energy_consumption} J')
    print(f'Transient state pitch delta time: {(transient_state_pitch_timestamp[-1] - transient_state_pitch_timestamp[0]) / 1000} s')
    print(f'Transient state pitch power consumption: {transient_state_pitch_energy_consumption / (transient_state_pitch_timestamp[-1] - transient_state_pitch_timestamp[0])} W')
    print(f'Steady state pitch energy consumption: {steady_state_pitch_energy_consumption} J')
    print(f'Steady state pitch delta time: {(steady_state_pitch_timestamp[-1] - steady_state_pitch_timestamp[0]) / 1000} s')
    total_pitch_energy_consumption = transient_state_pitch_energy_consumption + steady_state_pitch_energy_consumption
    print(f'Total pitch energy consumption: {total_pitch_energy_consumption} J')
    print(f'For a delta time of {(steady_state_pitch_timestamp[-1] - transient_state_pitch_timestamp[0]) / 1000 } s')
    print(f'Power consumption: {total_pitch_energy_consumption / (steady_state_pitch_timestamp[-1] - transient_state_pitch_timestamp[0])} W')
    print('--------------------------------------------------')

    # - Hovering to descending: 455753.184ms (indice: 1539) to 462253.184ms (indice: 1604) for the whole descending phase
    # Transient state: 455753.184ms to 459253.184ms (indice: 1539 to 1574)
    # Steady state: 459253.184ms to 462253.184ms (indice: 1574 to 1604)
    transient_state_descending_voltage = [float(dataset[i][header.index('BAT.Volt')]) for i in range(1539, 1574)]
    transient_state_descending_current = [float(dataset[i][header.index('BAT.Curr')]) for i in range(1539, 1574)]
    transient_state_descending_timestamp = [float(dataset[i][header.index('timestamp(ms)')]) for i in range(1539, 1574)]
    steady_state_descending_voltage = [float(dataset[i][header.index('BAT.Volt')]) for i in range(1574, 1604)]
    steady_state_descending_current = [float(dataset[i][header.index('BAT.Curr')]) for i in range(1574, 1604)]
    steady_state_descending_timestamp = [float(dataset[i][header.index('timestamp(ms)')]) for i in range(1574, 1604)]

    transient_state_descending_energy_consumption = compute_energy_consumption(transient_state_descending_voltage, transient_state_descending_current, transient_state_descending_timestamp)
    steady_state_descending_energy_consumption = compute_energy_consumption(steady_state_descending_voltage, steady_state_descending_current, steady_state_descending_timestamp)
    print(f'Transient state descending energy consumption: {transient_state_descending_energy_consumption} J')
    print(f'Transient state descending delta time: {(transient_state_descending_timestamp[-1] - transient_state_descending_timestamp[0]) / 1000} s')
    print(f'Transient state descending power consumption: {transient_state_descending_energy_consumption / (transient_state_descending_timestamp[-1] - transient_state_descending_timestamp[0])} W')
    print(f'Steady state descending energy consumption: {steady_state_descending_energy_consumption} J')
    print(f'Steady state descending delta time: {(steady_state_descending_timestamp[-1] - steady_state_descending_timestamp[0]) / 1000} s')
    total_descending_energy_consumption = transient_state_descending_energy_consumption + steady_state_descending_energy_consumption
    print(f'Total descending energy consumption: {total_descending_energy_consumption} J')
    print(f'For a delta time of {(steady_state_descending_timestamp[-1] - transient_state_descending_timestamp[0]) / 1000 } s')
    print(f'Power consumption: {total_descending_energy_consumption / (steady_state_descending_timestamp[-1] - transient_state_descending_timestamp[0])} W')
    print('--------------------------------------------------')

    ########################################################
    """

    ############## For 3-Test_loit_4ms.csv: ##############
    # - Hovering to ascending: 206129.65ms (indice: 99) to 210529.65ms (indice: 143) for the whole ascending phase
    # Transient state: 206129.65ms to 208029.65ms (indice: 99 to 118)
    # Steady state: 208029.65ms to 210529.65ms (indice: 118 to 143)
    transient_state_ascending_voltage = [float(dataset[i][header.index('BAT.Volt')]) for i in range(99, 118)]
    transient_state_ascending_current = [float(dataset[i][header.index('BAT.Curr')]) for i in range(99, 118)]
    transient_state_ascending_timestamp = [float(dataset[i][header.index('timestamp(ms)')]) for i in range(99, 118)]
    steady_state_ascending_voltage = [float(dataset[i][header.index('BAT.Volt')]) for i in range(118, 143)]
    steady_state_ascending_current = [float(dataset[i][header.index('BAT.Curr')]) for i in range(118, 143)]
    steady_state_ascending_timestamp = [float(dataset[i][header.index('timestamp(ms)')]) for i in range(118, 143)]

    transient_state_ascending_energy_consumption = compute_energy_consumption(transient_state_ascending_voltage, transient_state_ascending_current, transient_state_ascending_timestamp)
    steady_state_ascending_energy_consumption = compute_energy_consumption(steady_state_ascending_voltage, steady_state_ascending_current, steady_state_ascending_timestamp)
    print(f'Transient state ascending energy consumption: {transient_state_ascending_energy_consumption} J')
    print(f'Transient state ascending delta time: {(transient_state_ascending_timestamp[-1] - transient_state_ascending_timestamp[0]) / 1000} s')
    print(f'Transient state ascending power consumption: {transient_state_ascending_energy_consumption / ((transient_state_ascending_timestamp[-1] - transient_state_ascending_timestamp[0]) / 1000)} W')
    print(f'Steady state ascending energy consumption: {steady_state_ascending_energy_consumption} J')
    print(f'Steady state ascending delta time: {(steady_state_ascending_timestamp[-1] - steady_state_ascending_timestamp[0]) / 1000} s')
    print(f'Steady state ascending power consumption: {steady_state_ascending_energy_consumption / ((steady_state_ascending_timestamp[-1] - steady_state_ascending_timestamp[0]) / 1000)} W')
    total_ascending_energy_consumption = transient_state_ascending_energy_consumption + steady_state_ascending_energy_consumption
    print(f'Total ascending energy consumption: {total_ascending_energy_consumption} J')
    print(f'For a delta time of {(steady_state_ascending_timestamp[-1] - transient_state_ascending_timestamp[0]) / 1000 } s')
    print(f'Power consumption: {total_ascending_energy_consumption / ((steady_state_ascending_timestamp[-1] - transient_state_ascending_timestamp[0]) / 1000)} W')
    print('--------------------------------------------------')

    # - Hovering to roll: 224829.65ms (indice: 286) to 228229.65ms (indice: 320) for the whole roll phase
    # Transient state: 224829.65ms to 226129.65ms (indice: 286 to 299)
    # Steady state: 226129.65ms to 228229.65ms (indice: 299 to 320)
    transient_state_roll_voltage = [float(dataset[i][header.index('BAT.Volt')]) for i in range(286, 299)]
    transient_state_roll_current = [float(dataset[i][header.index('BAT.Curr')]) for i in range(286, 299)]
    transient_state_roll_timestamp = [float(dataset[i][header.index('timestamp(ms)')]) for i in range(286, 299)]
    steady_state_roll_voltage = [float(dataset[i][header.index('BAT.Volt')]) for i in range(299, 320)]
    steady_state_roll_current = [float(dataset[i][header.index('BAT.Curr')]) for i in range(299, 320)]
    steady_state_roll_timestamp = [float(dataset[i][header.index('timestamp(ms)')]) for i in range(299, 320)]

    transient_state_roll_energy_consumption = compute_energy_consumption(transient_state_roll_voltage, transient_state_roll_current, transient_state_roll_timestamp)
    steady_state_roll_energy_consumption = compute_energy_consumption(steady_state_roll_voltage, steady_state_roll_current, steady_state_roll_timestamp)
    print(f'Transient state roll energy consumption: {transient_state_roll_energy_consumption} J')
    print(f'Transient state roll delta time: {(transient_state_roll_timestamp[-1] - transient_state_roll_timestamp[0]) / 1000} s')
    print(f'Transient state roll power consumption: {transient_state_roll_energy_consumption / ((transient_state_roll_timestamp[-1] - transient_state_roll_timestamp[0]) / 1000)} W')
    print(f'Steady state roll energy consumption: {steady_state_roll_energy_consumption} J')
    print(f'Steady state roll delta time: {(steady_state_roll_timestamp[-1] - steady_state_roll_timestamp[0]) / 1000} s')
    print(f'Steady state roll power consumption: {steady_state_roll_energy_consumption / ((steady_state_roll_timestamp[-1] - steady_state_roll_timestamp[0]) / 1000)} W')
    total_roll_energy_consumption = transient_state_roll_energy_consumption + steady_state_roll_energy_consumption
    print(f'Total roll energy consumption: {total_roll_energy_consumption} J')
    print(f'For a delta time of {(steady_state_roll_timestamp[-1] - transient_state_roll_timestamp[0]) / 1000 } s')
    print(f'Power consumption: {total_roll_energy_consumption / ((steady_state_roll_timestamp[-1] - transient_state_roll_timestamp[0]) / 1000)} W')
    print('--------------------------------------------------')

    # - Hovering to pitch: 230929.65ms (indice: 347) to 234729.65ms (indice: 385) for the whole pitch phase
    # Transient state: 230929.65ms to 232029.65ms (indice: 347 to 358)
    # Steady state: 232029.65ms to 234729.65ms (indice: 358 to 385)
    transient_state_pitch_voltage = [float(dataset[i][header.index('BAT.Volt')]) for i in range(347, 358)]
    transient_state_pitch_current = [float(dataset[i][header.index('BAT.Curr')]) for i in range(347, 358)]
    transient_state_pitch_timestamp = [float(dataset[i][header.index('timestamp(ms)')]) for i in range(347, 358)]
    steady_state_pitch_voltage = [float(dataset[i][header.index('BAT.Volt')]) for i in range(358, 385)]
    steady_state_pitch_current = [float(dataset[i][header.index('BAT.Curr')]) for i in range(358, 385)]
    steady_state_pitch_timestamp = [float(dataset[i][header.index('timestamp(ms)')]) for i in range(358, 385)]

    transient_state_pitch_energy_consumption = compute_energy_consumption(transient_state_pitch_voltage, transient_state_pitch_current, transient_state_pitch_timestamp)
    steady_state_pitch_energy_consumption = compute_energy_consumption(steady_state_pitch_voltage, steady_state_pitch_current, steady_state_pitch_timestamp)
    print(f'Transient state pitch energy consumption: {transient_state_pitch_energy_consumption} J')
    print(f'Transient state pitch delta time: {(transient_state_pitch_timestamp[-1] - transient_state_pitch_timestamp[0]) / 1000} s')
    print(f'Transient state pitch power consumption: {transient_state_pitch_energy_consumption / ((transient_state_pitch_timestamp[-1] - transient_state_pitch_timestamp[0]) / 1000)} W')
    print(f'Steady state pitch energy consumption: {steady_state_pitch_energy_consumption} J')
    print(f'Steady state pitch delta time: {(steady_state_pitch_timestamp[-1] - steady_state_pitch_timestamp[0]) / 1000} s')
    print(f'Steady state pitch power consumption: {steady_state_pitch_energy_consumption / ((steady_state_pitch_timestamp[-1] - steady_state_pitch_timestamp[0]) / 1000)} W')
    total_pitch_energy_consumption = transient_state_pitch_energy_consumption + steady_state_pitch_energy_consumption
    print(f'Total pitch energy consumption: {total_pitch_energy_consumption} J')
    print(f'For a delta time of {(steady_state_pitch_timestamp[-1] - transient_state_pitch_timestamp[0]) / 1000 } s')
    print(f'Power consumption: {total_pitch_energy_consumption / ((steady_state_pitch_timestamp[-1] - transient_state_pitch_timestamp[0]) / 1000)} W')
    print('--------------------------------------------------')

    # - Hovering to descending: 211529.65ms (indice: 153) to 217029.65ms (indice: 208) for the whole descending phase
    # Transient state: 211529.65ms to 213229.65ms (indice: 153 to 170)
    # Steady state: 213229.65ms to 217029.65ms (indice: 170 to 208)
    transient_state_descending_voltage = [float(dataset[i][header.index('BAT.Volt')]) for i in range(153, 170)]
    transient_state_descending_current = [float(dataset[i][header.index('BAT.Curr')]) for i in range(153, 170)]
    transient_state_descending_timestamp = [float(dataset[i][header.index('timestamp(ms)')]) for i in range(153, 170)]
    steady_state_descending_voltage = [float(dataset[i][header.index('BAT.Volt')]) for i in range(170, 208)]
    steady_state_descending_current = [float(dataset[i][header.index('BAT.Curr')]) for i in range(170, 208)]
    steady_state_descending_timestamp = [float(dataset[i][header.index('timestamp(ms)')]) for i in range(170, 208)]

    transient_state_descending_energy_consumption = compute_energy_consumption(transient_state_descending_voltage, transient_state_descending_current, transient_state_descending_timestamp)
    steady_state_descending_energy_consumption = compute_energy_consumption(steady_state_descending_voltage, steady_state_descending_current, steady_state_descending_timestamp)
    print(f'Transient state descending energy consumption: {transient_state_descending_energy_consumption} J')
    print(f'Transient state descending delta time: {(transient_state_descending_timestamp[-1] - transient_state_descending_timestamp[0]) / 1000} s')
    print(f'Transient state descending power consumption: {transient_state_descending_energy_consumption / ((transient_state_descending_timestamp[-1] - transient_state_descending_timestamp[0]) / 1000)} W')
    print(f'Steady state descending energy consumption: {steady_state_descending_energy_consumption} J')
    print(f'Steady state descending delta time: {(steady_state_descending_timestamp[-1] - steady_state_descending_timestamp[0]) / 1000} s')
    print(f'Steady state descending power consumption: {steady_state_descending_energy_consumption / ((steady_state_descending_timestamp[-1] - steady_state_descending_timestamp[0]) / 1000)} W')
    total_descending_energy_consumption = transient_state_descending_energy_consumption + steady_state_descending_energy_consumption
    print(f'Total descending energy consumption: {total_descending_energy_consumption} J')
    print(f'For a delta time of {(steady_state_descending_timestamp[-1] - transient_state_descending_timestamp[0]) / 1000 } s')
    print(f'Power consumption: {total_descending_energy_consumption / ((steady_state_descending_timestamp[-1] - transient_state_descending_timestamp[0]) / 1000)} W')
    print('--------------------------------------------------')


