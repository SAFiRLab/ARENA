import csv
import os
import math

import numpy as np
from matplotlib import pyplot as plt


# =============================================================================
# Configuration
# =============================================================================

# Quadric surface:
#
#   aX^2 + bY^2 + cZ^2 + gX + hY + iZ + 1 = 0
#
quadric_surface_coefficients = np.array([
    -1.04591340e-07,  # a
    -9.39985225e-08,  # b
    -9.67627744e-08,  # c
     1.58417906e-05,  # g
     1.33867670e-06,  # h
     6.48778020e-05   # i
])

max_power_value = 3475.0  # W
drone_mass = 25.0          # kg


# =============================================================================
# CSV utilities
# =============================================================================

def extract_csv_data(file):
    """
    Extract valid rows from a CSV file.

    Rows containing values outside [-1e20, 1e20] are discarded.
    """
    with open(file, 'r', newline='\n') as f:
        reader = csv.reader(f, delimiter=',')

        header = next(reader)
        data = []

        for row in reader:
            valid = True

            for value in row:
                value = float(value)

                if value > 1.0e20 or value < -1.0e20:
                    valid = False
                    break

            if valid:
                data.append(row)

    return header, data


# =============================================================================
# Velocity utilities
# =============================================================================

def get_velocity_vector(row, header):
    """
    Extract UAV velocity from the log.

    PX4 XKF1 velocity is expressed in the NED frame:
        VN = North velocity
        VE = East velocity
        VD = Down velocity

    Returns:
        np.ndarray of shape (3,), in m/s.
    """
    return np.array([
        float(row[header.index('XKF1[0].VN')]),
        float(row[header.index('XKF1[0].VE')]),
        float(row[header.index('XKF1[0].VD')])
    ])


def get_time(row, header):
    """
    Return timestamp in seconds.
    """
    return float(row[header.index('timestamp(ms)')]) / 1000.0


def extract_speed_unit_vectors_from_data(data, header):
    """
    Extract normalized velocity vectors.

    The input velocity is in the NED frame.
    The Down component is not inverted here; the inversion is performed
    before querying the quadric surface.
    """
    speed_unit_vectors = []

    for row in data:
        velocity_vector = get_velocity_vector(row, header)
        speed_norm = np.linalg.norm(velocity_vector)

        if speed_norm > 0.0:
            speed_unit_vector = velocity_vector / speed_norm
        else:
            speed_unit_vector = np.zeros(3)

        speed_unit_vectors.append(speed_unit_vector)

    return speed_unit_vectors


# =============================================================================
# Quadric surface / steady-state power
# =============================================================================

def get_power_value_from_quadric_surface(speed_unit_vector):
    """
    Estimate steady-state power from the quadric power surface.

    The input direction is assumed to be expressed in the coordinate system
    used by the fitted quadric surface.

    Returns:
        Estimated steady-state power in W.
    """

    A = (
        quadric_surface_coefficients[0] * speed_unit_vector[0]**2
        + quadric_surface_coefficients[1] * speed_unit_vector[1]**2
        + quadric_surface_coefficients[2] * speed_unit_vector[2]**2
    )

    B = (
        quadric_surface_coefficients[3] * speed_unit_vector[0]
        + quadric_surface_coefficients[4] * speed_unit_vector[1]
        + quadric_surface_coefficients[5] * speed_unit_vector[2]
    )

    C = 1.0

    delta = B**2 - 4.0 * A * C

    if delta < 0.0:
        raise ValueError(
            "No intersection point with the quadric surface."
        )

    sqrt_delta = np.sqrt(delta)

    t1 = (-B + sqrt_delta) / (2.0 * A)
    t2 = (-B - sqrt_delta) / (2.0 * A)

    # Select a positive intersection.
    positive_intersections = [
        t for t in (t1, t2)
        if t >= 0.0
    ]

    if len(positive_intersections) == 0:
        raise ValueError(
            "The quadric surface has no positive intersection "
            "along the supplied velocity direction."
        )

    # Closest positive intersection.
    t = min(positive_intersections)

    intersection_point = t * speed_unit_vector

    power_value = np.linalg.norm(intersection_point)

    return power_value


# =============================================================================
# Empirical energy
# =============================================================================

def evaluate_empirical_energy_consumption(data, header):
    """
    Integrate measured battery power over time.

    E_i = E_{i-1} + P_i * dt_i

    Returns:
        List of cumulative empirical energy values in J.
    """
    energy_values = []
    total_energy = 0.0

    previous_time = get_time(data[0], header)

    for row in data:
        current_time = get_time(row, header)
        delta_time = current_time - previous_time

        voltage = float(row[header.index('BAT.Volt')])
        current = float(row[header.index('BAT.Curr')])

        empirical_power = voltage * current

        total_energy += empirical_power * delta_time

        energy_values.append(total_energy)

        previous_time = current_time

    return energy_values


# =============================================================================
# Transient energy / power
# =============================================================================

def evaluate_transient_energy(velocity_previous, velocity_current, mass):
    """
    Compute the transient energy contribution.

    This reproduces the current C++ implementation:

        velocity_diff =
            velocity_i_m_1_vector.cwiseAbs2()
            - velocity_i_vector.cwiseAbs2();

        transient_energy =
            (0.5 * mass * velocity_diff).norm();

    Mathematically:

        E_tr =
            m/2 * || |v_{i-1}^2 - v_i|^2 ||

    where the squaring is performed component-wise.

    Returns:
        Transient energy in J.
    """

    velocity_diff = np.abs(velocity_previous**2 - velocity_current**2)
    transient_energy = 0.5 * mass * np.linalg.norm(velocity_diff)

    return transient_energy


def evaluate_transient_power(velocity_previous, velocity_current, delta_time, mass):
    """
    Convert transient energy into equivalent average transient power
    over the interval delta_time.

        P_tr = E_tr / delta_time

    Returns:
        Transient power in W.
    """

    if delta_time <= 0.0:
        return 0.0

    transient_energy = evaluate_transient_energy(velocity_previous, velocity_current, mass)
    transient_power = transient_energy / delta_time

    return transient_power


# =============================================================================
# Theoretical power
# =============================================================================

def evaluate_theoretical_power_consumption(speed_unit_vectors, data, header, mass):
    """
    Compute the theoretical power consumption.

    Total power is decomposed into:

        P_model = P_steady + P_transient

    where:

        P_steady
            is obtained from the fitted quadric surface.

        P_transient
            is the transient energy increment converted into an
            equivalent average power over the sampling interval:

            P_transient = E_transient / delta_t

    Returns:
        Theoretical power values in W.
    """

    power_values = []

    previous_time = get_time(data[0], header)

    for i, speed_unit_vector in enumerate(speed_unit_vectors):

        current_time = get_time(data[i], header)
        delta_time = current_time - previous_time

        # -----------------------------------------------------------------
        # Steady-state power
        # -----------------------------------------------------------------

        # Convert NED -> coordinate convention used by the quadric.
        quadric_direction = np.array([speed_unit_vector[0], speed_unit_vector[1], -speed_unit_vector[2]])

        # Handle zero velocity explicitly.
        if np.linalg.norm(quadric_direction) > 0.0:
            steady_power = get_power_value_from_quadric_surface(quadric_direction)
        else:
            steady_power = 0.0

        # -----------------------------------------------------------------
        # Transient power
        # -----------------------------------------------------------------

        if i > 0:
            velocity_previous = get_velocity_vector(data[i - 1], header)
            velocity_current = get_velocity_vector(data[i], header)

            transient_power = evaluate_transient_power(velocity_previous, velocity_current, delta_time, mass)
        else:
            # No previous velocity exists for the first sample.
            transient_power = 0.0

        # -----------------------------------------------------------------
        # Total power
        # -----------------------------------------------------------------

        total_power = steady_power + transient_power

        power_values.append(total_power)

        previous_time = current_time

    return power_values


# =============================================================================
# Theoretical energy
# =============================================================================

def evaluate_theoretical_energy_consumption(speed_unit_vectors, data, header, mass):
    """
    Integrate the theoretical power model to obtain cumulative energy.

        E_model = integral(P_model dt)

    with:

        P_model = P_steady + P_transient

    and:

        P_transient = E_transient / delta_t.

    Consequently, for each interval:

        P_model * delta_t
            =
        P_steady * delta_t + E_transient

    which is consistent with the original discrete transient-energy
    formulation.

    Returns:
        Cumulative theoretical energy values in J.
    """

    energy_values = []

    total_energy = 0.0

    previous_time = get_time(data[0], header)

    for i, speed_unit_vector in enumerate(speed_unit_vectors):

        current_time = get_time(data[i], header)
        delta_time = current_time - previous_time

        # -----------------------------------------------------------------
        # Steady-state power
        # -----------------------------------------------------------------

        quadric_direction = np.array([speed_unit_vector[0], speed_unit_vector[1], -speed_unit_vector[2]])

        if np.linalg.norm(quadric_direction) > 0.0:
            steady_power = get_power_value_from_quadric_surface(quadric_direction)
        else:
            steady_power = 0.0

        # -----------------------------------------------------------------
        # Transient power
        # -----------------------------------------------------------------

        if i > 0:
            velocity_previous = get_velocity_vector(data[i - 1], header)
            velocity_current = get_velocity_vector(data[i], header)

            transient_power = evaluate_transient_power(velocity_previous, velocity_current, delta_time, mass)
        else:
            transient_power = 0.0

        # -----------------------------------------------------------------
        # Total power
        # -----------------------------------------------------------------

        total_power = steady_power + transient_power

        # -----------------------------------------------------------------
        # Integrate power over the interval
        # -----------------------------------------------------------------

        total_energy += total_power * delta_time

        energy_values.append(total_energy)

        previous_time = current_time

    return energy_values


# =============================================================================
# Empirical power
# =============================================================================

def evaluate_empirical_power_consumption(data, header):
    """
    Compute measured electrical power:

        P = V * I

    A three-sample moving-average filter is applied afterwards.
    """

    power_values = []

    for row in data:
        voltage = float(row[header.index('BAT.Volt')])
        current = float(row[header.index('BAT.Curr')])

        power_values.append(voltage * current)

    if len(power_values) <= 2:
        return power_values

    filtered_power_values = []

    filtered_power_values.append(power_values[0])

    for i in range(1, len(power_values) - 1):
        filtered_power_values.append(np.mean([power_values[i - 1],power_values[i], power_values[i + 1]]))

    filtered_power_values.append(power_values[-1])

    return filtered_power_values


# =============================================================================
# Plotting
# =============================================================================

def get_relative_time_values(data, header):
    """
    Return timestamps relative to the first sample, in seconds.
    """
    first_time = get_time(data[0], header)

    return np.array([get_time(row, header) - first_time for row in data])


def plot_power_consumptions(empirical_power_values, theoretical_power_values, data, header, title_trajectory):

    time_values = get_relative_time_values(data, header)

    plt.figure()

    plt.plot(time_values, empirical_power_values, label='Empirical power consumption')

    plt.plot(time_values, theoretical_power_values, label='Model estimation')

    plt.xlabel('Time (s)', fontsize=26)
    plt.ylabel('Power (W)', fontsize=26)

    plt.legend(fontsize=26)

    plt.title('Empirical power consumption VS model estimation for the "' + title_trajectory + '" trajectory', fontsize=26)

    plt.tick_params(axis='both', which='major', labelsize=26)

    plt.ylim(2500, 3650)

    fig = plt.gcf()
    fig.set_size_inches(18.5, 10.5)

    plt.show()


def plot_energy_consumptions(empirical_energy_values, theoretical_energy_values, data, header):

    time_values = get_relative_time_values(data, header)

    errors = np.subtract(empirical_energy_values, theoretical_energy_values)

    plt.figure()

    plt.plot(time_values, empirical_energy_values, label='Empirical energy consumption')

    plt.plot(time_values, theoretical_energy_values, label='Theoretical energy consumption')

    plt.fill_between(time_values, theoretical_energy_values, theoretical_energy_values + errors, alpha=0.5, label='Error')

    plt.xlabel('Time (s)', fontsize=18)
    plt.ylabel('Energy (J)', fontsize=18)

    plt.legend()

    fig = plt.gcf()
    fig.set_size_inches(18.5, 10.5)

    plt.show()


# =============================================================================
# Main
# =============================================================================

if __name__ == '__main__':

    # -------------------------------------------------------------------------
    # Input files
    # -------------------------------------------------------------------------

    current_path1 = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'data', 'test5', 'data_3.csv')
    current_path2 = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'data', 'test6', 'data_2.csv')

    # -------------------------------------------------------------------------
    # Load data
    # -------------------------------------------------------------------------

    header, data = extract_csv_data(current_path1)
    header2, data2 = extract_csv_data(current_path2)

    # -------------------------------------------------------------------------
    # Extract velocity directions
    # -------------------------------------------------------------------------

    speed_unit_vectors = extract_speed_unit_vectors_from_data(data, header)
    speed_unit_vectors2 = extract_speed_unit_vectors_from_data( data2, header2)

    # -------------------------------------------------------------------------
    # Dataset 1
    # -------------------------------------------------------------------------

    empirical_power_values = evaluate_empirical_power_consumption(data, header)
    theoretical_power_values = evaluate_theoretical_power_consumption(speed_unit_vectors, data, header, drone_mass)

    empirical_energy_values = evaluate_empirical_energy_consumption(data, header)
    theoretical_energy_values = evaluate_theoretical_energy_consumption(speed_unit_vectors, data, header, drone_mass)

    # -------------------------------------------------------------------------
    # Dataset 2
    # -------------------------------------------------------------------------

    empirical_power_values2 = evaluate_empirical_power_consumption(data2, header2)
    theoretical_power_values2 = evaluate_theoretical_power_consumption(speed_unit_vectors2, data2, header2, drone_mass)

    empirical_energy_values2 = evaluate_empirical_energy_consumption(data2, header2)
    theoretical_energy_values2 = evaluate_theoretical_energy_consumption(speed_unit_vectors2, data2, header2, drone_mass)

    # -------------------------------------------------------------------------
    # Power plots
    # -------------------------------------------------------------------------

    plot_power_consumptions(empirical_power_values, theoretical_power_values, data, header, "safety")
    plot_power_consumptions(empirical_power_values2, theoretical_power_values2, data2, header2, "energy")

    # -------------------------------------------------------------------------
    # Energy plots
    # -------------------------------------------------------------------------

    #plot_energy_consumptions(empirical_energy_values, theoretical_energy_values, data, header)
    #plot_energy_consumptions(empirical_energy_values2, theoretical_energy_values2, data2, header2)
