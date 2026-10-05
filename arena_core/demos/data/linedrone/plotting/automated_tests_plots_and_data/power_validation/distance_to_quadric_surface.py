import numpy as np
from matplotlib import pyplot as plt


def distance_to_empirical_quadric_surface(point, Z, X, Y):
    """
    Compute the shortest distance from a point to an empirical quadric surface.

    Parameters:
        point (tuple): The given point (x, y, z).
        quadric_surface_points (list): List of points (x, y, z) that define the quadric surface.

    Returns:
        dict: The distance and the closest point on the quadric.
    """

    # X, Y, and Z are np.meshgrid arrays that define the quadric surface in 3D space so we have to do the opposite operation to get the points in arrays [x, y, z]
    x = X.flatten()
    y = Y.flatten()
    z = Z.flatten()

    # Create a list of points that define the quadric surface
    quadric_surface_points = np.array(list(zip(x, y, z)))

    # remove NaN values
    quadric_surface_points = quadric_surface_points[~np.isnan(quadric_surface_points).any(axis=1)]

    # Compute the distance between the point and all the points on the quadric surface
    distances = np.linalg.norm(quadric_surface_points - point, axis=1)

    # Get the index of the closest point
    closest_point_index = np.argmin(distances)

    # Get the closest point
    closest_point = quadric_surface_points[closest_point_index]

    # Compute the distance between the point and the closest point
    distance = np.linalg.norm(point - closest_point)

    return {
        'distance': distance,
        'closest_point': closest_point
    }


# Test the function
if __name__ == '__main__':
    # Define the quadric surface
    max_value = 10
    step = 1
    x = range(-max_value, max_value, step)
    y = range(-max_value, max_value, step)
    X, Y = np.meshgrid(x, y)
    Z = X ** 2 + Y ** 2

    # Try to unmesh the grid
    x2 = X[:, 0]
    y2 = Y[:, 0]
    z2 = Z[:, 0]

    # Define the point
    point = (1, 1, 1)

    # Compute the distance to the quadric surface
    result = distance_to_empirical_quadric_surface(point, X, Y, Z)

    print(result)
