from copy import deepcopy

import numpy as np
import pyread7k
from math import sin, cos, acos
from numpy.linalg import norm
from numpy import dot, pi, transpose
import matplotlib.pyplot as plt

# Setup dataset
path = "NBS-Snippets-Sensor-WC.s7k"
dataset = pyread7k.PingDataset(path, include=pyread7k.PingType.BEAMFORMED)

# Setting first ping for base coordinates
ping0 = dataset[0]
base_lat = ping0.position_set[0].latitude
base_long = ping0.position_set[0].longitude

# General epsilon value to avoid 0 division etc.
epsilon = 0.000001

# Calculating cartesian coordinates for first ping
R = 6371 * 1000
base_x = R * cos(base_lat) * cos(base_long)
base_y = R * cos(base_lat) * sin(base_long)
base_z = R * sin(base_lat)

ping_coords = []
ping_point_rows = []
rotated_points = []
boat_directions = []

def compute_coordinates():
    for p in dataset:
        # Setting cartesian coordinates for each ping location with ping0 as 0-reference
        p_lat = p.position_set[0].latitude
        p_long = p.position_set[0].longitude
        p_x = R * cos(p_lat) * cos(p_long) - base_x
        p_y = R * cos(p_lat) * sin(p_long) - base_y
        #p_z = R * sin(p_lat) - base_z
        ping_coords.append((p_x, p_y, 0.0))

        # Setting cartesian coordinates for all points in a ping
        p_points = []
        for d in p.raw_detections.detections:
            length = d[1]
            angle = d[2]

            p_x, p_y, p_z = ping_coords[-1]
            point_y = p_y + sin(angle) * length
            point_z = -(p_z + cos(angle) * length)
            p_points.append((p_x, point_y, point_z))
        ping_point_rows.append(p_points)
        rotated_points.append(deepcopy(p_points))

def rotate_individual_pings():
    for ping_i, ping_point_row in enumerate(rotated_points):
        for point_i, point in enumerate(ping_point_row):
            if ping_i == len(rotated_points) - 1:
                break
            dir_vec = tuple(map(lambda i, j: i - j, ping_coords[ping_i + 1], ping_coords[ping_i]))
            ref_vec = (dir_vec[0], 0, 0)

            # Angle the points should be rotated
            angle_dir = -acos(dot(dir_vec, ref_vec) / (norm(dir_vec) * norm(ref_vec)))
            # Rotating points around z-axis by angle
            rot_z = np.array([[cos(angle_dir), sin(angle_dir), 0], [-sin(angle_dir), cos(angle_dir), 0], [0, 0, 1]])

            ping_point_row[point_i] = dot(rot_z, point)


def generate_plots():
    # Setting plotting data
    x_boat_coords = []
    y_boat_coords = []

    x_ping_coords = []
    y_ping_coords = []

    x_ping_rotated = []
    y_ping_rotated = []

    for p in ping_coords:
        x_boat_coords.append(p[0])
        y_boat_coords.append(p[1])

    for i, ping in enumerate(ping_point_rows):
        for point in ping:
            x_ping_coords.append(point[0])
            y_ping_coords.append(point[1])

    for i, ping in enumerate(rotated_points):
        for point in ping:
            x_ping_rotated.append(point[0])
            y_ping_rotated.append(point[1])

    # Displaying plot of boat path
    plt.plot(x_boat_coords, y_boat_coords, "r--", label = "Boats Path")
    plt.xlabel("X coordinate")
    plt.ylabel("Y coordinate")
    plt.title("Plotting boat coordinates 2D")
    #plt.plot(x_ping_coords, y_ping_coords, "bo", ms=0.4, label = "Sonar Points")
    plt.legend()
    plt.show()

    # Displaying plot of boat path with sonar points
    plt.plot(x_boat_coords, y_boat_coords, "r--", label="Boats Path")
    plt.xlabel("X coordinate")
    plt.ylabel("Y coordinate")
    plt.title("Plotting boat coordinates 2D")
    plt.plot(x_ping_coords, y_ping_coords, "bo", ms=0.4, label = "Sonar Points")
    plt.legend()
    plt.show()

    # Displaying plot of boat path with sonar points
    plt.plot(x_boat_coords, y_boat_coords, "r--", label="Boats Path")
    plt.xlabel("X coordinate")
    plt.ylabel("Y coordinate")
    plt.title("Plotting boat coordinates 2D")
    plt.plot(x_ping_rotated, y_ping_rotated, "bo", ms=0.4, label="Sonar Points (Rotated)")
    plt.legend()
    plt.show()

compute_coordinates()
rotate_individual_pings()
generate_plots()