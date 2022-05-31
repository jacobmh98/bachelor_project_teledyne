from copy import deepcopy

import numpy as np
import pyread7k
from math import sin, cos, acos
from numpy.linalg import norm
from numpy import dot, pi, transpose
import matplotlib.pyplot as plt
import json
from scipy import stats
import sys


class cartesian_coordinates:

    def bruh(self):
        print("bruh")


if __name__ == '__main__':

    # Setup dataset
    path = "C:/Users/Max/Desktop/NBS-Snippets-Sensor-WC+-+1.s7k"
    if len(sys.argv) > 1:
        print(sys.argv[1])
        path = sys.argv[1]
    else:
        path = "NBS-Snippets-Sensor-WC+-+1.s7k"
    dataset = pyread7k.PingDataset(path, include=pyread7k.PingType.BEAMFORMED)

    # Setting first ping for base coordinates
    ping0 = dataset[0]
    base_lat = ping0.position_set[0].latitude
    base_long = ping0.position_set[0].longitude

    # General epsilon value to avoid 0 division etc.
    epsilon = 0.000001

    # Removing outliers if true
    remove_outliers = True
    z_score_threshold = 4

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
            z_values = []
            for d in p.raw_detections.detections:
                length = d[1]
                angle = d[2]

                p_x, p_y, p_z = ping_coords[-1]
                point_y = p_y + sin(angle) * length
                point_z = -(p_z + cos(angle) * length)

                p_points.append((p_x, point_y, point_z))
                z_values.append(point_z)

            if remove_outliers:
                z = np.abs(stats.zscore(z_values))
                p_points_length = len(p_points)
                i = 0
                j = 0
                while i < p_points_length:
                    if z[j] > z_score_threshold:
                        p_points.pop(i)
                        i -= 1
                        p_points_length -= 1
                    i += 1
                    j += 1

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


    def generate_json():
        data = {}
        data["no_pings"] = len(ping_coords)
        data["pings"] = []

        for i, point_row in enumerate(ping_point_rows):
            ping = {
                "pingID": i,
                "no_points": len(point_row),
                "ping_coord": list(ping_coords[i]),
                "coords_x": [],
                "coords_y": [],
                "coords_z": []
            }

            for x,y,z in point_row:
                ping["coords_x"].append(x)
                ping["coords_y"].append(y)
                ping["coords_z"].append(z)

            data["pings"].append(ping)

        with open("C:/Users/Max/Desktop/testpointcloud/7k_data_extracted.json", "w") as f:
            json.dump(data, f)


    print("Creating JSON...")
    compute_coordinates()
    generate_json()
    # rotate_individual_pings()
    # generate_plots()
