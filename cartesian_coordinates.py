import pyread7k
from math import sin, cos
import matplotlib.pyplot as plt

# Setup dataset
path = "NBS-Snippets-Sensor-WC.s7k"
dataset = pyread7k.PingDataset(path, include=pyread7k.PingType.BEAMFORMED)

# Setting first ping for base coordinates
ping0 = dataset[0]
base_lat = ping0.position_set[0].latitude
base_long = ping0.position_set[0].longitude

# Calculating cartesian coordinates for first ping
R = 6371 * 1000
base_x = R * cos(base_lat) * cos(base_long)
base_y = R * cos(base_lat) * sin(base_long)
base_z = R * sin(base_lat)

ping_coords = [(0, 0, 0)]
ping_point_rows = []

def compute_coordinates():
    for p in dataset:
        # Setting cartesian coordinates for each ping location with ping0 as 0-reference
        if p != ping0:
            p_lat = p.position_set[0].latitude
            p_long = p.position_set[0].longitude
            p_x = R * cos(p_lat) * cos(p_long) - base_x
            p_y = R * cos(p_lat) * sin(p_long) - base_y
            #p_z = R * sin(p_lat) - base_z
            ping_coords.append((p_x, p_y, 0))

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

def generate_plots():
    x_boat_coords = []
    y_boat_coords = []

    x_ping_coords = []
    y_ping_coords = []

    for p in ping_coords:
        x_boat_coords.append(p[0])
        y_boat_coords.append(p[1])

    plt.plot(x_boat_coords, y_boat_coords, "r--", label = "Boats Path")
    plt.xlabel("X coordinate")
    plt.ylabel("Y coordinate")
    plt.title("Plotting boat coordinates 2D")

    plt.ylim(-10, 10)

    for ping in ping_point_rows:
        for point in ping:
            x_ping_coords.append(point[0])
            y_ping_coords.append(point[1])

    plt.plot(x_ping_coords, y_ping_coords, "bo", ms=0.4, label = "Sonar Points")
    plt.legend()
    plt.show()
# TODO account for changing of basis vectors when boat rotates (VERY IMPORTANT)



compute_coordinates()
generate_plots()
#print(ping_coords[0])
#print(ping_coords[1])
#print(ping_coords[2])
#print(ping_coords[3])