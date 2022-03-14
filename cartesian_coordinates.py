import pyread7k
from math import sin, cos

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

#
ping_coords = [(0, 0, 0)]
ping_point_rows = []
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

# TODO account for changing of basis vectors when boat rotates (VERY IMPORTANT)
