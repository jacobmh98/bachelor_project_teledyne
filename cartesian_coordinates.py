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

# Setting cartesian coordinates for each ping location with ping0 as 0-reference
ping_coords = [(0,0)]
for p in dataset:
    if p != ping0:
        p_lat = p.position_set[0].latitude
        p_long = p.position_set[0].longitude
        p_x = R * cos(p_lat) * cos(p_long) - base_x
        p_y = R * cos(p_lat) * sin(p_long) - base_y
        #p_z = R * sin(p_lat) - base_z
        ping_coords.append((p_x, p_y))

for i, c in enumerate(ping_coords):
    print(f"(x{i}, y{i}) = {c}")