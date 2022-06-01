import sys
import numpy as np
import math
import pyread7k
import json
import os
from pyread7k import Ping
from geopy import Point
from geopy.distance import distance
from progressbar import progressbar

if __name__ == '__main__':

    if len(sys.argv) > 1:
        print(sys.argv[1])
        path = sys.argv[1]
    else:
        print("No path chosen, path set to: ")
        path = "C:/Users/Max/Desktop/NBS-Snippets-Sensor-WC+-+1.s7k"
        print(path)

    path = input()
    # Setup dataset
    print("Path is: " + path)
    if(not(os.path.exists(path))):
        print("PATH DOESN'T EXIST!")
    dataset = pyread7k.FileDataset(path)

    # Highest and lowest point in dataset
    highest_point = -1.7976931348623157e+308  # Smallest negative float that is not negative infinity
    lowest_point = 1.7976931348623157e+308  # Biggest float that is not infinity

    # Largest and smallest values for length and width axis
    min_length_axis = 1.7976931348623157e+308
    max_length_axis = -1.7976931348623157e+308
    min_width_axis = 1.7976931348623157e+308
    max_width_axis = -1.7976931348623157e+308

    # List to save the boats coordinates for each ping
    ping_boat_coord = []

    def translate(points, bearing, dist):
        """Apply a translation to the points

        Args:
            points: An array of xyz points
            bearing: The angle they should be moved
            dist: A distance they should be moved
        Returns
            Translated xyz points

        """

        #print("Applying translation to points...")
        new_points = points[:]
        dy = np.sin(bearing) * dist
        dx = np.cos(bearing) * dist
        new_points[:, 0] += dx
        new_points[:, 1] += dy
        #print("Applied translation!")
        return new_points


    def bearing(point_a: Point, point_b: Point):
        """Compute the bearing between to geolocations.
        Function to calculate the bearing between two points.
        There is a difference in return depending on order,
        i.e. bearing(a, b) != bearing(b, a).

        Args:
            point_a: The first point
            point_b: The other point
        Returns
            The angle between the two points as seen from point a.

        """
        #print("Computing bearing between geolocations...")
        lat1 = math.radians(point_a[0])
        lat2 = math.radians(point_b[0])

        delta_lon = math.radians(point_a[1] - point_b[1])

        x = math.cos(lat2) * math.sin(delta_lon)
        y = math.cos(lat1) * math.sin(lat2) - (
                math.sin(lat1) * math.cos(lat2) * math.cos(delta_lon)
        )
        #print("Computed bearing!")
        return math.atan2(x, y)


    def bda(ping):

        #print("In bda function...")
        raw_rb = np.vstack(
            (
                ping.raw_detections.detections["detection_point"]
                * ping.sonar_settings.sound_velocity
                / (2 * ping.sonar_settings.sample_rate),
                ping.raw_detections.detections["rx_angle"],
            )
        ).T
        print(ping.raw_detections.detections["detection_point"])
        indices = np.array(list(zip(
            ping.raw_detections.detections["detection_point"].astype(int),
            ping.raw_detections.detections["beam_descriptor"].astype(int),
        )))

        raw_points = np.array(
            [
                (
                    0,
                    -raw_rb[k, 0] * np.sin(raw_rb[k, 1]),
                    -raw_rb[k, 0] * np.cos(raw_rb[k, 1]),
                )
                for k in range(len(raw_rb))
            ]
        )
        points_amplitudes = ping.raw_detections.detections["intensity"].reshape(-1, 1)
        #print(raw_points)
        return indices, points_amplitudes, raw_points


    def Ryaw(alpha: float):
        """Yaw rotation matrix - Rotation of alpha around the z-axis"""

        #print("Created yaw rotation matrix!")
        return np.array(
            [
                [np.cos(alpha), -np.sin(alpha), 0],
                [np.sin(alpha), np.cos(alpha), 0],
                [0, 0, 1],
            ]
        )


    def Rpitch(beta: float):
        """Pitch rotation matrix - Rotation of beta around the y-axis"""
        #print("Created pitch rotation matrix!")
        return np.array(
            [
                [np.cos(beta), 0, np.sin(beta)],
                [0, 1, 0],
                [-np.sin(beta), 0, np.cos(beta)]
            ]
        )


    def Rroll(gamma: float):
        """Roll rotation matrix - Rotation gamma around the x-axis"""
        #print("Created roll rotation matrix!")
        return np.array(
            [
                [1, 0, 0],
                [0, np.cos(gamma), -np.sin(gamma)],
                [0, np.sin(gamma), np.cos(gamma)],
            ]
        )


    def Rmat(pitch: float, roll: float, yaw: float):
        """Create the yaw, pitch, and roll matrix."""
        #print("Created yaw, pitch and roll matrix!")
        return Rroll(roll) @ Rpitch(pitch) @ Ryaw(yaw)  # Theoretically correct


    def rotate(ping, indices: np.ndarray, points: np.ndarray, roll_corrected: bool):
        #print("Rotating the points...")
        rotated_points = np.zeros_like(points)
        for i, idx in enumerate(indices):
            rph, h = ping.receiver_motion_for_sample(idx[0])
            euler_roll = -np.arcsin(np.sin(rph.roll) / (np.cos(rph.pitch)))
            rotmat = Rmat(rph.pitch, euler_roll, h.heading)  # rph.roll)
            rotated_points[i, :] = np.dot(points[i, :].reshape(1, -1), rotmat)
            # Correct for heave
            rotated_points[i, -1] += rph.heave
        #print("Rotated the points!")
        return rotated_points, rph.heave


    def to_points(ping: Ping, correct_motion: bool = True):
        """Create points from a ping
        This function is used to create x,y,z points from a pings amplitudes and motion parameters

        Args:
            ping: A s7k ping
            cm_per_sample: Number of centimeters per sample. Default is 10.
            point_filter: A filter that selects relevant samples and beams. Default is the simple amplitude filter.
            detections: Flag specifying whether to use the raw detections. Default is false
        Returns:
            indices: A list of indices that were deemed relevant
            points_amplitudes: The amplitudes of the detections
            raw_points: The raw points in x-y-z coordinates (x=0)
            rotated_points: The motion corrected points in x-y-z coordinates

        """
        #print("Creating points from a ping...")
        # Check if roll compensation has been applied
        roll_corrected = ping.sonar_settings.receive_flags == 1

        indices, points_amplitudes, raw_points = bda(ping)
        if correct_motion:
            points, heave = rotate(ping, indices, raw_points, roll_corrected)
        else:
            points = raw_points

        # Calculating the boats position at the current ping
        #ping_boat_coord.append(boat_coordinates(ping))
        #print("Created points from a ping!")
        return indices, points_amplitudes, points, heave


    def to_pointcloud(dataset, correct_motion: bool = True):
        """Create points clouds from an iterable of pings
        This function is used to create x,y,z points from a pings amplitudes and motion parameters.

        NOTE: The current implementation of pyread7k doesn't support record 7503, meaning that
              pointclouds created from sonars that are tilted, such as dual-head sonars, will
              look correct but will be sloped oddly. An implementation of 7503 is underway.

        Args:
            dataset: An iterable of pings
            correct_motion: Whether or not to apply motion correction
        Returns:
            An array of x,y,z, and an amplitudes array

        """
        print("Creating point cloud from pings...")
        for count in progressbar(range(1)):

            associated_pings = []
            associated_index = []
            indices = []
            pointcloud = []
            amplitudes = []
            pings = [p for p in dataset if len(p.position_set) > 0]
            first_ping = pings[0]

            for i, ping in enumerate(pings):
                if not len(ping.position_set):
                    continue
                if i > 0:
                    dist = distance(ping.gps_position, first_ping.gps_position).meters
                    phi = bearing(ping.gps_position, first_ping.gps_position)
                else:
                    dist = 0
                    phi = 0
                idx, amps, rotp, heave = to_points(ping, correct_motion)

                # Calculating boat points
                boat_point_x = dist * math.cos(phi)
                boat_point_y = dist * math.sin(phi)

                ping_boat_coord.append([boat_point_x, boat_point_y, heave])

                new_xyz = translate(rotp, phi, dist)
                pointcloud.append(new_xyz)
                amplitudes.append(amps)
                indices.append(idx)
                associated_pings.append(np.ones_like(amps) * ping.ping_number)
                associated_index.append(np.ones_like(amps) * i)

                global lowest_point, highest_point, min_length_axis, max_length_axis, min_width_axis, max_width_axis
                for point in new_xyz:
                    if point[2] > highest_point:
                        highest_point = point[2]
                    elif point[2] < lowest_point:
                        lowest_point = point[2]
                    if point[0] < min_length_axis:
                        min_length_axis = point[0]
                    elif point[0] > max_length_axis:
                        max_length_axis = point[0]
                    if point[1] < min_width_axis:
                        min_width_axis = point[1]
                    elif point[1] > max_width_axis:
                        max_width_axis = point[1]

            dataset.minimize_memory()
        print("Created point cloud!")
        return pointcloud


    def generate_json(pointcloud):
        print("Generating JSON...")
        for count in progressbar(range(1)):

            ping_amount = len(pointcloud)
            number_of_counts_sum = 0
            for pings in pointcloud:
                number_of_counts_sum += len(pings)

            data = {}
            data["no_pings"] = ping_amount
            data["no_counts"] = number_of_counts_sum
            data["minimum_depth"] = math.ceil(highest_point)
            data["maximum_depth"] = math.floor(lowest_point)
            data["min_length_axis"] = math.floor(min_length_axis)
            data["max_length_axis"] = math.ceil(max_length_axis)
            data["min_width_axis"] = math.floor(min_width_axis)
            data["max_width_axis"] = math.ceil(max_width_axis)
            data["pings"] = []

            for i, point_row in enumerate(pointcloud):
                ping = {
                    "pingID": i,
                    "no_points": len(point_row),
                    "ping_boat_coord": ping_boat_coord[i],
                    "coords_x": [],
                    "coords_y": [],
                    "coords_z": []
                }

                for x, y, z in point_row:
                    ping["coords_x"].append(x)
                    ping["coords_y"].append(y)
                    ping["coords_z"].append(z)

                data["pings"].append(ping)

            with open("point_cloud_data.json", "w") as f:
                json.dump(data, f)
        print("Generated JSON!")

    pointcloud = to_pointcloud(dataset)
    generate_json(pointcloud)
