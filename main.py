import sys
import numpy as np
import math
import pyread7k
import json
import os
from geopy import Point
from geopy.distance import distance
from progressbar import progressbar

if __name__ == '__main__':

    if len(sys.argv) > 1:
        print(sys.argv[1])
        path = sys.argv[1]
    else:
        print("No path chosen, path set to: ")
        path = "C:/Users/Kanne/Desktop/NBS-Snippets-Sensor-WC.s7k"
        print(path)

    path = input()
    # Setup dataset
    print("Path is: " + path)
    if(not(os.path.exists(path))):
        print("PATH DOESN'T EXIST!")
    dataset = pyread7k.FileDataset(path)

    # Highest and lowest point in dataset
    shallowestPoint = -1.7976931348623157e+308  # Smallest negative float that is not negative infinity
    deepestPoint = 1.7976931348623157e+308  # Biggest float that is not infinity

    # Largest and smallest values for length and width axis
    minLength = 1.7976931348623157e+308
    maxLength = -1.7976931348623157e+308
    minWidth = 1.7976931348623157e+308
    maxWidth = -1.7976931348623157e+308

    # List to save the boats coordinates for each ping
    pingBoatCoordList = []

    def Translate(points, bearing, dist):
        """Apply a translation to the points
        Args:
            points: An array of xyz points
            bearing: The angle they should be moved
            dist: A distance they should be moved
        Returns
            Translated xyz points
        """

        translatedPoints = points[:]
        translatedPoints[:, 0] += np.cos(bearing) * dist
        translatedPoints[:, 1] += np.sin(bearing) * dist

        return translatedPoints


    def Bearing(pointA: Point, pointB: Point):
        """Compute the bearing between to geolocations.
        Function to calculate the bearing between two points.
        There is a difference in return depending on order,
        i.e. bearing(a, b) != bearing(b, a).
        Args:
            pointA: The first point
            pointB: The other point
        Returns
            The angle between the two points as seen from point a.
        """
        latitudeA = math.radians(pointA[0])
        latitudeB = math.radians(pointB[0])

        deltaLongitude = math.radians(pointA[1] - pointB[1])

        x = math.cos(latitudeB) * math.sin(deltaLongitude)
        y = math.cos(latitudeA) * math.sin(latitudeB) - \
            (math.sin(latitudeA) * math.cos(latitudeB) * math.cos(deltaLongitude))

        return math.atan2(x, y)


    def BDA(ping):
        """Computes the raw points and indices for a ping.
        Args:
            ping: A .s7k ping
        Returns:
            indices: A list of the pings indices
            rawPoints: A list of the pings raw points
        """

        raw_rb = np.vstack(
            (
                ping.raw_detections.detections["detection_point"]
                * ping.sonar_settings.sound_velocity
                / (2 * ping.sonar_settings.sample_rate),
                ping.raw_detections.detections["rx_angle"],
            )
        ).T
        indices = np.array(list(zip(
            ping.raw_detections.detections["detection_point"].astype(int),
            ping.raw_detections.detections["beam_descriptor"].astype(int),
        )))

        rawPoints = np.array(
            [
                (
                    0,
                    -raw_rb[k, 0] * np.sin(raw_rb[k, 1]),
                    -raw_rb[k, 0] * np.cos(raw_rb[k, 1]),
                )
                for k in range(len(raw_rb))
            ]
        )

        return indices, rawPoints


    def RotationMatrix(pitch: float, roll: float, yaw: float):
        """Creating the rotation matrix from the boats position
        when recording the ping, by multiplying the three axes'
        rotation matrices
            Args:
                pitch: Pitch of the boat
                yaw: Yaw of the boat
                roll: Roll of the boat
            Returns:
                R: The rotation matrix
        """

        # Yaw rotation matrix - Rotation of yaw around the z-axis
        yawRotationMatrix = np.array([
                                        [np.cos(yaw), -np.sin(yaw), 0],
                                        [np.sin(yaw), np.cos(yaw), 0],
                                        [0, 0, 1],
                                     ])

        # Pitch rotation matrix - Rotation of pitch around the y-axis
        pitchRotationMatrix = np.array([
                                        [np.cos(pitch), 0, np.sin(pitch)],
                                        [0, 1, 0],
                                        [-np.sin(pitch), 0, np.cos(pitch)]
                                       ])

        # Roll rotation matrix - Rotation of roll around the x-axis
        rollRotationMatrix = np.array([
                                        [1, 0, 0],
                                        [0, np.cos(roll), -np.sin(roll)],
                                        [0, np.sin(roll), np.cos(roll)]
                                      ])

        return rollRotationMatrix @ pitchRotationMatrix @ yawRotationMatrix


    def Rotate(ping, indices: np.ndarray, points: np.ndarray):
        """Rotating points and correcting for heave

        Args:
            ping: A s7k ping
            indices: An array of indices
            points: The raw points
        Returns:
            rotatedPoints: The rotated points
            heave: The heave for the ping, used for boat point calculations
        """

        rotatedPoints = np.zeros_like(points)

        for i, idx in enumerate(indices):
            rollPitchHeave, h = ping.receiver_motion_for_sample(idx[0])
            eulerRoll = -np.arcsin(np.sin(rollPitchHeave.roll) / (np.cos(rollPitchHeave.pitch)))
            heave = rollPitchHeave.heave

            # Creating rotation matrix R
            R = RotationMatrix(rollPitchHeave.pitch, eulerRoll, h.heading)

            # Applying R to all points
            rotatedPoints[i, :] = np.dot(points[i, :].reshape(1, -1), R)

            # Correcting points for heave
            rotatedPoints[i, -1] += heave

        return rotatedPoints, heave


    def ToPointcloud(dataset):
        """Create points clouds from an iterable of pings
        This function is used to create x,y,z points from a pings amplitudes and motion parameters.
        Args:
            dataset: An iterable of pings
        Returns:
            pointcloud: All points sorted in pings from the .s7k file
        """

        print("Creating point cloud from pings...")
        for count in progressbar(range(1)):
            pointcloud = []
            pings = [p for p in dataset if len(p.position_set) > 0]
            firstPing = pings[0]

            # Iterating through every ping in the .s7k file
            for i, ping in enumerate(pings):
                if not len(ping.position_set):
                    continue
                if i > 0:
                    dist = distance(ping.gps_position, firstPing.gps_position).meters
                    angle = Bearing(ping.gps_position, firstPing.gps_position)
                else:
                    dist = 0
                    angle = 0

                # Computing raw points and indicies
                indices, rawPoints = BDA(ping)

                # Rotating points and computing heave for each ping
                rotatedPoints, heave = Rotate(ping, indices, rawPoints)

                # Translating rotated points to obtain the optimal transformed points
                transformedPoints = Translate(rotatedPoints, angle, dist)

                # Appending the transformed points to the pointcloud
                pointcloud.append(transformedPoints)

                # Calculating boat points
                boatPointX = dist * math.cos(angle)
                boatPointY = dist * math.sin(angle)

                # Appending the pings boat points to the list of boat points
                pingBoatCoordList.append([boatPointX, boatPointY, heave])

                # Finding the minimum and maximum values for each dimension, used to set slider values in Unity
                global deepestPoint, shallowestPoint, minLength, maxLength, minWidth, maxWidth
                for point in transformedPoints:
                    if point[2] > shallowestPoint:
                        shallowestPoint = point[2]
                    elif point[2] < deepestPoint:
                        deepestPoint = point[2]
                    if point[0] < minLength:
                        minLength = point[0]
                    elif point[0] > maxLength:
                        maxLength = point[0]
                    if point[1] < minWidth:
                        minWidth = point[1]
                    elif point[1] > maxWidth:
                        maxWidth = point[1]

            dataset.minimize_memory()

        print("Created point cloud!")
        return pointcloud


    def generate_json(pointcloud):
        """ Creates a JSON file from the pointcloud.
        Args:
            pointcloud: The computed pointcloud from the .s7k file
        """
        print("Generating JSON...")
        for count in progressbar(range(1)):

            ping_amount = len(pointcloud)
            number_of_counts_sum = 0
            for pings in pointcloud:
                number_of_counts_sum += len(pings)

            data = {}
            data["no_pings"] = ping_amount
            data["no_counts"] = number_of_counts_sum
            data["minimum_depth"] = math.ceil(shallowestPoint)
            data["maximum_depth"] = math.floor(deepestPoint)
            data["min_length_axis"] = math.floor(minLength)
            data["max_length_axis"] = math.ceil(maxLength)
            data["min_width_axis"] = math.floor(minWidth)
            data["max_width_axis"] = math.ceil(maxWidth)
            data["pings"] = []

            for i, pointRow in enumerate(pointcloud):
                ping = {
                    "pingID": i,
                    "no_points": len(pointRow),
                    "ping_boat_coord": pingBoatCoordList[i],
                    "coords_x": [],
                    "coords_y": [],
                    "coords_z": []
                }

                for x, y, z in pointRow:
                    ping["coords_x"].append(x)
                    ping["coords_y"].append(y)
                    ping["coords_z"].append(z)

                data["pings"].append(ping)

            with open("point_cloud_data.json", "w") as f:
                json.dump(data, f)
        print("Generated JSON!")

    pointcloud = ToPointcloud(dataset)
    generate_json(pointcloud)