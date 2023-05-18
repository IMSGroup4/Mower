
#!/usr/bin/env python3

'''
rpslam.py : BreezySLAM Python with SLAMTECH RP A1 Lidar
                 
Copyright (C) 2018 Simon D. Levy
This code is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as 
published by the Free Software Foundation, either version 3 of the 
License, or (at your option) any later version.
This code is distributed in the hope that it will be useful,     
but WITHOUT ANY WARRANTY without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU Lesser General Public License 
along with this code.  If not, see <http://www.gnu.org/licenses/>.
'''

MAP_SIZE_PIXELS         = 500
MAP_SIZE_METERS         = 1
LIDAR_DEVICE            = '/dev/ttyUSB0'


# Ideally we could use all 250 or so samples that the RPLidar delivers in one 
# scan, but on slower computers you'll get an empty map and unchanging position
# at that rate.
MIN_SAMPLES   = 2

from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import RPLidarA1 as LaserModel
from rplidar import RPLidar as Lidar
from rplidar import RPLidarException
from roboviz import MapVisualizer
import time

if __name__ == '__main__':

    # Connect to Lidar unit
    lidar = Lidar(LIDAR_DEVICE)

    # Create an RMHC SLAM object with a laser model and optional robot model
    slam = RMHC_SLAM(LaserModel(), MAP_SIZE_PIXELS, MAP_SIZE_METERS)

    # Set up a SLAM display
    viz = MapVisualizer(MAP_SIZE_PIXELS, MAP_SIZE_METERS, 'SLAM', show_trajectory=False)

    # Initialize an empty trajectory
    trajectory = []

    # Initialize empty map
    mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)

    # Create an iterator to collect scan data from the RPLidar
    

    # We will use these to store previous scan in case current scan is inadequate
    previous_distances = 500
    previous_angles    = 10

    # First scan is crap, so ignore it
    width = 1000
    height = 1000
    while True:
        try:
            
            iterator = lidar.iter_scans()
            #next(iterator)
            ############################################################
            #distances = []
            #angles = []
            """
            for i, scan in enumerate(lidar.iter_scans()):
                distances.append(scan[2])
                angles.append(scan[1])
                if i > 10:
                    break"""
            #print(distances)
            ############################################################
            # Extract (quality, angle, distance) triples from current scan
            
            print("FETCHING ITEMS")
            items = [item for item in next(iterator)]
            print("FETCHED")

            # Extract distances and angles from triples

            distances = [item[2] for item in items]
            angles    = [item[1] for item in items]
            
            # Update SLAM with current Lidar scan and scan angles if adequate
            if len(distances) > MIN_SAMPLES:
                print("BAJS")
                slam.update(distances, scan_angles_degrees=angles)
                print("BAJS")
                previous_distances = distances.copy()
                previous_angles    = angles.copy()
                #print(distances)
            # If not adequate, use previous
            elif previous_distances is not None:
                slam.update(previous_distances, scan_angles_degrees=previous_angles)
            # Get current robot position
            x, y, theta = slam.getpos()
            print(x,y)
            # Get current map bytes as grayscale
            slam.getmap(mapbytes)
            #slam.getpos() works and gives current location now parse the data
            #print(slam.getpos())
            # Display map and robot pose, exiting gracefully if user closes it
            #viz.display(x/1000.,y/1000.,theta,mapbytes)
            if not viz.display(x/1000., y/1000., theta, mapbytes):
                exit(0)
        except RPLidarException:
            print("SKEEP")
            #time.sleep(2)
            lidar.clean_input()
            print("INPUT CLEANED")
            lidar.reset()
            #iterator = lidar.iter_scans()
 
    # Shut down the lidar connection
    lidar.stop()
    lidar.disconnect()
