from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import RPLidarA1 as LaserModel
from rplidar import RPLidar as Lidar
from rplidar import RPLidarException

MAP_SIZE_PIXELS = 500
MAP_SIZE_METERS = 10
LIDAR_DEVICE = '/dev/ttyUSB0'

MIN_SAMPLES = 360
def main():
    lidar = Lidar(LIDAR_DEVICE)
    slam = RMHC_SLAM(LaserModel(), MAP_SIZE_PIXELS,MAP_SIZE_METERS)
    trajectory = []
    mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)
    
    
    previous_distance = None
    previous_angle = None
    while True:
        distances = []
        angles = []
        try:
            for i, scan in lidar.iter_scans():
                distances.append(scan[2])
                angles.append(scan[1])
                if i > 500:
                    break
        except RPLidarException:
            lidar.clean_input()
            
        if len(distances) > MIN_SAMPLES:
                slam.update(distances, scan_angles_degrees=angles)
                previous_distance = distances.copy()
                previous_angle = angles.copy()
        elif previous_distance is not None:
            slam.update(previous_distance, scan_angles_degrees=previous_angle)
        
        x, y, theta = slam.getpos()
        print("X: {}, Y: {}".format(x,y))
        
if __name__ == "__main__":
    main()