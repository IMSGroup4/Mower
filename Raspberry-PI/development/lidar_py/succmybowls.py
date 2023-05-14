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
        lidar.reset()
        iterator = lidar.iter_measures()
        distances = []
        angles = []
        try:
            #items = [item for item in next(iterator)]
            
        except RPLidarException:
            lidar.clean_input()
            
        distances = [item[2] for item in items]
        angles = [item[1] for item in items]
        
        #print(distances)
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
