from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import RPLidarA1 as LaserModel
from rplidar import RPLidar as Lidar


MAP_SIZE_PIXELS         = 500
MAP_SIZE_METERS         = 10
LIDAR_DEVICE            = '/dev/ttyUSB0'


# Ideally we could use all 250 or so samples that the RPLidar delivers in one 
# scan, but on slower computers you'll get an empty map and unchanging position
# at that rate.
MIN_SAMPLES   = 200

#info = lidar.get_info()
#print(info)

lidar = Lidar(LIDAR_DEVICE)
mapbytes = bytearray(800*800)

slam = RMHC_SLAM(lidar, 800, 35)

while True:
	#lidarData = readLidar()
	
	slam.update(lidarData)
	
	x, y, theta = slam.getpos()
	slam.getmap(mapbytes)
