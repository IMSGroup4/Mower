from rplidar import RPLidar
import time
import sys

lidar = RPLidar('/dev/ttyUSB0')

info = lidar.get_info()
print(info)

health = lidar.get_health()
print(health)
for i, scan in enumerate(lidar.iter_measures()):
    if scan[1] == 0:
        continue
    elif (scan[2] > 340 or scan[2] < 20) and 300 > scan[3] > 0:
        print(f"What are you doing stepmotor! Collision ahead {scan}")

lidar.stop()
lidar.stop_motor()
lidar.disonnect()
