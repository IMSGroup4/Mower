from rplidar import RPLidar
import time
lidar = RPLidar('/dev/ttyUSB0')

info = lidar.get_info()
print(info)

health = lidar.get_health()
print(health)
for i, scan in enumerate(lidar.iter_measures()):
    print('%d: Got %d measurements' % (i, len(scan)))
        #print("scan1",scan[1])
        #print("scan2",scan[2])
        #print("scan3",scan[3])
        #print("scan4",scan[4])
    #investigate values more carefully to find good values for autonomous driving
    if scan[3] != 0 and scan[3] < 200 and (scan[2] <= 30 or scan[2] >= 400):
        print("Collision flag ULLA")
        time.sleep(5)
#have to find a effective way of restarting the data collection
#the buffer is getting absolutely destroyed atm.

lidar.stop()
lidar.stop_motor()
lidar.disonnect()
