from rplidar import RPLidar
from rplidar import RPLidarException
import time
import sys
import serial
from location_data import LidarData
#from main import lidarData

lidar = RPLidar('/dev/ttyUSB0')
lidarData = LidarData()

def forward_detection():
    print("Enter forward detection")
    findings = []
    print("findings:", findings)
    lidar.reset()
    print("LIDAR RESET")    
    #lidar.clean_input()
    #print("Getting info")
    #info = self.lidar.get_info()
    print("Entering for loop")
    #print(info)
    try:
        for i, scan in enumerate(lidar.iter_measures()):
            #print("INSIDE FOR LOOP")
            #print(scan)
            if scan[1] == 0:
                continue
            elif (scan[2] > 325 or scan[2] < 35) and 300 > scan[3] > 0:
                print(f"What are you doing stepmotor! Collision ahead {scan}")
                findings.append(scan)
            elif len(findings) >= 2:
                avg_deg, avg_len = evaluate_findings(findings)
                #print("GOT DEG AND LEN {}".format(avg_deg))
                #print("AVG DEG {}: AVG LEN {}".format(round(avg_deg)), format(round(avg_len)))
                findings.clear()
                #self.lidar.reset()
                print("LEFT SCAN")
                #lidar.clean_input()
                lidarData.avg_deg = avg_deg
                lidarData.avg_len = avg_len
                lidarData.obstacleDetected = True
                print("LIDARDATA DEG {} LEN {}  DETECTED {}".format(lidarData.avg_deg,lidarData.avg_len, lidarData.obstacleDetected))
                #return avg_deg, avg_len
        print("EXITED FOR LOOP")
    except Exception as e:
        print(e)
        if e is KeyboardInterrupt:
            sys.exit()
        print("Cleaning")
        if e is RPLidarException:
            lidar.clean_input()

def evaluate_findings(findings):
    print("EVALUATING: {}".format(findings))
    degrees = []
    distances = []
    for finding in findings:
        deg = 0
        if finding[2] > 180:
            deg = finding[2] - 360 
        else:
            deg = finding[2]       
        degrees.append(deg)
        distances.append(finding[3])
    avg_deg = sum(degrees) / len(degrees)
    avg_len = sum(distances) / len(distances)
    print("AVG DEGDEG {}, AVG LENLEN{}".format(avg_deg,avg_len))
    return avg_deg, avg_len

def main():
    #lidarData.forwardDetection = True
    
    while True:
        try:
            #lidar.reset()
            forward_detection()
        except Exception as e:
            print(e)

    
