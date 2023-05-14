from rplidar import RPLidar
from rplidar import RPLidarException
import time
import sys
import serial


class CollisionDetector:
    def __init__(self, address):
        #self.address = address
        self.lidar = RPLidar(address)
        
        print("LIDAR CONNECTED")

    def forward_detection(self):
        print("Enter forward detection")
        findings = []
        print("findings:", findings)
        #lidar.reset()
        print("LIDAR RESET")
        while True:        
            self.lidar.clean_input()
            #print("Getting info")
            #info = self.lidar.get_info()
            print("Entering for loop")
            #print(info)
            try:
                for i, scan in enumerate(self.lidar.iter_measures()):
                    #print("INSIDE FOR LOOP")
                    #print(scan)
                    if scan[1] == 0:
                        continue
                    elif (scan[2] > 335 or scan[2] < 25) and 300 > scan[3] > 0:
                        #print(f"What are you doing stepmotor! Collision ahead {scan}")
                        findings.append(scan)
                    elif len(findings) >= 10:
                        avg_deg, avg_len = self.evaluate_findings(findings)
                        findings.clear()
                        #self.lidar.reset()
                        print("LEFT SCAN")
                        return round(avg_deg), round(avg_len)
                print("EXITED FOR LOOP")
            except Exception as e:
                if e is KeyboardInterrupt:
                    sys.exit()
                print("Cleaning")
                self.lidar.clean_input()

    def evaluate_findings(self,findings):
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
        return avg_deg, avg_len
        
