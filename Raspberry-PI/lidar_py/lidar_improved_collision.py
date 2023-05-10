from rplidar import RPLidar
from rplidar import RPLidarException
import time
import sys
import serial


class CollisionDetector:       
    def forward_detection(self):
        print("Enter forward detection")
        findings = []
        print("findings:", findings)
        while True:        
            print("Enter for-loop")
            lidar = RPLidar('/dev/ttyUSB0')
            info = lidar.get_info()
            print(info)
            try:
                for i, scan in enumerate(lidar.iter_measures()):
                    if scan[1] == 0:
                        continue
                    elif (scan[2] > 335 or scan[2] < 25) and 400 > scan[3] > 0:
                        print(f"What are you doing stepmotor! Collision ahead {scan}")
                        findings.append(scan)
                    elif len(findings) >= 10:
                        self.evaluate_findings(findings)
                        findings.clear()
                        lidar.reset()
                        print("LEFT SCAN")
            except RPLidarException:
                lidar.clean_input()

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
        
