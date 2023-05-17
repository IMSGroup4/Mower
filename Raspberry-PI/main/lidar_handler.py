from rplidar import RPLidar
from rplidar import RPLidarException
import time
import sys
import serial


class CollisionDetector:
    def __init__(self):
        pass
        #self.address = address
        #self.lidar = RPLidar(address)
        
        #print("LIDAR CONNECTED")

    def forward_detection(self, lidar):
        print("Enter forward detection")
        findings = []
        print("findings:", findings)
        #lidar.reset()
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
                elif i > 3000:
                    print("ZERO ZERO EXXIT")
                    return (0, 0)
                elif (scan[2] > 335 or scan[2] < 25) and 300 > scan[3] > 0:
                    print(f"What are you doing stepmotor! Collision ahead {scan}")
                    findings.append(scan)
                elif len(findings) >= 5:
                    avg_deg, avg_len = self.evaluate_findings(findings)
                    print("GOT DEG AND LEN {}".format(avg_deg))
                    #print("AVG DEG {}: AVG LEN {}".format(round(avg_deg)), format(round(avg_len)))
                    findings.clear()
                    #self.lidar.reset()
                    print("LEFT SCAN")
                    #lidar.clean_input()
                    
                    return avg_deg, avg_len
            print("EXITED FOR LOOP")
        except Exception as e:
            print(e)
            if e is KeyboardInterrupt:
                sys.exit()
            print("Cleaning")
            if e is RPLidarException:
                lidar.clean_input()

    def evaluate_findings(self,findings):
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
        
