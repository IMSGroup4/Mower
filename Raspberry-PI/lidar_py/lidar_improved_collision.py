from rplidar import RPLidar
from rplidar import RPLidarException
import time
import sys
import serial

ser = serial.Serial("/dev/ttyUSB1", 115200, timeout=1)

def forward_detection():
    print("Enter forward detection")
    #health = lidar.get_health()
    #print(health)
    findings = []
    print("findings:", findings)
    while True:        
        print("Enter for-loop")
        lidar = RPLidar('/dev/ttyUSB0')
        info = lidar.get_info()
        print(info)
        try:
            for i, scan in enumerate(lidar.iter_measures()):
                #print(scan[2])
                #print(f"len of finding{len(findings)}")
                if scan[1] == 0:
                    continue
                elif (scan[2] > 335 or scan[2] < 25) and 400 > scan[3] > 0:
                    print(f"What are you doing stepmotor! Collision ahead {scan}")
                    findings.append(scan)
                elif len(findings) >= 10:
                    evaluate_findings(findings)
                    findings.clear()
                    lidar.reset()
                    print("LEFT SCAN")
        except RPLidarException:
            lidar.clean_input()
    print("Exit forward detection")

def evaluate_findings(findings):
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
    
    print("ULLA SHOULD TURN {} Degrees because the object is {} points in that direction".format(avg_deg, avg_len))
    send_data = f"{1337},{int(avg_deg)}"
    print(f"send_data has: {send_data}")
    ser.write(send_data.encode('utf-8'))
    
        

def main():
    while True:
        print("Enter Main")
        objectbro = ""
        forward_detection()
        #evaluate_findings(objectbro)
        #lidar.reset()
        #time.sleep(5)
        print("Exit main")
if __name__ == "__main__":
    main()
