from rplidar import RPLidar
import time
import sys

lidar = RPLidar('/dev/ttyUSB0')

info = lidar.get_info()
print(info)
def forward_detection():
    health = lidar.get_health()
    print(health)
    findings = []
    for i, scan in enumerate(lidar.iter_measures()):
        if scan[1] == 0:
            continue
        elif (scan[2] > 340 or scan[2] < 20) and 300 > scan[3] > 0:
            print(f"What are you doing stepmotor! Collision ahead {scan}")
            findings.append(scan)
        elif len(findings) >= 10:
            break

    lidar.stop()
    lidar.stop_motor()
    lidar.disonnect()
    return findings

def evaluate_findings(findings):
    degrees = []
    distances = []
    for finding in findings:
        deg = 0
        if finding[2] > 180:
            deg = 360 - finding[2]
        else:
            deg = finding[2]       
        degrees.append(deg)
        distances.append(finding[3])
    avg_deg = sum(degrees) / len(degrees)
    avg_len = sum(distances) / len(distances)
    print("ULLA SHOULD TURN {} Degrees because the object is {} points in that direction".format(avg_deg, avg_len))
    
        

def main():
    objectbro = forward_detection()
    evaluate_findings(objectbro)
if __name__ == "__main__":
    main()
