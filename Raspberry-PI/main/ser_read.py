import serial
import time
from main import ser
from location_data import Positioning
#ser = serial.Serial("/dev/ttyUSB1", 115200, timeout=1)
#time.sleep(3)
#send_data = f'10,0'
#ser.write(send_data.encode('utf-8'))
global leftSpeed
global rightSpeed
global angle
position = Positioning()
leftSpeed = 0
rightSpeed = 0
angle = 0
def main():
    global leftSpeed
    global rightSpeed
    global angle
    global position
    while True:
        response = ser.readline().decode('utf-8').rstrip()
        if len(response) > 0:
            #print("Arduino sent:", response)
    
            vals = response.split(",")
            leftSpeed = vals[0]
            rightSpeed = vals[1]
            angle = vals[2]
            position.leftSpeed = int(leftSpeed)
            position.rightSpeed = int(rightSpeed)
            position.angle = int(angle)