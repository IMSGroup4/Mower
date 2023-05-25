import threading
import serial
import time
import math
from matplotlib import pyplot as plt
#from main import ser
from restAPI_handler import RestAPIHandler
#from ser_read import leftSpeed,rightSpeed,angle
#import ser_read
from ser_read import position
#ser = serial.Serial("/dev/ttyUSB1", 115200, timeout=1)
#time.sleep(3)
#send_data = f'10,0'
#ser.write(send_data.encode('utf-8'))
restAPI = RestAPIHandler()
wheelCirc = math.pi * 0.045
leftMotorSpeed = 0
rightMotorSpeed = 0


def coordinateConversion():
    global times
    global posY
    global posX
    global coordinateList
    print("COORDINATE CONVERSTION ENTRY: LEFT {}:   RIGHT: {}   ANGLE: {}".format(position.leftSpeed,position.rightSpeed,position.angle))
    motorTime = time.time() * 1000
    if times > 0:
        elapsedTime = motorTime - times
        leftMotorSpeed = wheelCirc * position.leftSpeed * (elapsedTime / 60000)
        rightMotorSpeed = wheelCirc * position.rightSpeed * (elapsedTime /60000)
        averageSpeed = (leftMotorSpeed+rightMotorSpeed) / 2
        newX = posX + (math.cos(math.radians(position.angle)) * averageSpeed) 
        newY = posY + (math.sin(math.radians(position.angle)) * averageSpeed)
        print("leftspeed: {}    rightspeed: {}   ANGLE {}:".format(position.leftSpeed,position.rightSpeed,position.angle))
        print("X: {}    Y: {}   ANGLE {}:".format(newX,newY,position.angle))
        posX = newX
        posY = newY
        restAPI.position_send(round(posX*100),round(posY*100))
        #coordinateList.append((posX,posY))
        """if len(coordinateList) > 3000:
            print("PLOTTING")
            for coordinate in coordinateList:
                plt.plot(coordinate[0],coordinate[1],marker="o", c="red")
            ax = plt.gca()
            ax.set_aspect('equal', adjustable='box')
            plt.show()
        """
        position.posX = posX
        position.posY = posY
        times = motorTime
    else:
        times = motorTime
def main():
    global times 
    global posX
    global posY
    global coordinateList

    posX = 0
    posY = 0
    times = 0
    coordinateList = []
    while True:
        #print("LEFT: {}, RIGHT: {}  ANG: {}".format(leftSpeed,rightSpeed, angle))
        coordinateConversion()
        print("Sent coordinates to backend______")
        #time.sleep(0.2)
if __name__ == "__main__":
    main()
    