import math
import numpy


joystick_coords = [(1,0), (0,1), (-1, 0), (0,-1),(0, 0.8)]
forward_angle = (0,1)


def convert_joystick_to_angle(coords):
    angles = []
    forward_angle_length = math.sqrt(pow(forward_angle[0],2) + pow(forward_angle[1],2))
    for item in coords:
        dot = numpy.dot(item, forward_angle)
        item_length = math.sqrt(pow(item[0],2) + pow(item[1], 2))
        angle = dot/(item_length * forward_angle_length)
        if item[0] >= 0:
            deg_angle = math.degrees(math.acos(angle))
        else:
            deg_angle = -math.degrees(math.acos(angle))
        print("Joystick Setting:    {} , Angle:    {} , VectorLength:   {}".format(item, deg_angle, item_length))
    return angles

def convert_angle_to_motorspeed(angle, speed):
    pass

def trashpanda_run(joystick_coords):
    speeds = []
    for item in joystick_coords:
        item_length = math.sqrt(pow(item[0],2) + pow(item[1], 2))
        big_speed = 180
        direction = 1
        if item[1] < 0:
            direction = -1
        leftMotorDifferential = abs((item[0] + 1) / 2)
        rightMotorDifferential = (1 - leftMotorDifferential)
        leftMotorDifferential *= direction
        rightMotorDifferential *= direction
        #print("LEFT MOTOR:  {}, RIGHT MOTOR:    {}".format(leftMotorDifferential,rightMotorDifferential))
        speed = int(big_speed * item_length)
        rightMotor = rightMotorDifferential * speed
        leftMotor = leftMotorDifferential * speed
        speeds.append((leftMotor,rightMotor))
    return speeds



def main():
    angles = convert_joystick_to_angle(joystick_coords)
    speeds = trashpanda_run(joystick_coords)
    for speed in speeds:
        print("RIGHT MOTOR: {}, LEFT MOTOR {}".format(speed[0], speed[1]))
    #print(angles)
if __name__ == "__main__":
    main()