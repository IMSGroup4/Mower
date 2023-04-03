import numpy as np
import math
from turtle import *


def getMotorSpeeds(velocity, omega, L=0.15, r=0.02):
    return(velocity - (L/2)*omega)/r, (velocity + (L/2)*omega)/r

def ddr_fk(phidot_L, phidot_r, L=0.15, r=0.02):
    return 
            
def main():
    L, R = getMotorSpeeds(velocity=0.05, omega=2 *math.pi)
    print("Left: {}, Right:{}".format(math.degrees(L),math.degrees(R)))

if __name__ == "__main__":
    main()
        
        
        