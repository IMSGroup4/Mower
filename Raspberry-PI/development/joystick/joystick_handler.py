import random
import math

class  JoystickHandler:
    def __init__(self,xin,yin):
        self.x = xin
        self.y = yin    

    def update_x_and_y(self):
        self.x = random.randint(-512,512)
        self.y = random.randint(-512,512)
    

    def convert_to_degrees(self):
        origo = (0,0)
        if ((self.x,self.y) == origo):
            print("Center")
        else:
            vector1 = (self.x - origo[0],self.y - origo[1])

            vector2 = (0, 512)
            if self.x <0:
                print(-math.degrees(self.angle(vector1,vector2)))
            else:
                print(math.degrees(self.angle(vector1,vector2)))
            if(math.degrees(self.angle(vector1,vector2))>160):
                print("-----------------------------------",math.degrees(self.angle(vector1,vector2)))

    def dotproduct(self,v1,v2):
        return sum((a*b) for a, b in zip(v1,v2))

    def lenght(self,v):
        return math.sqrt(self.dotproduct(v,v))

    def angle(self,v1,v2):
        return math.acos(self.dotproduct(v1,v2) / (self.lenght(v1)*self.lenght(v2)))

def main():
    handler = JoystickHandler(0,0)
    print(handler.lenght((0,512/2)))
    """
    if length > 512:
    speed = 1

    else:
        speed = length/ 512
    
    Serial.butthole.sendData(speedMultiplier)

    ################## PÅ DET ANDRA HÅLLET!
    int maxSpeed = 180;    


    """
    for i in range(-512,512):
        for j in range(-512,512):
            handler = JoystickHandler(i,j)
            handler.convert_to_degrees()
            """
if __name__ == "__main__":
    main()