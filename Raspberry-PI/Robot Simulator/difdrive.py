import pygame
import math
import os

class Envir:
    def __init__(self, dimensions):
        #Colors
        self.black = (0,0,0) 
        self.white = (255,255,255)
        self.green = (0,255,0)
        self.blue = (0,0,255)
        self.red = (255,0,0)
        self.yellow = (255,255,0)
        # Map dimentions
        self.height = dimensions[0]
        self.width = dimensions[1]
        #Window setup
        pygame.display.set_caption("ULLA DRIVE SIM")
        self.map = pygame.display.set_mode((self.height, self.width))
        self.font = pygame.font.Font('freesansbold.ttf', 50)
        self.text = self.font.render('default', True, self.white, self.black)
        self.textRect = self.text.get_rect()
        self.textRect.center = (dimensions[0] -600, dimensions[1]-100)
        self.trail_set = []

    def write_info(self, Vl,Vr, theta):
        txt = f"Vl = {Vl}   Vr = {Vr}   theta = {int(math.degrees(theta))}"
        self.text = self.font.render(txt, True, self.white, self.black)
        self.map.blit(self.text,self.textRect)

    def trail(self, pos):
        for i in range(0, len(self.trail_set)-1):
            pygame.draw.line(self.map,
                            self.green,
                             (self.trail_set[i][0], self.trail_set[i][1]),
                             (self.trail_set[i+1][0], self.trail_set[i+1][1]))
        if self.trail_set.__sizeof__() > 50000:
            self.trail_set.pop(0)
        self.trail_set.append(pos)




class Ulla:
    def __init__(self,startPos, robotImg, width):
        self.m2p = 3779.52 #Meters 2 Pixels
        self.w = width
        self.x = startPos[0]
        self.y = startPos[1]
        #Theta is heading angle!
        self.theta = 0
        self.velocityLeft = 0.01 * self.m2p  #meter/s
        self.velocityRight = 0.01 * self.m2p #meter/s
        self.maxspeed = 0.02 * self.m2p
        self.minspeed = -0.02 * self.m2p
        self.img = pygame.image.load(robotImg)
        self.rotated=self.img
        self.rect = self.rotated.get_rect(center=(self.x, self.y))

    def draw(self,map):
        map.blit(self.rotated, self.rect)
    
    def move(self,dt, event=None):

        if event is not None:
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_KP4:
                    self.velocityLeft+= 0.001*self.m2p
                elif event.key == pygame.K_KP1:
                    self.velocityLeft-= 0.001*self.m2p
                elif event.key == pygame.K_KP6:
                    self.velocityRight+= 0.001 * self.m2p
                elif event.key == pygame.K_KP3:
                    self.velocityRight -= 0.001 * self.m2p
                else:
                    pass
        self.x += ((self.velocityLeft + self.velocityRight)/2) * math.cos(self.theta) * dt
        self.y -= ((self.velocityLeft + self.velocityRight)/2) * math.sin(self.theta) * dt
        self.theta += (self.velocityRight-self.velocityLeft)/self.w* dt

        self.rotated=pygame.transform.rotozoom(self.img,
                                               math.degrees(self.theta),
                                               1)
        self.rect = self.rotated.get_rect(center=(self.x,self.y))
        
1
def main():
    #initialization
    pygame.init()
    #start_position
    start=(200,200)
    #dimensions
    dims = (1200,800)

    running = True
    #Environment
    environment = Envir(dims)

    dt = 0
    lasttime = pygame.time.get_ticks()
    # ULLA
    path = os.path.abspath("DDR.png")
    ulla = Ulla(start, path, 80)

    # simulation loop
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            ulla.move(dt,event)
        dt=(pygame.time.get_ticks()-lasttime)/1000
        lasttime=pygame.time.get_ticks()
        pygame.display.update()
        environment.map.fill(environment.black)
        ulla.move(dt)
        ulla.draw(environment.map)
        environment.trail((ulla.x, ulla.y))
        environment.write_info(int(ulla.velocityLeft), int(ulla.velocityRight), ulla.theta)
        
if __name__ == "__main__":
    main()