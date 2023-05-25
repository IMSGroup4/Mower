class Positioning:
    def __init__(self):
        self.leftSpeed = 0
        self.rightSpeed = 0
        self.angle = 0
        self.posX = 0
        self.posY = 0
class LidarData:
    def __init__(self):
        self.forwardDetection = False
        self.obstacleDetected = False
        self.avg_deg = 0
        self.avg_len = 0
