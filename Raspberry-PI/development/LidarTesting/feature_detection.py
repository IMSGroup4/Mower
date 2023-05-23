import numpy as np
import math
from fractions import Fraction
from scipy.odr import *

class featureDetection:
    def __init__(self):
        #variables
        self.EPSILON = 10
        self.DELTA = 501
        self.SNUM = 6
        self.PMIN = 20
        self.GMAX = 20
        self.SEED_SEGMENTS = []
        self.LINE_SEGMENTS = []
        self.LASERPOINTS = []
        self.LINE_PARAMS = None
        self.NP = len(self.LASERPOINTS)-1
        self.LMIN = 20 # minimum length of a line segment
        self.LR = 0 # Real length of a line segment
        self.PR = 0 # The number of laser points contained in the line segment
        
        #euclidian distance from point1 to point2
        def distance_point2point(self, point1, point2):
            px = (point1[0] - point2[0]) **2
            py = (point1[1] - point2[1]) **2
            return math.sqrt(px + py)
        
        #distance point to line in the general form
        def distance_point2line(self, params, point):
            A,B,C = params
            distance = abs(A* point[0] + B * point[1] + C) / math.sqrt(A**2 + B**2)
            return distance
