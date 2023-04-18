import math
from matplotlib import pyplot as plt
from shapely.geometry import Point
from operator import itemgetter
import os

def loadMultipleFiles():
    pass


def loadData():
    data_file = open("lidar_data_iter_measures.txt")
    lidar_data = []
    for line in data_file.readlines():
        newLine = line[1:-2] 
        raw_object = newLine.split(", ")
        lidar_data.append([int(raw_object[1]), float(raw_object[2]), float(raw_object[3])])
    return lidar_data

def cullData(data):
    culled_data = []
    for measurement in data:
        if measurement[2] == 0.0:
            pass
        else:
            culled_data.append(measurement)
    return culled_data

def sortByAngle(data):
    return sorted(data, key=itemgetter(1))

def calculateVectors(data):
    vectors = []
    for measurement in data:
        x = measurement[2] * math.cos(math.radians(measurement[1]))
        y = measurement[2] * math.sin(math.radians(measurement[1]))
        vectors.append(Point(x,y))
    return vectors
def plotPoints(points):
    i = 0
    for point in points:
        plt.plot(point.x,point.y,marker="o", c="red")
        print("{} of {}".format(i, len(points)))
        i += 1
    print("Showing")
    ax = plt.gca()
    ax.set_aspect('equal', adjustable='box')
    plt.show()
        

def main():
    path = "LidarTesting/lidar_test"
    for root, dirs,files in os.walk(path, topdown=False):
        for name in files:
            print(name)
            #THIS WORKS USE THIS TO OPEN THE TEST FILES
    """
    raw_data = loadData()
    print(len(raw_data))
    culled_data = cullData(raw_data)
    print(len(culled_data))
    culled_data = sortByAngle(culled_data)
    vectors = calculateVectors(culled_data)
    plotPoints(vectors)
    """

if __name__ == "__main__":
    main()