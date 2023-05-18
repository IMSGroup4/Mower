import math
from matplotlib import pyplot as plt
from shapely.geometry import Point
from operator import itemgetter
import os

def loadMultipleFiles():
    path = "LidarTesting/lidar_test"
    all_files = []
    for root, dirs, files in os.walk(path, topdown=False):
        for name in files:
            all_files.append(open("LidarTesting/lidar_test/" + name))
    
    return all_files
        


def loadData():
    data_file = open("lidar_data_iter_measures.txt")
    lidar_data = []
    for line in data_file.readlines():
        newLine = line[1:-2] 
        raw_object = newLine.split(", ")
        lidar_data.append([int(raw_object[1]), float(raw_object[2]), float(raw_object[3])])
    return lidar_data

def loadData(file):
    lidar_data = []
    for line in file.readlines():
        newLine = line[1:-2]
        raw_object = newLine.split(", ")
        lidar_data.append([int(raw_object[1]),float(raw_object[2]),float(raw_object[3])])
    return lidar_data

def cullData(data):
    culled_data = []
    for measurement in data:
        if measurement[2] == 0.0:
            pass
        elif measurement[0] != 15:
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

def calculateVectorsWithOffset(data,xOff=0,yOff=0):
    vectors = []
    for measurement in data:
        x = xOff + measurement[2] * math.cos(math.radians(measurement[1]))
        y = yOff + measurement[2] * math.sin(math.radians(measurement[1]))
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

def pickBestCandidate(buffer):
    length_vals = []
    for candidate in buffer:
        length_vals.append(candidate[2])
    i = length_vals.index(max(length_vals))
    return buffer[i]

def theReaping(data):
    reaped_data = []
    buffer = []
    deg = 0
    for value in data:
        if int(value[1]) == deg:
            buffer.append(value)
        else:
            if len(buffer) != 0:
                reaped_data.append(pickBestCandidate(buffer))
                deg +=1
                buffer.clear()
            else:
                deg +=1
    return reaped_data


def printPointsToTxt(points, filename):

    fixedFileName = filename + ".txt"
    fileobject = open(fixedFileName, "w+")
    for point in points:
        fileobject.write("({}, {}) \n".format(point.x, point.y))
    fileobject.close()
        

def main():
    files = loadMultipleFiles()
    print(len(files))

    all_data = []
    culled_data =[]
    reaped_data =[]
    for file in files:
        all_data.append(loadData(file))
    for data in all_data:
        culled_data.append(cullData(data))
    for data in culled_data:
        reaped_data.append(theReaping(sortByAngle(data)))
    print(len(reaped_data[0]))
    vector_list = []
    multiplier = 0
    """
    for data in reaped_data:
        vectors = calculateVectorsWithOffset(data,multiplier)
        plt.plot(multiplier,0, marker = "o", c="green")
        vector_list += vectors
        multiplier += 200
    print(len(vector_list))
    plotPoints(vector_list)
    """
    printPointsToTxt(calculateVectors(reaped_data[0]),"bajs")

    
    

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