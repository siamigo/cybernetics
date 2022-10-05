from statistics import variance, covariance
from numpy import *
def readFile(filename):
    dataSet1=[]
    dataSet2=[]
    dataset3=[]

    with open(filename, 'r') as dataFile:
        headerLine = dataFile.readline()
        headerLine = headerLine.strip("\n")
        for line in dataFile:
            line = line.strip("\n")
            lineList = line.split(" ")
            dataSet1.append(float(lineList[1]))
            dataSet2.append(float(lineList[2]))
            dataset3.append(float(lineList[3]))
    return dataSet1, dataSet2, dataset3

def calibrateGyrometer(gyr_x, gyr_y, gyr_z):
    gyr_x_error=gyr_x/131/len(gyr_x)
    gyr_y_error=gyr_y/131/len(gyr_y)
    gyr_z_error=gyr_z/131/len(gyr_z)
    return gyr_x_error, gyr_y_error, gyr_z_error

def calibrateAccelerometer(acc_x, acc_y, acc_z):
    acc_x_error=acc_x/16384/len(acc_x)
    acc_y_error=acc_y/16384/len(acc_y)
    acc_z_error=acc_z/16384/len(acc_z)
    return acc_x_error, acc_y_error, acc_z_error

# Function taken from Implementation of Kalman Filter with Python Language  by Mohamed LAARAIEDH
def kalmanFilter(X, P, A, Q, B, U):
    X = dot(A, X) + dot(B, U)
    P = dot(A, dot(P, A.T)) + Q
    return(X,P)

dt = 0.5 #Give it an actually value
variance2 = 15 #Give it an actually value

X = array([[0.0], [0.0], [0.0]])
P = diag((0.0, 0.0, 0.0))
A = array([[1, dt, 0.5*(dt**2)], [0.0, 1., dt], [0.0, 0.0, 1]]) 
Q = array[[(dt**6)/36, (dt**5)/12, (dt**4)/6], [(dt**5)/12, (dt**4)/6, (dt**3)/2], [(dt**4)/6, (dt**3)/2, dt**2]]*variance2
