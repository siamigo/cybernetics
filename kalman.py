<<<<<<< HEAD
from functions import *
import numpy as np

acc_x_cali, acc_y_cali, acc_z_cali = readFile('calibration_accelerometer.txt')
acc_x_error, acc_y_error, acc_z_error = errorAccelerometer(acc_x_cali, acc_y_cali, acc_z_cali)

gyr_x_cali, gyr_y_cali, gyr_z_cali = readFile('calibration_gyrometer.txt')
gyr_x_error, gyr_y_error, gyr_z_error = errorGyrometer(gyr_x_cali, gyr_y_cali, gyr_z_cali)

millisOld=0

#--------------------- Not actual values---------------------
acc_x_raw=0
acc_y_raw=0 
acc_z_raw=0
gyr_x_raw=0 
gyr_y_raw=0 
gyr_z_raw=0
millis=0
while True:
    acc_x=acc_x_raw-acc_x_error
    acc_y=acc_y_raw-acc_y_error
    acc_z=acc_z_raw-acc_z_error
    gyr_x=gyr_x_raw-gyr_x_error
    gyr_y=gyr_y_raw-gyr_y_error
    gyr_z=gyr_z_raw-gyr_z_error


    thetaM=np.arctan2(-acc_x/9.8, np.sqrt(acc_y*acc_y+acc_z*acc_z)/9.8)*(180./np.pi)
    phiM=-np.arctan2(acc_y/9.8, np.sqrt(acc_x*acc_x+acc_z*acc_z)/9.8)*(180./np.pi)
    
    dt=(millis-millisOld)/1000.
    millisOld=millis
    
    theta=(theta+gyr_y*dt)*0.95+thetaM*0.05
    phi=(phi-gyr_x*dt)*0.95+phiM*0.05

    
    
    
=======
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
>>>>>>> fa4552361fefa3f12bc34b092c807512c0d8911e
