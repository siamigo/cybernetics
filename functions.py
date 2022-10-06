from statistics import variance
import numpy as np
def readFile(filename):
    data_raw = np.loadtxt(filename, delimiter=',', skiprows=0, dtype=float)
    data=[[],[],[]]
    for i in range(0, len(data_raw[0])):
        data[i]=data_raw[:,i]
    return data

def sensorError(sensor_raw):
    sensor_error = np.empty((len(sensor_raw),1), float)
    for i in range(0,len(sensor_raw)):
        sensor_error[i]=np.sum(np.subtract(sensor_raw[i], 1))/len(sensor_raw[i])
    return sensor_error

# Function taken from Implementation of Kalman Filter with Python Language  by Mohamed LAARAIEDH
def kalmanFilter(X, P, A, Q, B, U):
    X = np.dot(A, X) + np.dot(B, U)
    P = np.dot(A, np.dot(P, A.T)) + Q
    return(X,P)

acc_raw=readFile('accCalib.txt')
acc_error = sensorError(acc_raw)
print(acc_error)
