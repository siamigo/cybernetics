from statistics import variance
import numpy as np

# Should be able to make X-D amount of uknown data, and place it in the same dataset array.
def readFile(filename):
    data_raw = np.loadtxt(filename, delimiter=',', skiprows=0, dtype=float)
    data=[]
    for i in range(0, len(data_raw[0])):
        data.append([])
        data[i]=data_raw[:,i]
    return data
# Calculates error from the closes integer.
def sensorError(sensor_raw):
    sensor_error = np.empty((len(sensor_raw),1), dtype=float)
    for i in range(0,len(sensor_raw)):
        realNumber=int(sum(sensor_raw[i])/np.shape(sensor_raw[i]))
        sensor_error[i]=np.sum(np.subtract(sensor_raw[i], realNumber))/len(sensor_raw[i])
    return sensor_error

def removeSensorError(sensor_raw, sensor_error):
    sensor_value=sensor_raw-sensor_error
    return sensor_value

# Function taken from Implementation of Kalman Filter with Python Language  by Mohamed LAARAIEDH
def kalmanFilter(X, P, A, Q, B, U):
    X = np.dot(A, X) + np.dot(B, U)
    P = np.dot(A, np.dot(P, A.T)) + Q
    return(X,P)

acc_raw=readFile('accCalib.txt')
acc_error = sensorError(acc_raw)
sensor=removeSensorError([[0.01], [-0.01], [1.05]], acc_error)
print(sensor)
