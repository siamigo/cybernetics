from statistics import variance
import numpy as np

# Should be able to make X-D amount of uknown data, and place it in the same dataset array.
def readFile(filename):
    data_raw = np.loadtxt(filename, delimiter=',', skiprows=0, dtype=float)
    data=[]
    for i in range(0, len(data_raw[0])):
        data.append(data_raw[:,i])
    return data
# Calculates error from the closes integer.
def sensorError(sensor_raw):
    sensor_error = []
    for i in range(0,len(sensor_raw)):
        avrage = np.average(sensor_raw[i])
        sensor_error.append(avrage-int(avrage))
    return sensor_error
# Gives total acceleration from raw data.
def getTotalAcceleration(acc_raw, acc_error):
    acc_corr=acc_raw-acc_error
    acc_x, acc_y, acc_z = acc_corr
    acc_tot = np.sqrt(acc_x*acc_x+acc_y*acc_y+acc_z*acc_z)
    return acc_tot
# Function taken from Implementation of Kalman Filter with Python Language  by Mohamed LAARAIEDH
def kalmanFilter(X, P, A, Q, B, U):
    X = np.dot(A, X) + np.dot(B, U)
    P = np.dot(A, np.dot(P, A.T)) + Q
    return(X,P)

