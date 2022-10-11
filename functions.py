from numpy import average, loadtxt, var, dot, transpose, pi
# Should be able to make X-D amount of uknown data, and place it in the same dataset array.
def readFile(filename):
    data_raw = loadtxt(filename, delimiter=',', skiprows=0, dtype=float)
    return [data_raw[:,i] for i, _ in enumerate(data_raw[0])]
# Calculates error from the closes integer.
def sensorError(sensor_raw):
    return [average(sensor_raw[index])-int(average(sensor_raw[index])) for index, _ in enumerate(sensor_raw)]
# Gives total acceleration from raw data.
def getTotalAcceleration(acc_raw):
    acc_x, acc_y, acc_z = acc_raw
    return (acc_x*acc_x+acc_y*acc_y+acc_z*acc_z)**0.5
# Calculates the angular velocity from calibration data
def calVelocity(ang, dt):
    return [(ang[i] - ang[i-1])/dt * 0.004 for i in range(1, len(ang))]
# Give the covariance matrix
def getCovMatrix(dist, vel ,acc):
    dist_v, vel_v, acc_v = var(dist), var(vel), var(acc)
    return [[dist_v, dist_v*vel_v, dist_v*acc_v], [vel_v*dist_v, vel_v, vel_v*acc_v], [acc_v*dist_v, acc_v*vel_v, acc_v]]
#Gives the variance matrix
def getVarMatrix(dist, vel, acc):
    return[[var(dist)], [var(vel)],[var(acc)]]
# Function taken from Implementation of Kalman Filter with Python Language  by Mohamed LAARAIEDH
def predictions(X, P, A, Q):
    X = dot(A, X) #+ dot(B, U)
    P = dot(A, dot(P, transpose(A))) + Q
    return(X,P)

dis_calib, ang_raw, acc_raw=readFile('disAngCalib.txt')
acc_raw = readFile('accCalib.txt')
acc_calib = getTotalAcceleration(acc_raw)
vel_calib=calVelocity(ang_raw, 0.0031)

R = getCovMatrix(dis_calib, vel_calib, acc_calib)
x0=[[average(dis_calib)],[average(vel_calib)], [average(acc_calib)]]
H=[1,1,1]
