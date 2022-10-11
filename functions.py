from numpy import average, loadtxt, var, dot, transpose, zeros, shape, array, around, subtract, pi
# Should be able to make X-D amount of uknown data, and place it in the same dataset array.
def readFile(filename):
    data_raw = loadtxt(filename, delimiter=',', skiprows=0, dtype=float)
    return array([data_raw[:,i] for i, _ in enumerate(data_raw[0])])
# Calculates error from the closes integer.
def sensorError(sensor_raw):
    return array([average(sensor_raw[index])-0 for index, _ in enumerate(sensor_raw)])
# Gives total acceleration from raw data.
def getTotalAcceleration(acc_raw):
    acc_x, acc_y, acc_z = acc_raw
    return acc_x*acc_x+acc_y*acc_y+acc_z*acc_z**(0.5) #subtract((acc_x*acc_x+acc_y*acc_y+acc_z*acc_z)**(0.5),sum(acc_error))
# Calculates the angular velocity from calibration data
def calVelocity(ang, dt):
    return array([(ang[i] - ang[i-1])/dt * 0.004 for i in range(1, len(ang))])
# Give the covariance matrix
def getCovMatrix(dist, vel ,acc):
    dist_v, vel_v, acc_v = var(dist), var(vel), var(acc)
    return array([[dist_v, dist_v*vel_v, dist_v*acc_v], [vel_v*dist_v, vel_v, vel_v*acc_v], [acc_v*dist_v, acc_v*vel_v, acc_v]])
#Gives the variance matrix
def getVarMatrix(dist, vel, acc):
    return array([[var(dist)], [var(vel)],[var(acc)]])
# Function taken from Implementation of Kalman Filter with Python Language  by Mohamed LAARAIEDH
def predictions(X, P, A, Q):
    X = dot(A, X) #+ dot(B, U)
    P = dot(A, dot(P, transpose(A))) + Q
    return(X,P)

def kalman_gain(P_kd, Hd, Rd):
    num = dot(P_kd, transpose(Hd))
    den = dot(dot(Hd, P_kd), transpose(Hd)) + Rd
    kk = zeros(shape(P_kd))
    for i in range(len(P_kd)):
        for j in range(len(P_kd)):
            if (num[i, j] or den[i, j]) == 0:
                kk[i, j] = 0
            else:
                kk[i, j] = num[i, j] / den[i, j]
    return kk