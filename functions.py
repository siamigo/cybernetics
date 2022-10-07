from numpy import array, average, loadtxt, cov, var, pi

# Should be able to make X-D amount of uknown data, and place it in the same dataset array.
def readFile(filename):
    data_raw = loadtxt(filename, delimiter=',', skiprows=0, dtype=float)
    return [data_raw[:,i] for i, _ in enumerate(data_raw[0])]
# Calculates error from the closes integer.
def sensorError(sensor_raw):
    return [average(sensor_raw[index])-int(average(sensor_raw[index])) for index, _ in enumerate(sensor_raw)]
# Gives total acceleration from raw data.
def getTotalAcceleration(acc_raw, acc_error):
    acc_x, acc_y, acc_z = acc_raw-acc_error
    return (acc_x*acc_x+acc_y*acc_y+acc_z*acc_z)**0.5

def getvelocity(puls, dt):
    return (0.008*pi*puls)/(1024*dt*10**-3)
# Function taken from Implementation of Kalman Filter with Python Language  by Mohamed LAARAIEDH
def kalmanFilter(X, P, A, Q, B, U):
    X = np.dot(A, X) + np.dot(B, U)
    P = np.dot(A, np.dot(P, A.T)) + Q
    return(X,P)


acc_raw = readFile('accCalib.txt')
acc_error=sensorError(acc_raw)
acc_corr = getTotalAcceleration(array([0.02,-0.01,1.06]), acc_error)
print(acc_error)