from numpy import array, average, loadtxt, cov, var, pi

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

def getvelocity(ang, dt):
    return [(ang[i] - ang[i-1])/dt * 0.004 for i in range(1, len(ang))]
# Function taken from Implementation of Kalman Filter with Python Language  by Mohamed LAARAIEDH
def getCovMatrix(dist, vel ,acc):
    return [[var(dist), cov(dist,vel), cov(dist,acc)], [cov(vel,dist), var(vel), cov(vel,acc)], [cov(acc,dist), cov(acc,vel), var(acc)]]

v_raw=getvelocity(angle)
acc_raw = readFile('accCalib.txt')
acc_error=sensorError(acc_raw)
acc_corr = getTotalAcceleration(acc_raw)

print(acc_corr)