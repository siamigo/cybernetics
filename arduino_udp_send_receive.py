from KalmanJor import *

estimate = 0.0
delta = 1.0

ar = 0.004 # axle radius
dt = 0
prevAngle = 0

dRaw, vRaw, aRaw = readFile('TestValues.txt')
R = cal_covar(dRaw, vRaw, aRaw)
P_k = R # Initially P matrix is same as R

x0 = np.transpose(np.array([[0, 0, 1]]))
x_k = x0
x_km = x0

# A = np.array([[1, dt, 0.5*(dt**2)], [0.0, 1., dt], [0.0, 0.0, 1]]) 
H = np.identity(len(A))
B = 0
u = 0

while(True):
    estimate = estimate + delta
    if(estimate > 100.0):
        delta = -1
    elif(estimate < -100):
        delta = 1

    x_kp = kalman_predict_x(A, x_k, B, u)
    P_kp = kalman_predict_P(A, P_k)

    print("Predicted x matrix: ")
    print(x_kp)
    print("Predicted P matrix: ")
    print(P_kp)

    sensor_values = arduino_send_receive(estimate)
    if(sensor_values is not None):
        dt = sensor_values[3] * 10**(-3)
        A = np.array([[1.0, dt, 0.5*(dt**2.0)], [0.0, 1.0, dt], [0.0, 0.0, 1.0]])
        v = round((sensor_values[0]*np.pi/180 - prevAngle*np.pi/180) * ar / dt, 3)
        a = sensor_values[1]
        d = sensor_values[2]
        
        prevAngle = sensor_values[0]

        x_km[0] = d
        x_km[1] = v
        x_km[2] = a

        print("Measured distance: ")
        print(d)
        print("Measured velocity: ")
        print(v)
        print("Measured acceleration: ")
        print(a)

        Y = np.dot(H, x_km)

    else:
        arduino_has_been_reset()

    K = kalman_gain(P_kp, H, R)
    x_k = kalman_newstate(x_kp, K, Y, H)
    P_k = kalman_newerror(K, H, P_kp)
    
    print("Updated x matrix: ")
    print(x_k)
    print("Updated P matrix: ")
    print(P_k)

    t.sleep(1)