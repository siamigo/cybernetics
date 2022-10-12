from KalmanJor import *

DEBUG = True # Print values and add a delay
delay = 0.25
estimate = 0.0
delta = 1.0

ar = 0.4 # axle radius in cm
dt = 0.0
prevAngle = 0.0

dRaw, vRaw, aRaw = readFile('TestValues.txt')
R = cal_covar(dRaw, vRaw, aRaw)
"""P_km1 = np.array([[300, 0, 0],
                  [0, 11.85, 0], 
                  [0, 0, 0.24]]) # Initial process covariance"""
P_km1 = R # Initial process covariance

x_km1 = np.transpose(np.array([[0.0, 0.0, 0.0]])) # Initial state
x_kmes = x_km1
 
H = np.identity(len(R))
B = 0.0
u = 0.0

while(True):
    estimate = estimate + delta
    if(estimate > 100.0):
        delta = -1
    elif(estimate < -100):
        delta = 1

    sensor_values = arduino_send_receive(estimate)
    if(sensor_values is not None):
        dt = sensor_values[3] * 10**(-3)

        A = np.array([[1.0, dt, 0.5*(dt**2.0)], 
                      [0.0, 1.0, dt], 
                      [0.0, 0.0, 1.0]])

        v = round( (sensor_values[0]*np.pi/180.0 - prevAngle*np.pi/180.0) * ar / dt, 3)
        a = sensor_values[1] - 9.81
        d = sensor_values[2]
        
        prevAngle = sensor_values[0]

        # x measurements
        x_kmes[0] = d
        x_kmes[1] = v
        x_kmes[2] = a

        Yk = np.dot(H, x_kmes)    

    else:
        arduino_has_been_reset()

    x_kp = kalman_predict_x(A, x_km1, B, u)
    P_kp = kalman_predict_P(A, P_km1)

    K = kalman_gain(P_kp, H, R)
    
    x_k = kalman_newstate(x_kp, K, Yk, H)
    P_k = kalman_newerror(K, H, P_kp)

    if DEBUG:
        print("Yk: ")
        print(Yk)
        print("Predicted x matrix: ")
        print(x_kp)
        print("Predicted P matrix: ")
        print(P_kp)
        print("Kalman gain: ")
        print(K)
        print("Updated x matrix: ")
        print(x_k)
        print("Updated P matrix: ")
        print(P_k)
        t.sleep(delay)

    x_km1 = x_k
    P_km1 = P_k
