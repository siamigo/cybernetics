from KalmanJor import *

DEBUG = False # Print values and add a delay
delay = 0.25

ar = 0.4 # axle radius in cm
dt = 0.1
prevAngle = 0.0

dRaw, vRaw, aRaw = readFile('TestValues.txt')
R = cal_covar(dRaw, vRaw, aRaw)
R[0, 0] += 20.0
R[0, 2] += 0.02
R[2, 0] += 0.02
R[2, 2] += 0.1

dRawQ, vRawQ, aRawQ = readFile('QtestValues.txt')
Q = np.array([[1, 0., 0.01],[0.0,0.0,0.0], [0.01, 0.0, 0.1]])

P_km1 = R # Initial process covariance

A_km1 = np.array([[1.0, dt, 0.5*(dt**2.0)], 
                  [0.0, 1.0, dt], 
                  [0.0, 0.0, 1.0]])

x_km1 = np.transpose(np.array([[0.0, 0.0, 0.0]])) # Initial state
x_kmes = x_km1
 
H = np.identity(len(R))
B = 0.0
u = 0.0

if DEBUG:
    print("R: ")
    print(R)
    print("Q: ")
    print(Q)

while 1:
    sensor_values = arduino_send_receive(x_km1[0])
    if(sensor_values is not None):
        dt = sensor_values[3] * 10**(-3)

        G = np.array([[dt**(3)/6],[dt**(2)/2],[dt]])
        Q = G*np.transpose(G)*R

        x_kp = kalman_predict_x(A_km1, x_km1, B, u)
        P_kp = kalman_predict_P(A_km1, P_km1, Q)

        v = round( (sensor_values[0]*np.pi/180.0 - prevAngle*np.pi/180.0) * ar / dt, 3)
        a = sensor_values[1] - 9.81
        d = sensor_values[2]
        
        prevAngle = sensor_values[0]

        # x measurements
        x_kmes[0] = d
        x_kmes[1] = v
        x_kmes[2] = a

        Yk = np.dot(H, x_kmes) 

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

        A_km1 = np.array([[1.0, dt, 0.5*(dt**2.0)], 
                          [0.0, 1.0, dt], 
                          [0.0, 0.0, 1.0]])

        x_km1 = x_k
        P_km1 = P_k   

    else:
        arduino_has_been_reset()
