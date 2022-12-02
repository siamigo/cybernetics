from funtions.functions import *

# All functions are in functions.py, as well as library imports
def main():

    DEBUG = True # Print values and add a delay
    delay = 0.25
    ar = 9.2 / 2 # axle radius in mm
#----------------------------------------------------------------Filehandling----------------------------------------------------------------
    dRaw, vRaw, aRaw, dtR = readFileComma('Optimal_state\calibrationdata\calibR_data.csv') # Read the test values from a file
    dRawQ, vRawQ, aRawQ, dtQ = readFileComma('Optimal_state\calibrationdata\calibQ_data.csv')
#----------------------------------------------------------------Accelerometer gravity correction----------------------------------------------------------------
    accR, avrR = sensorError(aRaw)
    accQ, avrQ = sensorError(aRawQ)
    acc_error=(avrR+avrQ)/2
#----------------------------------------------------------------Noize matrixes----------------------------------------------------------------

    Q = cal_covar(dRawQ, vRawQ, accQ) #np.array([[3, 0., 0.3],[0.0,0.0,0.0], [0.3, 0.0, 0.1]]) # Tuned Q matrix manually
    R = cal_covar(dRaw, vRaw, accR) # Calculate the covariance matrix
    #R[0, 0] += 20.0
    #R[0, 2] += 0.02
    #R[2, 0] += 0.02
    #R[2, 2] += 0.1

    if DEBUG:
        print("R: ")
        print(R)
        print("Q: ")
        print(Q)
        print("acc_error: ")
        print(acc_error)
#----------------------------------------------------------------Initial values----------------------------------------------------------------
    dt = .032
    prevAngle = 0.0

    P_km1 = R # Initial error covariance

    A_km1 = np.array([[1.0, dt, 0.5*(dt**2.0)], 
                      [0.0, 1.0, dt], 
                      [0.0, 0.0, 1.0]])

    x_km1 = np.transpose(np.array([[0.0, 0.0, 0.0]])) # Initial state matrix
    x_kmes = x_km1 # Initial measurement matrix
    
    H = np.identity(len(x_kmes))
    B = 0.0
    u = 0.0

    while 1:
        sensor_values = arduino_send_receive(x_km1[0])
        if(sensor_values is not None):
            dt = sensor_values[3] * 10**(-3)

            x_kp = kalman_predict_x(A_km1, x_km1, B, u)
            P_kp = kalman_predict_P(A_km1, P_km1, Q)

            v = (sensor_values[0]*np.pi/180.0 - prevAngle*np.pi/180.0) * ar / dt # Calculate linear velocity from angular velocity, using encoder angle measured in degrees
            a = sensor_values[1]-acc_error  # Get acceleration measurement and gravity compensate
            d = sensor_values[2]
            stop = sensor_values[4]

            prevAngle = sensor_values[0]

            # x measurements matrix
            x_kmes[0] = d
            x_kmes[1] = v
            x_kmes[2] = a

            Yk = np.dot(H, x_kmes) # Senor fusion

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

            if (stop == 0.):
                arr = np.concatenate((Yk[:,0], x_kp[:,0], [dt]))
                write_csv(arr, 'kalman_data6.csv')
            #t.sleep(0.05)

        else:
            arduino_has_been_reset()

if __name__ == '__main__':
    main()