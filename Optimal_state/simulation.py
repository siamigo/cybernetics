from funtions.functions import *

def main():

#------------------------------------ Filehandling ------------------------------------
    dRaw, vRaw, aRaw, _ = readFileComma('Optimal_state\calibrationdata\calibR_data.csv')
    dRawQ, vRawQ, aRawQ, _ = readFileComma('Optimal_state\calibrationdata\calibQ_data.csv')
    dis, vel, acc, _, _, _, time = readFileComma('Optimal_state\kalman_data\kalman_data.csv')
#------------------------------------ Accelerometer gravity correction ------------------------------------
    accR, _ = sensorError(aRaw)
    accQ, _ = sensorError(aRawQ)
#------------------------------------Noize matrixes-------------------------------------
    Q = cal_covar(dRawQ, vRawQ, accQ)
    R = cal_covar(dRaw, vRaw, accR)
#------------------------------------Initial values ------------------------------------
    dt = .032

    P_km1 = R 

    A_km1 = np.array([[1.0, dt, 0.5*(dt**2.0)], 
                      [0.0, 1.0, dt], 
                      [0.0, 0.0, 1.0]])

    x_km1 = np.transpose(np.array([[0.0, 0.0, 0.0]]))
    x_kmes = x_km1
    
    H = np.identity(len(R))
    B = 0.0
    u = 0.0

    r_d, r_v, r_a, e_d, e_v, e_a = [], [], [], [], [], []
   
#------------------------------------ Kalman loop ------------------------------------
    for i, _ in enumerate(time):
            dt = time[i]

            x_kp = kalman_predict_x(A_km1, x_km1, B, u)
            P_kp = kalman_predict_P(A_km1, P_km1, Q)

            v = vel[i]
            a = acc[i]
            d = dis[i]

            x_kmes[0] = d
            x_kmes[1] = v
            x_kmes[2] = a

            Yk = np.dot(H, x_kmes)

            K = kalman_gain(P_kp, H, R)

            x_k = kalman_newstate(x_kp, K, Yk, H)
            P_k = kalman_newerror(K, H, P_kp)

            A_km1 = np.array([[1.0, dt, 0.5*(dt**2.0)], 
                              [0.0, 1.0, dt], 
                              [0.0, 0.0, 1.0]])

            x_km1 = x_k
            P_km1 = P_k

            val = np.concatenate((Yk[:,0], x_k[:,0]))
            r_d.append(val[0])
            r_v.append(val[1])
            r_a.append(val[2])
            e_d.append(val[3])
            e_v.append(val[4])
            e_a.append(val[5])

#------------------------------------ Plotting ------------------------------------
    t_time = plotTime(time)

    plt.close(1); plt.figure(1, figsize=(8, 6))

    plt.subplot(311)
    plt.plot(t_time, r_d, label='$rd$', color='orange')
    plt.plot(t_time, e_d, label='$ed$', color='blue')
    plt.ylabel(r'Distance [mm]')
    plt.xlabel(r'Time[s]')
    plt.title('Sensor estimate and real values')
    plt.legend()

    plt.subplot(312)
    plt.plot(t_time, r_v, label='$rv$', color='orange')
    plt.plot(t_time, e_v, label='$ev$', color='blue')
    plt.ylabel(r'Velocity [mm/s]')
    plt.xlabel(r'Time[s]')
    plt.legend()

    plt.subplot(313)
    plt.plot(t_time, r_a, label='$ra$', color='orange')
    plt.plot(t_time, e_a, label='$ea$', color='blue')
    plt.ylabel(r'Acceleration $[m/s^2]$')
    plt.xlabel(r'Time[s]')
    plt.legend()

    plt.show()

if __name__ == '__main__':
    main()