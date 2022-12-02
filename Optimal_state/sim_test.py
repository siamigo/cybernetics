from funtions.functions import *


# All functions are in functions.py, as well as library imports

def main():

#----------------------------------------------------------------Filehandling----------------------------------------------------------------
    dRaw, vRaw, aRaw, dtR = readFileComma('Optimal_state\calibrationdata\calibR_data.csv')
    dRawQ, vRawQ, aRawQ, dtR = readFileComma('Optimal_state\calibrationdata\calibQ_data.csv')
    dis, vel, acc, est_dis, est_vel, est_acc, time = readFileComma('Optimal_state\kalman_data\kalman_data1.csv')
#----------------------------------------------------------------Accelerometer gravity correction----------------------------------------------------------------
    accR, avrR = sensorError(aRaw)
    accQ, avrQ = sensorError(aRawQ)
#----------------------------------------------------------------Noize matrixes----------------------------------------------------------------
    R=np.array([[np.var(dRaw)],[np.var(vRaw)],[np.var(accR)]])
    Q=cal_covar(dRawQ, vRawQ,accQ)
#----------------------------------------------------------------Initial values----------------------------------------------------------------
    dt = .032
    prevAngle = 0.0

    P = cal_covar(dRaw, vRaw,accR)

    A = np.array([[1.0, dt, 0.5*(dt**2.0)], 
                      [0.0, 1.0, dt], 
                      [0.0, 0.0, 1.0]])

    x_km1 = np.transpose(np.array([[0.0, 0.0, 0.0]]))
    x = x_km1
    
    P_p=A*P*np.transpose(A)+Q
    H = [1, 1, 1]
    B = 0.0
    u = 0.0

    r_d, r_v, r_a, e_d, e_v, e_a = [], [], [], [], [], []

    #---------------------------------------------------------------- Kalman loop----------------------------------------------------------------
    for i, _ in enumerate(time):
            dt = time[i]

            v = vel[i]
            a = acc[i]
            d = dis[i]

            # x measurements matrix
            x[0] = d
            x[1] = v
            x[2] = a

            z = np.dot(H, x)

            X_p=A*x

            s=H*P_p*np.transpose(H)+R

            K = P*(np.transpose(H)*(1/s))

            X_k=X_p+K*(z-H*X_p)
            P_k = P_p-K*H*P_p

            print(s)
            A = np.array([[1.0, dt, 0.5*(dt**2.0)], 
                              [0.0, 1.0, dt], 
                              [0.0, 0.0, 1.0]])
            print(z, X_k)
            val = np.concatenate((z, X_k[:,0]))
            r_d.append(val[0])
            r_v.append(val[1])
            r_a.append(val[2])
            e_d.append(val[3])
            e_v.append(val[4])
            e_a.append(val[5])


    t_time = []
    for count, element in enumerate(time):
        if count == 0:
            t_time.append(element)
        else:
            t_time.append(element+t_time[count-1])

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