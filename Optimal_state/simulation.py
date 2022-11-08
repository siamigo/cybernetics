from functions import *
import matplotlib.pyplot as plt

# All functions are in functions.py, as well as library imports

DEBUG = False # Print values and add a delay
delay = 0.25

ar = 9.2 / 2 # axle radius in mm
dt = 0.32

dRaw, vRaw, aRaw = readFile('Optimal_state\TestValues.txt') # Read the test values from a file
R = cal_covar(dRaw, vRaw, aRaw) # Calculate the covariance matrix
#R[0, 0] += 20.0
#R[0, 2] += 0.02
#R[2, 0] += 0.02
#R[2, 2] += 0.1

dRawQ, vRawQ, aRawQ = readFile('Optimal_state\QtestValues.txt') # Not used as measurements was done incorrectly
Q = cal_covar(dRawQ, vRawQ, aRawQ) # Tuned Q matrix manually

P_km1 = R # Initial process covariance

A_km1 = np.array([[1.0, dt, 0.5*(dt**2.0)], 
                  [0.0, 1.0, dt], 
                  [0.0, 0.0, 1.0]])

x_km1 = np.transpose(np.array([[0.0, 0.0, 0.0]])) # Initial state
x_kmes = x_km1 # Just for definition
 
H = np.identity(len(R))
B = 0.0
u = 0.0

r_d, r_v, r_a, e_d, e_v, e_a = [], [], [], [], [], []
# if DEBUG:
#     print("R: ")
#     print(R)
#     print("Q: ")
#     print(Q)

dis, vel, acc, est_dis, est_vel, est_acc, time = readFileComma('Optimal_state\kalman_data\kalman_data.csv')
for i, _ in enumerate(time):
        dt = time[i]

        x_kp = kalman_predict_x(A_km1, x_km1, B, u)
        P_kp = kalman_predict_P(A_km1, P_km1, Q)

        v = vel[i]
        a = acc[i]
        d = dis[i]

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

        val = np.concatenate((Yk[:,0], x_k[:,0]))
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