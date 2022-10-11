import numpy as np

dt = 0.1

x0 = np.transpose(np.array([[0, 0, 1]]))
P0 = np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]])

A = np.array([[1, dt, 0.5*(dt**2)], [0.0, 1., dt], [0.0, 0.0, 1]]) 
H = np.array([[1, 0, 1]])
R = np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]])
x = x0
P_k = P0

Y = np.dot(H, x)

def kalman_predict_x(Ad, x_km1, Bd, u_km1):
    x_kd = np.dot(Ad, x_km1) + np.dot(Bd, u_km1)
    return x_kd

def kalman_predict_P(Ad, P_km1):
    P_kd = np.dot(np.dot(Ad, P_km1), np.transpose(Ad))
    return P_kd

def kalman_gain(P_kd, Hd, Rd):
    num = np.dot(P_kd * np.transpose(Hd))
    den = np.linalg.inv(np.dot(np.dot(Hd, P_kd), np.transpose(Hd)) + Rd)
    Kk = np.dot(num, den)
    return Kk

def kalman_newstate(x_km1, Kk, zk, Hd):
    x_kd = x_km1 + np.dot(Kk, (zk - np.dot(Hd, x_km1)))
    return x_kd

def kalman_newerror(Kk, Hd, P_km1):
    I = np.identity(np.sHdape(Hd)[1])
    P_kd = np.dot((I - np.dot(Kk, Hd), P_km1))
    return P_kd

while True:
    x_kp = kalman_predict_x(A, x)
    P_kp = kalman_predict_P(A, P_k)

    K = kalman_gain(P_kp, H, R)
    x = kalman_newstate(x_kp, K, Y, H)
    P_k = kalman_newerror(K, H, P_kp)
    