import time
from socket import *
import numpy as np

udp_socket = socket(AF_INET, SOCK_DGRAM)
udp_socket.settimeout(1)

np.set_printoptions(suppress = True)

arduino_ip = '192.168.10.240'
arduino_port = 8888

dt = 0.1

x0 = np.transpose(np.array([[0, 0, 1]]))
P0 = np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]])

A = np.array([[1, dt, 0.5*(dt**2)], [0.0, 1., dt], [0.0, 0.0, 1]]) 
H = np.array([[1, 0, 1]])
R = np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]])
x = x0
P_k = P0

Y = np.dot(H, x)

def arduino_send_receive(estimate):
    udp_socket.sendto(str(estimate).encode(), (arduino_ip, arduino_port))
    try:
        inbound_message, remote_address = udp_socket.recvfrom(1024)
        # returns an array with the following values
        # [accel_x, accel_y, accel_z, range_sensor]
        return np.array(inbound_message.decode('ascii').split(',')).astype(float)
    except Exception as e:
        print(e)


def conv_to_speed(sensorV, r, deltaT):
    w = sensorV[0] / deltaT
    v = w * r
    a = sensorV[1]
    d = sensorV[2]
    return d, v, a

def arduino_has_been_reset():
    print("Arduino is offline.. Resetting")

def readFile(filename):
    data_raw = np.loadtxt(filename, delimiter=' ', skiprows=0, dtype=float)
    return [data_raw[:,i] for i, _ in enumerate(data_raw[0])]

def kalman_predict_x(Ad, x_km1, Bd, u_km1):
    x_kd = np.dot(Ad, x_km1) + np.dot(Bd, u_km1)
    return x_kd

def kalman_predict_P(Ad, P_km1):
    P_kd = np.dot(np.dot(Ad, P_km1), np.transpose(Ad))
    return P_kd

def kalman_gain(P_kd, Hd, Rd):
    num = np.dot(P_kd, np.transpose(Hd))
    den = np.dot(np.dot(Hd, P_kd), np.transpose(Hd)) + Rd
    kk = np.zeros(np.shape(P_kd))
    for i in range(len(P_kd)):
        for j in range(len(P_kd)):
            if (num[i, j] or den[i, j]) == 0:
                kk[i, j] = 0
            else:
                kk[i, j] = num[i, j] / den[i, j]
    return kk

def kalman_newstate(x_km1, Kk, zk, Hd):
    x_kd = x_km1 + np.dot(Kk, (zk - np.dot(Hd, x_km1)))
    return x_kd

def kalman_newerror(Kk, Hd, P_km1):
    I = np.identity(np.shape(Hd)[1])
    P_kd = np.dot((I - np.dot(Kk, Hd)), P_km1)
    return P_kd

def cal_covar(dList, vList, aList):
    d_var = np.var(dList)
    v_var = np.var(vList)
    a_var = np.var(aList)

    Rd = np.array([[d_var, d_var*v_var, d_var*a_var], 
                   [v_var*d_var, v_var, v_var*a_var], 
                   [a_var*d_var, a_var*v_var, a_var]])
    
    return Rd
