import time
from socket import *
import numpy as np

from KalmanJor import *

udp_socket = socket(AF_INET, SOCK_DGRAM)
udp_socket.settimeout(1)

np.set_printoptions(suppress = True)

arduino_ip = '192.168.10.240'
arduino_port = 8888

estimate = 0.0
delta = 1.0

ar = 0.004 # axle radius
dt = 0
prevAngle = 0

dRaw, vRaw, aRaw = readFile('TestValues.txt')
R = cal_covar(dRaw, vRaw, aRaw)

x0 = np.transpose(np.array([[0, 0, 1]]))

A = np.array([[1, dt, 0.5*(dt**2)], [0.0, 1., dt], [0.0, 0.0, 1]]) 
H = np.array([[1, 1, 1]])
x = x0
P_k = R # Initially P matrix is same as R

while(True):
    estimate = estimate + delta
    if(estimate > 100.0):
        delta = -1
    elif(estimate < -100):
        delta = 1

    x_kp = kalman_predict_x(A, x)
    P_kp = kalman_predict_P(A, P_k)

    sensor_values = arduino_send_receive(estimate)
    if(sensor_values is not None):
        dt = sensor_values[3] * 10**(-3)
        v = round(((sensor_values[0] - prevAngle) * ar) / dt, 3)
        a = sensor_values[1]
        d = sensor_values[2]
        
        prevAngle = sensor_values[0]
    else:
        arduino_has_been_reset()

    K = kalman_gain(P_kp, H, R)
    x = kalman_newstate(x_kp, K, Y, H)
    P_k = kalman_newerror(K, H, P_kp)
  