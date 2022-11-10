#----------------------------------------------------------------All python libraries----------------------------------------------------------------
import time as t
from socket import *
import numpy as np
import csv
import matplotlib.pyplot as plt
import statistics as st
#----------------------------------------------------------------Communication----------------------------------------------------------------
udp_socket = socket(AF_INET, SOCK_DGRAM)
udp_socket.settimeout(1)

np.set_printoptions(suppress = True)

arduino_ip = '192.168.10.240'
arduino_port = 8888

def arduino_send_receive(estimate):
    udp_socket.sendto(str(estimate).strip('[]').encode(), (arduino_ip, arduino_port))
    try:
        inbound_message, remote_address = udp_socket.recvfrom(1024)
        # returns an array with the following values
        # [accel_x, accel_y, accel_z, range_sensor]
        return np.array(inbound_message.decode('ascii').split(',')).astype(float)
    except Exception as e:
        print(e)

def arduino_has_been_reset():
    print("Arduino is offline.. Resetting")

#---------------------------------------------------------------- Kalman----------------------------------------------------------------
def kalman_predict_x(Ad, x_m1, Bd, u_km1):
    x_kd = np.dot(Ad, x_m1) + np.dot(Bd, u_km1)
    return x_kd

def kalman_predict_P(Ad, P_m1, Qd):
    P_kd = np.dot(Ad, np.dot(P_m1, np.transpose(Ad))) + Qd
    return P_kd

def kalman_gain(P_k, Hd, Rd):
    num = np.dot(P_k, np.transpose(Hd))
    den = np.dot(Hd, np.dot(P_k, np.transpose(Hd))) + Rd
    kk = np.dot(num, np.linalg.inv(den))
    return kk

def kalman_newstate(x_m1, Kk, Yd, Hd):
    x_kd = x_m1 + np.dot(Kk, (Yd - np.dot(Hd, x_m1)))
    return x_kd

def kalman_newerror(Kk, Hd, P_k):
    I = np.identity(np.shape(Hd)[1])
    P_kd = np.dot((I - np.dot(Kk, Hd)), P_k)
    return P_kd

def cal_covar(dList, vList, aList):
    d_var = np.var(dList)
    v_var = np.var(vList)
    a_var = np.var(aList)

    Rd = np.array([[d_var, d_var*v_var, d_var*a_var], 
                   [v_var*d_var, v_var, v_var*a_var], 
                   [a_var*d_var, a_var*v_var, a_var]])
    
    return Rd
#----------------------------------------------------------------File handling----------------------------------------------------------------
def readFileComma(filename):
    data_raw = np.loadtxt(filename, delimiter=',', dtype=float)
    return [data_raw[:,i] for i, _ in enumerate(data_raw[0])]

def readFile(filename):
    data_raw = np.loadtxt(filename, delimiter=' ', skiprows=0, dtype=float)
    return [data_raw[:,i] for i, _ in enumerate(data_raw[0])]

def write_csv(data , filename):
    with open(filename, 'a', encoding='UTF8', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(data)
    f.close()

#----------------------------------------------------------------Gravity correction----------------------------------------------------------------
def sensorError(sensor_raw):
    avr = np.average(sensor_raw)
    return np.array([np.average(value)-avr for index, value in enumerate(sensor_raw)]), avr