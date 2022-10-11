import time
from socket import *
import numpy as np

udp_socket = socket(AF_INET, SOCK_DGRAM)
udp_socket.settimeout(1)

np.set_printoptions(suppress = True)

arduino_ip = '192.168.10.240'
arduino_port = 8888

ar = 0.004 # axle radius
dt = 0

def arduino_send_receive(estimate):
    udp_socket.sendto(str(estimate).encode(), (arduino_ip, arduino_port))
    try:
        inbound_message, remote_address = udp_socket.recvfrom(24)
        # returns an array with the following values
        # [accel_x, accel_y, accel_z, range_sensor]
        return np.array(inbound_message.decode('ascii').split(',')).astype(float)
    except Exception as e:
        print(e)


def import_sensor_values(sensorV, r, deltaT):
    w = sensorV[0] / deltaT
    v = w * r
    a = sensorV[1]
    d = sensorV[2]
    return d, v, a

def arduino_has_been_reset():
    print("Arduino is offline.. Resetting")


estimate = 0.0
delta = 1.0
while(True):
    dt -= sensor_values[3]
    estimate = estimate + delta
    if(estimate > 100.0):
        delta = -1
    elif(estimate < -100):
        delta = 1

    sensor_values = arduino_send_receive(estimate)
    if(sensor_values is not None):
        import_sensor_values(sensor_values)
    else:
        arduino_has_been_reset()
