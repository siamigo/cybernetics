from functions import *

# All functions are in functions.py, as well as library imports
delay = 0.25

ar = 9.2 / 2 # axle radius in mm
dt = 1.0
prevAngle = 0.0

estimate = 0.0
delta = 1.0
while(True):
    sensor_values = arduino_send_receive(prevAngle)
    if(sensor_values is not None):
        v=(sensor_values[0]*np.pi/180.0 - prevAngle*np.pi/180.0) * ar / dt
        a=sensor_values[1]
        d=sensor_values[2]
        dt=sensor_values[3]
        stop=sensor_values[4]

        prevAngle=sensor_values[0]

        if(stop==0.):
            arr=np.array[d, v, a, dt]
            write_csv(arr, 'calibQ_data.csv')

    else:
        arduino_has_been_reset()