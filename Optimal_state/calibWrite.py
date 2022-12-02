from funtions.functions import *

ar = 9.2 / 2
dt = .032
prevAngle = 0.0
prev_d = 0.0

while(True):
    sensor_values = arduino_send_receive(prevAngle)
    if(sensor_values is not None):
        v=(sensor_values[0]*np.pi/180.0 - prevAngle*np.pi/180.0) * ar / dt
        a=sensor_values[1]
        d=sensor_values[2]
        dt=sensor_values[3]
        stop=sensor_values[4]

        delta_d = d - prev_d

        prev_d = d
        prevAngle = sensor_values[0]

        if(stop==0.):
            arr=[delta_d, v, a, dt]
            write_csv(arr, 'calibQ_data1.csv')
    else:
        arduino_has_been_reset()