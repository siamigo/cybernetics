# Writes the both the R- and Q matrices calibration data to a csv file.

from funtions.functions import *

def main():
    ar = 9.2 / 2
    dt = .032
    prevAngle = 0.0
    prev_d = 0.0

    firstPass = True

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
                if firstPass:
                    firstPass = False
                else:
                    arr=[delta_d, v, a, dt]
                    write_csv(arr, 'Optimal_state/calibrationdata/calibQ_data.csv')
        else:
            arduino_has_been_reset()

if __name__ == '__main__':
    main()