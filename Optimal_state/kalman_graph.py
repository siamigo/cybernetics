# Plots the sensor data and kalman-filtered data from a csv file.

from funtions.functions import *

def main():
    r_d, r_v, r_a, e_d, e_v, e_a, dt = readFileComma('Optimal_state\kalman_data\kalman_data3.csv')

    time = plotTime(time)

    plt.close(1); plt.figure(1, figsize=(8, 6))

    plt.subplot(311)
    plt.plot(time, r_d, label='$rd$', color='orange')
    plt.plot(time, e_d, label='$ed$', color='blue')
    plt.ylabel(r'Distance [mm]')
    plt.xlabel(r'Time[s]')
    plt.title('Sensor estimate and real values')
    plt.legend()

    plt.subplot(312)
    plt.plot(time, r_v, label='$rv$', color='orange')
    plt.plot(time, e_v, label='$ev$', color='blue')
    plt.ylabel(r'Velocity [mm/s]')
    plt.xlabel(r'Time[s]')
    plt.legend()

    plt.subplot(313)
    plt.plot(time, r_a, label='$ra$', color='orange')
    plt.plot(time, e_a, label='$ea$', color='blue')
    plt.ylabel(r'Acceleration $[m/s^2]$')
    plt.xlabel(r'Time[s]')
    plt.legend()

    plt.show()

if __name__ == '__main__':
    main()
