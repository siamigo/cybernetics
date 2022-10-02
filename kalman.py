from functions import *
import numpy as np

acc_x_cali, acc_y_cali, acc_z_cali = readFile('calibration_accelerometer.txt')
acc_x_error, acc_y_error, acc_z_error = errorAccelerometer(acc_x_cali, acc_y_cali, acc_z_cali)

gyr_x_cali, gyr_y_cali, gyr_z_cali = readFile('calibration_gyrometer.txt')
gyr_x_error, gyr_y_error, gyr_z_error = errorGyrometer(gyr_x_cali, gyr_y_cali, gyr_z_cali)

millisOld=0

#--------------------- Not actual values---------------------
acc_x_raw=0
acc_y_raw=0 
acc_z_raw=0
gyr_x_raw=0 
gyr_y_raw=0 
gyr_z_raw=0
millis=0
while True:
    acc_x=acc_x_raw-acc_x_error
    acc_y=acc_y_raw-acc_y_error
    acc_z=acc_z_raw-acc_z_error
    gyr_x=gyr_x_raw-gyr_x_error
    gyr_y=gyr_y_raw-gyr_y_error
    gyr_z=gyr_z_raw-gyr_z_error


    thetaM=np.arctan2(-acc_x/9.8, np.sqrt(acc_y*acc_y+acc_z*acc_z)/9.8)*(180./np.pi)
    phiM=-np.arctan2(acc_y/9.8, np.sqrt(acc_x*acc_x+acc_z*acc_z)/9.8)*(180./np.pi)
    
    dt=(millis-millisOld)/1000.
    millisOld=millis
    
    theta=(theta+gyr_y*dt)*0.95+thetaM*0.05
    phi=(phi-gyr_x*dt)*0.95+phiM*0.05

    
    
    
