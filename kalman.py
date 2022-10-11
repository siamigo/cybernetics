from numpy import array
from functions import *

dis_calib, ang_raw, acc_raw=readFile('disAngCalib.txt')
acc_raw = readFile('accCalib.txt')
acc_calib = getTotalAcceleration(acc_raw)
vel_calib=calVelocity(ang_raw, 0.0031)

dt=0.032
R = getCovMatrix(dis_calib, vel_calib, acc_calib)
x0=[[average(dis_calib)],[average(vel_calib)], [average(acc_calib)]]
H=[1,1,1]
A = [[1, dt, 0.5*(dt**2)], [0.0, 1., dt], [0.0, 0.0, 1]] 
Q = array([[(dt**6)/36., (dt**5)/12., (dt**4)/6.], [(dt**5)/12., (dt**4)/6., (dt**3)/2.], [(dt**4)/6., (dt**3)/2., dt**2]])*array(getVarMatrix(dis_calib, vel_calib, acc_calib))
X,P =predictions(x0, R, A, Q)

print(Q)
print(f"X={X},P={P}")