from numpy import *

dt = 0.1

A = array([[1, dt, 0.5*(dt**2)], [0.0, 1., dt], [0.0, 0.0, 1]]) 
H = array([1, 0, 1])

x0 = transpose(array([[0, 0, 0]]))

print(x0)