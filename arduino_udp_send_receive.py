from KalmanJor import *
"""
udp_socket = socket(AF_INET, SOCK_DGRAM)
udp_socket.settimeout(1)

np.set_printoptions(suppress = True)

arduino_ip = '192.168.10.240'
arduino_port = 8888
"""
estimate = 0.0
delta = 1.0

ar = 0.004 # axle radius
dt = 0
prevAngle = 0

dRaw, vRaw, aRaw = readFile('TestValues.txt')
R = cal_covar(dRaw, vRaw, aRaw)
P_k = R # Initially P matrix is same as R

x0 = np.transpose(np.array([[0, 0, 1]]))
x_k = x0
x_km = x0

A = np.array([[1, dt, 0.5*(dt**2)], [0.0, 1., dt], [0.0, 0.0, 1]]) 
H = np.identity(np.shape(A)[0])
B = 0
u = 0

while(True):
    estimate = estimate + delta
    if(estimate > 100.0):
        delta = -1
    elif(estimate < -100):
        delta = 1

    x_kp = kalman_predict_x(A, x_k, B, u)
    P_kp = kalman_predict_P(A, P_k)

    sensor_values = arduino_send_receive(estimate)
    if(sensor_values is not None):
        dt = sensor_values[3] * 10**(-3)
        v = round(((sensor_values[0] - prevAngle) * ar) / dt, 3)
        a = sensor_values[1]
        d = sensor_values[2]
        
        prevAngle = sensor_values[0]

        x_km[0] = d
        x_km[1] = v
        x_km[2] = a

        Y = np.dot(H, x_km)

    else:
        arduino_has_been_reset()

    K = kalman_gain(P_kp, H, R)
    x_k = kalman_newstate(x_kp, K, Y, H)
    P_k = kalman_newerror(K, H, P_kp)
    
    print(x_k, P_k)

    """
    Note to self
    Elementvis divison av num og den hvor hvis en er 0 så er produktet 0
    """