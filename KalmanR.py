import numpy as np

dValues = np.array([1.2, 1.3, 1.1, 1.2, 1.3])
vValues = np.array([2.2, 2.3, 2.1, 2.2, 2.3])
aValues = np.array([3.2, 5.3, 3.1, 3.2, 3.3])

def cal_covar(dList, vList, aList):
    d_var = np.var(dList)
    v_var = np.var(vList)
    a_var = np.var(aList)

    R = np.array([[d_var, d_var*v_var, d_var*a_var], 
                  [v_var*d_var, v_var, v_var*a_var], 
                  [a_var*d_var, a_var*v_var, a_var]])
    
    return R

x0 = np.transpose(np.array([[1, 2, 3]]))

"""
def kalman_gain(P_k, Hd, Rd):
    num = np.dot(P_k, np.transpose(Hd))
    den = np.dot(Hd, np.dot(P_k, np.transpose(Hd))) + Rd
    kk = np.zeros(np.shape(P_k))
    for i in range(len(P_k)): 
        for j in range(len(P_k)):
            if (num[i, j] or den[i, j]) == 0:
                kk[i, j] = 0
            else:
                kk[i, j] = num[i, j] / den[i, j]
    return kk"""
