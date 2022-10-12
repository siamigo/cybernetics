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

print(x0[0])
x0[0] = 10
print(x0[0])
