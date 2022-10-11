import numpy as np

def cal_covar(dList):
    var_d = np.zeros(len(dList))
    d_avg = np.average(dList)
    for i in range(len(dList)):
        var_d[i] = (dList[i] - d_avg)**2

    
    return var_d