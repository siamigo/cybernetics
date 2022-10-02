def readFile(filename):
    data_1=[]
    data_2=[]
    data_3=[]
    with open(filename, 'r') as dataFile:
        headerLine = dataFile.readline()
        headerLine = headerLine.strip("\n")
        for line in dataFile:
            line = line.strip("\n")
            lineList = line.split(" ")
            data_1.append(float(lineList[0]))
            data_2.append(float(lineList[1]))
            data_3.append(float(lineList[2]))
    return data_1, data_2, data_3

def getDt(time):
    dt = [0]
    for i in range(1, len(time)):
        dt.append(float(time[i]-time[i-1]))
    return dt

def errorAccelerometer(acc_x, acc_y, acc_z):
    c=0
    while c<len(acc_x):
        acc_x_error+=(acc_x/16384)
        acc_y_error+=(acc_y/16384)
        acc_z_error+=(acc_z/16384)
        c+=1
    acc_x_error=acc_x_error/len(acc_x)
    acc_y_error=acc_y_error/len(acc_x)
    acc_z_error=acc_z_error/len(acc_x)
    return acc_x_error, acc_y_error, acc_z_error

def errorGyrometer(gyr_x, gyr_y, gyr_z):
    c=0
    while c<len(gyr_x):
        gyr_x_error+=(gyr_x/131)
        gyr_y_error+=(gyr_y/131)
        gyr_z_error+=(gyr_z/131)
        c+=1
    gyr_x_error=gyr_x_error/len(gyr_x)
    gyr_y_error=gyr_y_error/len(gyr_x)
    gyr_z_error=gyr_z_error/len(gyr_x)
    return gyr_x_error, gyr_y_error, gyr_z_error