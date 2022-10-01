from statistics import variance, covariance

def readFile(filename):
    motorVoltage=[]
    encoderAngle=[]
    t=[]

    with open(filename, 'r') as dataFile:
        headerLine = dataFile.readline()
        headerLine = headerLine.strip("\n")
        for line in dataFile:
            line = line.strip("\n")
            lineList = line.split(" ")
            motorVoltage.append(float(lineList[1]))
            encoderAngle.append(float(lineList[2]))
            t.append(float(lineList[3])/1000.)
    return motorVoltage, encoderAngle, t

