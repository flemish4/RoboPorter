from sys import platform

import socket
import serial
import time
import numpy as np
import matplotlib.pyplot as plt
import math



if __name__ == '__main__':
    print("hi")
    count = 0
    first = 1
    finalVal = 235
    lidarStore = [None] * 237
    try: #try to connect
        #if (platform == "linux") or (platform == "linux2"):
        #    LIDARConn = serial.Serial('/dev/ttyACM0', 19200,timeout=5)
        #elif (platform == "win32"):
        LIDARConn = serial.Serial('COM5', 2000000)
    except Exception as e:
        print("oh no:" + str(e))
    print(LIDARConn)
    
    
        
    #while True:
    #    print LIDARConn.read()
    time.sleep(4)
    try:
        LIDARConn.write("4\n")
    except Exception as e:
        print("oh no2:" + str(e))

    while True :
        input = LIDARConn.readline()[:-2]
        if input == "$" :
            print("yay")
            count = 0
        else :
            angle = 360 * count / 237
            print(input)
            lidarStore[count] = float(input)
        print(input)    

    