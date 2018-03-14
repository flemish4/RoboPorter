#
# Module name - R4_server
# Module Description -

# Author     - C. Samarakoon
# Created    - 21/03/2017
# Modified   - 01/04/2017
#
###---Imports-------------------

from bresenham import bresenham
import MySQLdb 
import Queue
import base64
import copy
import cv2
import glob
import json
import math
import numpy as np
import pyttsx
import random 
import serial
import socket
import struct
import sys
import threading
import time
from breezyslam.algorithms import RMHC_SLAM
from roboPorterTestFunctions import roboPorterLaser

from sys import platform
#if on linux, import functions for IMU and ip
if platform == "linux" or platform == "linux2":
    import fcntl  # linux specific (keep note)
    import sys, getopt

    sys.path.append('.')
    import os.path

else:
    try :
        from msvcrt import getch
    except Exception as e:
        print str(e)
# -logging config
import logging
import logging.handlers
#the mask for data logging
logging.basicConfig( format='%(asctime)s - (%(threadName)s) %(levelname)s: %(message)s', datefmt='%m/%d/%Y %I:%M:%S %p',
                    level=logging.INFO) #change the logging level here to change the display verbosity

                    
###---Global Variables-------------

##--Setup

US_Enable = True
Motor_Enable = True
Speech_Enable = True
Debug_Enable = True
Lidar_Enable = True
SLAM_Enable = True

##--Motor Commands
global speedVector #demanded wheel speeds (left, right)

speedVector = [0, 0]

##-- MYSQL DB variables
global connection
global cursor

##--Serial Connections
global MotorConn #serial handle for the motor controller
global USConn1 #serial handle for the Ultrasonic controller 1
global USConn2 #serial handle for the Ultrasonic controller 2
global LIDARConn

##--Safety
global safetyOn #Boolean for the state of the safety toggle (used only in manual control)
safetyOn = True

##--US data
global USAvgDistances #vector holding the average US distances
global obstruction #Boolean for comminicating whether there is an obstruction in the direction of travel
global UShosts
global stoppingDistance
global maxSpeeds
global usCaution
global sysRunning
global SLAMMaxSpeed
SLAMMaxSpeed = 1000000000 # REVISIT: Set max SLAM speed
USAvgDistances = [0, 0, 0, 0, 0, 0, 0, 0]
obstruction = False
usCaution = False
USThresholds = [30, 20, 30] #threasholds for treating objects as obstacles [front,side,back]
#make sure stopping distance is > usThresholds
stoppingDistance = 15
UShosts = 2
maxSpeeds = [0, 0, 0, 0, 0, 0, 0, 0]

##--Threading
global threadLock #lock to be used when changing global variables
threadLock = threading.Lock()
threads = [] #Array holding information on the currently running threads

# Command Queue

global commandqueue 
commandqueue = Queue.Queue()

global speedsqueue 
speedsqueue = Queue.Queue()

# -Porter Localisation
global porterLocation #vector holding global location [x,y] in cm
global porterOrientation #angle from north (between -180,180) in degrees
porterOrientation = 0  # from north heading (in degrees?)
porterLocation = [0,0]

##-System State Variables
sysRunning = False #
cmdExpecting = False #
exitFlag = False # system shutdown command
global system_status
system_status = "AwaitingCommands"
global motordata
motordata = 0 

enduserloop = False

global leftpulse
global rightpulse
global odomLeftPulse
global odomRightPulse
global LIDARLeftPulse
global LIDARRightPulse
leftpulse = 0
rightpulse = 0
odomLeftPulse = 0
odomRightPulse = 0
LIDARLeftPulse = 0
LIDARRightPulse = 0

# odometry
global pulseDistConst
# distance = number of pulses * ( diameter of wheels / number of divisions in encoder)
pulseDistConst = 785 / 500 # REVISIT : this is a complete guess

# Lidar
global numSamples
global lidarStartTime
global lidarEndTime
numSamples = 237
lidarStartTime = 0
lidarEndTime = 0

# Slam
global runSLAM
runSLAM = ""

global lidarMap
global realPorterSize
global realPorterRadius
global navMap
global totalR
global totalWd
global encoderVectorAbs
global lidarPolarList
global adjustedLidarStore
global adjustedLidarList
global adjustedLidarListReady
global SLAMObject
global SLAMRunning
global SLAMOffset
global SLAMSimOffset
global pathMap
global MAP_SIZE_METERS
global MAP_SIZE_PIXELS        
global globalWd
global dxy
global angle_delta
global detailedMap
global pixelPorterSize
global loadMap
global speedVectorMaxSpeed
speedVectorMaxSpeed = 0
angle_delta = 1
globalWd = 0
dxy = 0
MAP_SIZE_METERS          = 25 # 32
MAP_SIZE_PIXELS          = MAP_SIZE_METERS * 50 #800
SLAMOffset = (1250,1250)
SLAMSimOffset = (0,0)
SLAMRunning = False
adjustedLidarStore = []
adjustedLidarList = []
adjustedLidarListReady = False
encoderVectorAbs = (0,0)
lidarPolarList = []
navMap = np.zeros(SLAMOffset)
detailedMap = np.zeros(SLAMOffset)
loadMap = np.zeros(SLAMOffset)
totalR          = 0
totalWd          = 0
pathMap             = {}
lidarRun            = ""
realPorterSize      = (50,50)
realPorterRadius    = math.sqrt(realPorterSize[0]**2+realPorterSize[1]**2)
realPorterWheelOffsetY = -8 # REVISIT : Make this accurate
realPorterLIDAROffsetY = 26
pixelPorterSize    = (realPorterSize[0]/2,realPorterSize[1]/2)


#lastSent = [0, 0]

def setExitFlag(status):
    global exitFlag
    global threadLock
    with threadLock :
        exitFlag = status

        
# Functions
def constrainAngle360(angle, max, min) : # Library export
    while angle >= max :
        angle -= 360
    while angle < min :
        angle += 360
    return angle
  
        
def constrainFloat(d, max, min) :
    if d >= max :
        return max
    if d < min :
        return min
    return d


def roundBase(x, base):
    return int(base * round(float(x)/base))

        
# Returns length of line given in format [x0, y0, x1, y1]
def getLength(line) :
    return math.sqrt((line[2]-line[0])**2+(line[3]-line[1])**2)   
    
    
# Returns length^2 of line given in format [x0, y0, x1, y1]
def getLengthSquared(line) : 
    return (line[2]-line[0])**2+(line[3]-line[1])**2
        
        
        
        
        
        
        
        
        
        
        
        
###--- Thread defininitions

class MultiThreadBase(threading.Thread): #Parent class for threading
    def __init__(self, threadID, name): #Class constructor
        threading.Thread.__init__(self) #Initiate thread
        self.threadID = threadID #Thread number
        self.name = name #Readable thread name


class debugThread(MultiThreadBase):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.loopsdone = 0
        self.debugServer = False
        self.debugClient = False
        self.dataStore = ""

    def run(self):
        
        logging.info("Starting %s", self.name)
        while not exitFlag:
            debuginfo = 0 
            if not self.debugServer:
                self.runServer()
            if self.debugServer and not self.debugClient:
                self.waitForClient()
            if self.debugClient:
                try:
                    debuginfo = {
                        'Type':'DebugData', # Data Type
                        'US1': str(USAvgDistances[0]) , # Ultrasonic Distances
                        'US2': str(USAvgDistances[1]) ,
                        'US3': str(USAvgDistances[2]) ,
                        'US4': str(USAvgDistances[3]) ,
                        'US5': str(USAvgDistances[4]) ,
                        'US6': str(USAvgDistances[5]) ,
                        'US7': str(USAvgDistances[6]) ,
                        'US8': str(USAvgDistances[7]) ,                       
                        'Speed Vector Left':str(speedVector[0]) , # Left Speed Vector
                        'Speed Vector Right':str(speedVector[1]), # Right Speed Vector
                        'Obstruction':str(obstruction), # Obstruction 
                        'Safety ON':str(safetyOn), # Is the Safety On
                        'Debug Data Sent':str(self.loopsdone), # is a way of visually checking the debug is still updating. Will increment 1 each time an update is sent 
                        'System Status': str(system_status),
                        'Battery': str("50"),
                        'Left Pulses' : str(leftpulse),
                        'Right Pulses' : str(rightpulse),
                        'Location' : str(porterLocation),
                        'Max Speeds': str(maxSpeeds)
                    }
                except Exception as e:
                    logging.warning("%s", str(e))
                    
                try:
                    datatosend = json.dumps(debuginfo)
                    self.clientConnection.send(datatosend)
                    try:
                        #print str(detailedMap)
                        self.send_map(detailedMap)
                    except Exception as e:
                        logging.error("%s", str(e))
                    self.loopsdone += 1 #increment loops done by 1
                    time.sleep(0.5)

                except Exception as e:
                    logging.error("%s", str(e))
                    self.debugClient = False
                    # REVISIT : individual thread states - andrew question?
                    # Program log? for debuging?

    def waitForClient(self):
        # for each connection received create a tunnel to the client
        try:
            logging.debug("Ready for a new client to connect...(5s Timeout)")
            self.clientConnection, self.clientAddress = self.SeverSocket.accept()
            logging.info("Connected by %s", self.clientAddress)
            # send welcome message

            logging.debug("Sending welcome message...")
            self.clientConnection.send('S0')
            logging.debug("Getting data ACK from Client")
            self.dataStore = self.clientConnection.recv(1024)
            if self.dataStore == "S1":
                logging.debug("Client Connection Validated")
                print "Connection made"
                self.debugClient = True
            else:
                logging.warning("Client Connection FAILED :( Try again...")
                self.clientConnection.close()
                self.debugClient = False
        except Exception as e:
            logging.error("%s", str(e))

    def runServer(self):
        logging.info("Setting up sockets...")
        try:
            if (platform == "linux") or (platform == "linux2"):
                HOST = ''  # socket.gethostbyname(socket.gethostname()) #socket.gethostname()
            else:
                HOST = ''
            PORT = 5003

            # create a socket to establish a server
            logging.debug("Binding the socket...")
            
            self.SeverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.SeverSocket.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
            self.SeverSocket.bind((HOST, PORT))
            self.SeverSocket.settimeout(5)

            # listen to incoming connections on PORT
            logging.info("Socket opened at %s listening to port %s", HOST, PORT)
            self.SeverSocket.listen(1)
            self.debugServer = True
        except Exception as e:
            logging.error("%s", str(e))
            self.debugServer = False
            logging.debug("Sleeping...")
            time.sleep(5)
    
    def send_map(self,map_to_send):
        cv2.imwrite('map.jpg', map_to_send)
        img_string = cv2.imencode('.jpg',map_to_send)[1].tostring()
        img_string = base64.b64encode(img_string)
        query = "INSERT INTO temp (data,type) VALUES ('%s','image')" %img_string
        cursor.execute(query)

            
class motorDataThread(MultiThreadBase):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.hexData = True
        self.inputBuf = ""
     
    def checkMotorConn(self) :
        if MotorConn.closed:
            try:
                logging.info("Trying to open serial port")
                MotorConn.open()
                MotorConn.flushInput()
            except Exception as e:
                logging.error("%s", str(e))
            finally:
                logging.info("No Motor Comms... Looping back to listening mode")

    def run(self):
        global speedVector
        global threadLock
        global USConn1
        global obstruction
        global system_status
        global motordata
        global motortest
        global lastSent
        lastSent = [0,0]
        global last_time_sent
        last_time_sent = 0
        global text_file
        logging.info("Starting %s", self.name)
        global leftpulse
        global rightpulse
        global odomLeftPulse
        global odomRightPulse
        global LIDARLeftPulse
        global LIDARRightPulse

   
        # Loop until told to stop
        while not exitFlag:
            # Check that motor arduino connection is good
            self.checkMotorConn()
            
            # If the ultrasonic sensors have detected an obstruction
            if obstruction:
                if safetyOn: #if the safety is on
                    if lastSent != [0, 0]:
                        logging.debug("Setting Speed Vector")
                        with threadLock:
                            speedVector = [0, 0]
                elif lastSent != speedVector: #otherwise...
                    logging.warning("Obstacle Detected But Safety OFF...") #give a warning, but dont do anything to stop
                
                try:
                    self.send_serial_data(speedVector)
                except Exception as e:
                    self.checkMotorConn()
                    
            elif (speedVector != lastSent) or ((time.time() - last_time_sent)>=1):
                last_time_sent = 0 
                print "Sending Data"
                logging.debug("Data Ready")
                logging.info("Data Ready")
                try:
                    if safetyOn:
                        logging.info("Trying to send data")
                    else:
                        logging.info("Trying to send data no safety")
                    
                    try:
                        self.send_serial_data(speedVector)
                    except Exception as e:
                        self.checkMotorConn()
                        
                except Exception as e:
                    logging.error("%s", str(e))
            
            # Read from arduino
            if MotorConn.inWaiting() > 0:
                self.inputBuf = MotorConn.readline()
                try:
                    if self.inputBuf[0] == "$":
                        try:
                            self.inputBuf = self.inputBuf.rstrip("\n")
                            self.inputBuf = self.inputBuf.lstrip("$")                            
                            self.inputBuf = self.inputBuf.rsplit(",")
                            with threadLock:
                                motordata = self.inputBuf 
                        except Exception as e:
                            logging.error("%s", str(e))
                    elif self.inputBuf[0] == "&":
                        try:
                            self.inputBuf = self.inputBuf.rstrip("\n")
                            self.inputBuf = self.inputBuf.lstrip("&") 
                            self.inputBuf = self.inputBuf.rsplit(",")
                            # print self.inputBuf
                            with threadLock:
                                leftpulse = self.inputBuf[0]
                                rightpulse = self.inputBuf[1]
                                odomLeftPulse += float(leftpulse)
                                odomRightPulse += float(rightpulse)
                                LIDARLeftPulse += float(leftpulse)
                                LIDARRightPulse += float(rightpulse)
                        except Exception as e:
                            logging.error("%s", str(e))
                    elif self.inputBuf[0] == "%":
                        try:
                            self.inputBuf = self.inputBuf.lstrip("%") 
                            print self.inputBuf
                        except Exception as e:
                            logging.error("%s", str(e))
                except Exception as e:
                    logging.error("%s", str(e))
            time.sleep(0.001)
        logging.info("Exiting")

    def send_serial_data(self, sendCommand):
        global lastSent
        global threadLock
        global last_time_sent
        #print "Sending Speed Vector"
        logging.info("Sending Speed Vector - %s", str(sendCommand))
        if self.hexData: #construct the command to be sent.
            sendData = "$"
            if sendCommand[0] >= 0:
                sendData += "+"
            else:
                sendData += "-"
            if abs(sendCommand[0]) < 16:
                sendData += "0"
            sendData += hex(abs(sendCommand[0]))[2:]

            if sendCommand[1] >= 0:
                sendData += "+"
            else:
                sendData += "-"
            if abs(sendCommand[1]) < 16:
                sendData += "0"
            sendData += hex(abs(sendCommand[1]))[2:]

            sendData += "R" + "\n"

        else:
            sendData = "+" + str(sendCommand[0]) + "," + str(sendCommand[1]) + "\n"

        try:
            MotorConn.write(sendData)
            logging.info("Successfully sent... - %s", str(sendData))
            with threadLock :
                lastSent = sendCommand[:]
                last_time_sent = time.time()
            # time.sleep(10)
            # print MotorConn.read(MotorConn.inWaiting())
        except Exception as e:
            logging.error("Sending to Motor failed - %s", str(e))

        
class usDataThread(MultiThreadBase):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.rawUSdata_1 = [0., 0., 0., 0.]
        self.rawUSdata_2 = [0., 0., 0., 0.]

        self.inputBuf1 = ""
        self.inputBuf2 = ""
        self.errorCount = 0


    def getUSvector1(self):
        while not exitFlag:
            try:
                self.inputBuf1 = USConn1.readline()
                self.inputBuf1 = self.inputBuf1.rstrip(",\n")
                self.rawUSdata_1 = self.inputBuf1.split(",")
                #print ("rw1 ", self.inputBuf1)
            except Exception as e:
                logging.error("%s", e)

    def getUSvector2(self):
        while not exitFlag:
            try:
                self.inputBuf2 = USConn2.readline()
                self.inputBuf2 = self.inputBuf2.rstrip(',\n')
                self.rawUSdata_2 = self.inputBuf2.split(",")
                #print ("rw2 ", self.rawUSdata_2)
            except Exception as e:
                logging.error("%s", e)

    def getMaxSpeeds(self,usData):
        global stoppingDistance
        global maxSpeeds
        global threadLock
        tempMaxSpeeds = [0,0,0,0,0,0,0,0]
        #aMax = (2*np.pi*1000)/(60*0.12)
        aMax = 872.664626 # Calculated from line above which was supplied by a previous group 
        #radToRPM = 60/(2*np.pi)
        for i in range(0,len(usData)):
            if int(usData[i]) >= stoppingDistance:
                tempMaxSpeeds[i] = min(int(np.sqrt(aMax*(((usData[i]/100) -(stoppingDistance/100))))), SLAMMaxSpeed)
            else:
                tempMaxSpeeds[i] = 0
        
        
        with threadLock :
            maxSpeeds = tempMaxSpeeds
        #print "max speeds = " + str(maxSpeeds)

    def run(self):
        global speedVector
        logging.info("US Starting")
        try:
            USConn1.flushInput()
            if UShosts == 2:
                USConn2.flushInput()
        except Exception as e:
            logging.error("%s", str(e))

        getUS1 = threading.Thread(target= self.getUSvector1)
        getUS1.daemon = True
        getUS1.start()

        getUS2 = threading.Thread(target= self.getUSvector2)
        getUS2.daemon = True
        getUS2.start()

        while not exitFlag:
            try:
                self.mAverage(11)
                self.getMaxSpeeds(USAvgDistances)
                self.errorCount = 0
                #print "avg US = " + str(USAvgDistances)

            except Exception as e:
                self.errorCount += 1
                logging.error("%s", str(e))
                if USConn1.closed:
                    try:
                        logging.debug("Trying to open US1 serial port ")
                        USConn1.open()
                    except Exception as e:
                        self.errorCount += 1
                        logging.error("Trying to open Ultrasonic port 1 - %s", str(e))
                        logging.info("No Ultrasonic comms... Looping Back...")

                if UShosts == 2 and USConn2.closed:
                    try:
                        logging.debug("Trying to open US2 serial port")
                        USConn2.open()
                    except Exception as e:
                        self.errorCount += 1
                        logging.error("Trying to open Ultrasonic port 2 - %s", str(e))
                        logging.info("No Ultrasonic comms... Looping Back...")

            if self.errorCount > 3:
                logging.warning("Consecutive Exceptions... pausing execution...")
                self.errorCount = 0
                time.sleep(3)
                logging.info("US resuming")

            self.US_safety_interrupt()


        logging.info("Exiting")

    def mAverage(self, n): # moving average of raw data
        global USAvgDistances
        global threadLock
        i = 0

        rawUSdata = self.rawUSdata_1 + self.rawUSdata_2
        if len(rawUSdata) == 8:
            with threadLock:
                for i in range(0, len(USAvgDistances)):
                    USAvgDistances[i] += (int(rawUSdata[i]) - USAvgDistances[i])/n
          
        #print USAvgDistances
        #logging.info(USAvgDistances) 

    def US_safety_interrupt(self):
        global USAvgDistances
        global obstruction
        global threadLock
        global usCaution
        global speedVectorMaxSpeed

        if safetyOn:
            try:
                # If heading forwards - check front USs for obstruction
                if (speedVector[0] > 0 ) and (speedVector[1] > 0):
                    speedVectorMaxSpeed = min(maxSpeeds[2:3])
                    if (int(USAvgDistances[2]) < USThresholds[0]) or (int(USAvgDistances[3]) < USThresholds[0]):
                        logging.warning("FRONT TOO CLOSE. STOPPPPP!!!")
                        if obstruction != True:
                            with threadLock:
                                obstruction = True
                        #print ("\t" + self.name + " Avg Vector - " + str(int(USAvgDistances[2])) + ", " + str(int(USAvgDistances[2])))
                    elif obstruction != False:
                        with threadLock:
                            obstruction = False

                    if (maxSpeeds[2] > abs(speedVector[0]) or (maxSpeeds[3] > abs(speedVector[0]))):
                        # print("CAUTION\n")
                        usCaution = True
                    else:
                        usCaution = False

                # If heading backwards - check backwards USs for obstruction
                elif (speedVector[0] < 0) and (speedVector[1] < 0 ):
                    speedVectorMaxSpeed = min(maxSpeeds[6:7])
                    if (int(USAvgDistances[6]) < USThresholds[2]) or (int(USAvgDistances[7]) < USThresholds[2]):
                        logging.warning("BACK TOO CLOSE. STOPPPPP!!!")
                        if obstruction != True:
                            with threadLock:
                                obstruction = True
                        print ("\t" + self.name + " Avg Vector - " + str(int(USAvgDistances[6])) + ", " + str(int(USAvgDistances[7])) )
                    elif obstruction != False:
                        with threadLock:
                            obstruction = False

                    if (maxSpeeds[6] > abs(speedVector[0]) or (maxSpeeds[7] > abs(speedVector[0]))):
                        usCaution = True
                        # print("CAUTION\n")
                    else:
                        usCaution = False

                # If heading left - check left USs for obstruction
                elif (speedVector[0]< 0 ) and (speedVector[1] > 0):
                    speedVectorMaxSpeed = min(maxSpeeds[0:1])
                    if (int(USAvgDistances[0]) < USThresholds[1]) or (int(USAvgDistances[1]) < USThresholds[1]):
                        logging.warning("LEFT SIDE TOO CLOSE. STOPPPPP!!!")
                        if obstruction != True:
                            with threadLock:
                                obstruction = True
                        print ("\t" + self.name + " Avg Vector - " + str(int(USAvgDistances[0])) + ", " + str(int(USAvgDistances[1])))
                    elif obstruction != False:
                        with threadLock:
                            obstruction = False

                # If heading right - check right USs for obstruction
                elif (speedVector[0] > 0) and (speedVector[1] < 0) :
                    speedVectorMaxSpeed = min(maxSpeeds[5:6])
                    if (int(USAvgDistances[5]) < USThresholds[1]) or (int(USAvgDistances[6]) < USThresholds[1]):
                        logging.warning("RIGHT SIDE TOO CLOSE. STOPPPPP!!!")
                        if obstruction != True:
                            with threadLock:
                                obstruction = True
                        print ("\t" + self.name + " Avg Vector - " + str(int(USAvgDistances[5])) + ", " + str(int(USAvgDistances[6])))
                    elif obstruction != False:
                        with threadLock:
                            obstruction = False


            except Exception as e:
                self.errorCount += 1
                logging.error("error in US interrupt function - %s", str(e))
        else:
            obstruction = False

            
class datafromUI(MultiThreadBase):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.RecieveServer = False
        self.clientconnect = False

    def startServer(self):
        logging.info("Setting up sockets...")
        try:
            PORT = 5002
            HOST = ''
            # create a socket to establish a server
            logging.debug("Binding recieveing Socket")   
            self.ServerSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.ServerSocket.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1) 
            self.ServerSocket.bind((HOST, PORT))
            self.ServerSocket.settimeout(5)

            # listen to incoming connections on PORT
            logging.info("Socket opened at %s listening to port %s", HOST, PORT)
            self.ServerSocket.listen(1)
            self.RecieveServer = True

        except Exception as e:
            logging.error("%s", str(e))
            self.RecieveServer = False
            logging.debug("Sleeping...")
            time.sleep(5)

    def userconnect(self):
        try:
            logging.info("Ready for a new client to connect...")
            self.clientConnection, self.address = self.ServerSocket.accept()
            logging.info('Connected by %s', self.address)
            print 'Connected by', self.address

            # send welcome message
            print ("Sending welcome message...")
            self.clientConnection.send('Connection ack')
            dataInput = self.clientConnection.recv(1024) # REVISIT : is dataInput meant to be a global variable? if so it should be using threadLock and should have 'global dataInput\nglobal threadLock' in the function... but it isn't used outside this class so maybe it should be local?
            print ("Client says - " + dataInput)
            dataInput = ""
            self.clientconnect = True
        except Exception as e:
            logging.error("%s", str(e))

    def run(self):
        global sysRunning
        global safetyOn
        global exitFlag
        global threadLock
        global enduserloop
        logging.info("Starting %s", self.name)
        while not exitFlag:
            if not self.RecieveServer:
                self.startServer()
            if self.RecieveServer and not self.clientconnect:
                self.userconnect()
            if self.clientconnect:
                try:
                    dataInputJson = self.clientConnection.recv(1024)
                    dataInputTidied = dataInputJson.split("$",1) # this code is to remove any random characters sent by acident with the socket connection
                    print "Command Recieved = " + dataInputTidied[0]
                    dataInput = json.loads(dataInputTidied[0]) 
                    if (dataInput['Type'] == "MiscCommand"):
                        if dataInput['Command'] == "x":
                            with threadLock:
                                enduserloop = True
                            if system_status == "UserCommand":
                                with commandqueue.mutex:
                                    commandqueue.queue.clear()
                                stop = {"Left":0, "Right":0}
                                speedsqueue.put(stop)

                            sysRunning = False
                            commandqueue.put("Close")
                        elif dataInput['Command'] == "s":
                            with threadLock:
                                safetyOn = not safetyOn ; 
                    elif dataInput['Type'] == "Cancel": # If the user wants to cancel the mapping or nav command and move onto the next item in the queue this is ran
                        with threadLock:
                            enduserloop = True

                        # add code to end loops # REVISIT : Is this not above?

                    elif dataInput['Type'] == "CancelEnterUserCommand": # When the user wants to enter a command immediatly it clears the queue. This is called straight before a switch to the User Control mode occurs
                        with commandqueue.mutex:
                            commandqueue.queue.clear()
                        with threadLock:
                            enduserloop = True
                        stop = {"Left":0, "Right":0}
                        speedsqueue.put(stop)

                        # add code to end loops 

                    elif(dataInput['Type'] == "UserSpeed"): # If the user sent a speed command it is added to the speed queue
                        print "UserSpeed"
                        speedsqueue.put(dataInput)
                    else: # REVISIT : Perhaps validate here
                        commandqueue.put(dataInput)
                except Exception as e:
                    logging.error("%s", str(e))

            
class lidarInterfaceThread(MultiThreadBase):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.sAngle  = 360 / numSamples
        self.currentAngle  = 0
         
    def checkLIDARConn(self) :
        global LIDARConn
        if LIDARConn.closed:
            try:
                logging.info("Trying to open lidar port")
                LIDARConn.open()
            except Exception as e:
                logging.error("%s", str(e))
            finally:
                logging.info("No LIDARConn Comms... Looping back to listening mode")

    def run(self) :
        global lidarRun
        global threadLock
        global lidarStartTime
        global lidarEndTime
        global LIDARConn
        self.currentAngle = 0

         # Loop until told to stop
        while not exitFlag:
            # Enact any user requests
            if lidarRun == "a" :                
                with threadLock :
                    lidarRun = ""
                try :
                    LIDARConn.write("4\n") # run continuous
                except Exception as e :
                    logging.error("%s", str(e))
                    self.checkLIDARConn()
            elif lidarRun == "s" :                
                with threadLock :
                    lidarRun = ""
                try :
                    LIDARConn.write("1\n") # stop!
                except Exception as e :
                    logging.error("%s", str(e))
                    self.checkLIDARConn()

            # Read data from lidar 
            if lidarStartTime == 0 :
                with threadLock :
                    lidarStartTime = time.time()
                    
            input = LIDARConn.readline()[:-2]

            if input == "$" :
                logging.debug("First sample of LIDAR recieved")
                self.currentAngle = 0
            else :     
                # Attempt to convert to float - else set to 0
                try :
                    r = float(input)
                except Exception as e:
                    r = 0
                    logging.warning("Invalid lidar sample ignored")

                with threadLock :
                    lidarPolarList.append({"angle": self.currentAngle, "dist" : r })
                self.currentAngle += self.sAngle
            
            with threadLock :
                lidarEndTime = time.time()    
                
            time.sleep(0.001)

        # Turn off lidar on shutdown
        try :
            LIDARConn.write("4\n") # run continuous
        except Exception as e :
            logging.error("%s", str(e))
            self.checkLIDARConn()
                    
  
class SLAMThread(MultiThreadBase):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name 
        
    # REVISIT : THIS WILL NOT WORK - NEEDS UPDATING FOR REAL PORTER
    def calculatePorterPosition(self) :
        global porterLocation
        global porterOrientation
        global porterOrientationUnConst
        global porterImuOrientation
        global wheelSpeeds
        global threadLock
        global dataMap
        global totalR
        global totalWd
        global odomLeftPulse
        global odomRightPulse

        if odomLeftPulse != 0 and odomRightPulse != 0 :  
            with threadLock :
                leftDelta  = odomLeftPulse * pulseDistConst
                rightDelta = odomRightPulse * pulseDistConst
                odomLeftPulse = 0
                odomRightPulse = 0
                
            totalR += (leftDelta + rightDelta)
            orientation = math.radians(porterOrientation - 90)
            orientationUnconst = math.radians(porterOrientationUnConst - 90)
            x   = porterLocation[0] - realPorterWheelOffsetY*math.cos(orientation)
            y   = porterLocation[1] - realPorterWheelOffsetY*math.sin(orientation)

            if (math.fabs(leftDelta - rightDelta) < 1.0e-6) : # basically going straight
                new_x = x + leftDelta * math.cos(orientation)
                new_y = y + rightDelta * math.sin(orientation)
                new_heading = orientation
                new_headingUnConst = orientationUnconst
            else :
                R = pixelPorterSize[0] * (leftDelta + rightDelta) / (2 * (rightDelta - leftDelta))
                wd = (rightDelta - leftDelta) / pixelPorterSize[0] 
                new_x = x + R * math.sin(wd + orientation) - R * math.sin(orientation)
                new_y = y - R * math.cos(wd + orientation) + R * math.cos(orientation)
                
                totalWd += wd/ 2   #   self.scale # REVISIT : why is scale needed?
                new_heading = orientation + wd/ 2 # self.scale;
                new_headingUnConst = orientationUnconst + wd/ 2 # self.scale;
            
            with threadLock :
                porterLocation = (new_x + realPorterWheelOffsetY*math.cos(orientation),new_y + realPorterWheelOffsetY*math.sin(orientation))
                porterOrientation = (math.degrees(new_heading) + 90)*0.5 + porterImuOrientation*0.5
                porterOrientationUnConst = (math.degrees(new_headingUnConst) + 90) # + porterImuOrientation*0.5
                porterOrientation = constrainAngle360(porterOrientation, 180, -180)

    # REVISIT : encoderVectorAbs needs to be created, work out real rotation times of lidar, 
    def adjustLidar(self) :
        global encoderVectorAbs
        global lidarPolarList
        global adjustedLidarStore
        global adjustedLidarList
        global adjustedLidarListReady
        global dataMap
        global lidarData
        global lidarStartTime
        global lidarEndTime
        global leftDelta
        global rightDelta
        global LIDARLeftPulse
        global LIDARRightPulse
        global numSamples
        

        if len(lidarPolarList) > 0 : # this should always be true at this point in simulation
            with threadLock:
                lidarData = list(lidarPolarList)
                lidarPolarList = []
                leftDelta  = LIDARLeftPulse * pulseDistConst
                rightDelta = LIDARRightPulse * pulseDistConst
            # print "new loop"
            #print str(lidarData)
            # REVISIT : Get more accurate times?
            sampleTime =  lidarEndTime - lidarStartTime
            sampleTimeFraction = sampleTime / len(lidarData)
            distFraction = (leftDelta*sampleTimeFraction,rightDelta*sampleTimeFraction)
            
            # clear variables
            with threadLock :
                LIDARLeftPulse = 0
                LIDARRightPulse = 0
                lidarStartTime = 0
                lidarEndTime = 0
            #self.lidarTotalNSamples += len(lidarData)
            for i, sample in enumerate(lidarData) :
                # do calculations else if sample is end sample then push all stored calculated values into output and set ready 
                if sample != "END" :
                    print "Adjusting"
                    # convert coordinate to x y 
                    sampleXY = (sample["dist"]*math.cos(math.radians(sample["angle"]-90)),sample["dist"]*math.sin(math.radians(sample["angle"]-90)))
                    # add motion i*distFraction
                    j = len(lidarData) - i # adjust samples to current position
                    sampleXY = (sampleXY[0]+distFraction[0]*j,sampleXY[1]+distFraction[1]*j)
                    # convert back to polar # REVISIT : ew square root
                    r = math.sqrt(sampleXY[0]**2 + sampleXY[1]**2)
                    # save to store
                    adjustedLidarStore.append(r*10)
                else : 
                    print "Leaving Adjust Lidar"
                    with threadLock :
                        adjustedLidarList = adjustedLidarStore[-self.lidarTotalNSamples-1:]
                        #with open("adjustLidar.log","a+") as f:
                        #    f.write("new\n" + str(adjustedLidarList) + "\n" + str(adjustedLidarStore)+"\n")
                        adjustedLidarStore = []
                        adjustedLidarListReady = True
                         
    def run(self) :
        global adjustedLidarList
        global adjustedLidarListReady
        global SLAMObject
        global SLAMRunning
        global porterLocation
        global porterOrientation
        global porterOrientationUnConst
        global threadLock
        global SLAMRunning
        global runSLAM
        global pathMap
        
        while not exitFlag :
            if runSLAM == "start" :
                with threadLock :
                    SLAMRunning = True
                    runSLAM = ""
            elif runSLAM == "stop" :
                with threadLock :
                    SLAMRunning = False
                    runSLAM = ""
            elif runSLAM == "loadMap" :
                with threadLock :
                    SLAMObject.setmap(loadMap)
                    runSLAM = ""

            if SLAMRunning :  
                self.calculatePorterPosition()
                self.adjustLidar()
                if adjustedLidarListReady:
                    SLAMObject.update(adjustedLidarList) #, (dxy*20, globalWd, 0.015 ))
                    # update position
                    with threadLock: 
                        x, y, newPorterOrientation = SLAMObject.getpos()
                        newPorterOrientation = constrainAngle360(newPorterOrientation, 180, -180)
                        dOrientation         = constrainAngle360(newPorterOrientation-porterOrientation, 180, -180)
                        porterOrientationUnConst += dOrientation
                        porterOrientation = newPorterOrientation
                        xSim =  y/20
                        ySim =  SLAMOffset[0] - x/20
                        porterLocation = (xSim + realPorterLIDAROffsetY*math.cos(porterOrientation),ySim + realPorterLIDAROffsetY*math.sin(porterOrientation))
                        adjustedLidarListReady = False
                    print "Slam Map Entering"
                    pathMap.updateMap()
                time.sleep(0.001)
    
# Class definitions 

class pathMapClass() :
    def __init__(self, dilateAmount) :
        global realPorterSize
        global threadLock
        global navMap
        global MAP_SIZE_PIXELS
        self.wallMap = set()
        self.SLAMScale               = 2 # 2
        self.mapGridResolution       = 10 #10 #cm how far apart are the grid nodes
        self.scaleAdjust             = float(self.SLAMScale)/self.mapGridResolution # 2/10 = 0.2 * the number of pixels
        self.dilateAmount            = dilateAmount
        self.wallSafetyDistance      = realPorterSize[0] / 2
        self.wallSafetyGridRadius    = int(math.ceil(self.wallSafetyDistance / self.mapGridResolution)*20)
        self.cornerPenaltyWeight     = 100
        self.cornerAngleWeight       = 100
        self.angleOffsetLookup       = { 0 : [0,-1,0],
                                    1 : [1,-1,45],
                                    2 : [1,0,90],
                                    3 : [1,1,135],
                                    4 : [0,1,180],
                                    5 : [-1,1,225],
                                    6 : [-1,0,270],
                                    7 : [-1,-1,315],
                                  }
        self.angleWeight       =    {         
                                        0 : 0,
                                        1 : 1,
                                        2 : 2,
                                        3 : 3,
                                        4 : 4,
                                        5 : 3,
                                        6 : 2,
                                        7 : 1,
                                    }
        self.pathMapLimits = {  "xMin" : 0,
                                "xMax" : int(MAP_SIZE_PIXELS-1), # May have to flip this
                                "yMin" : 0,
                                "yMax" : int(MAP_SIZE_PIXELS-1),
                }                                
        with threadLock :        
            navMap                       = np.zeros([self.pathMapLimits["xMax"],self.pathMapLimits["yMax"]])
    
    
    def updateMap(self) :
        global threadLock
        global navMap
        global detailedMap
        global SLAMObject
        #print "Slam Map"
        # Make array for SLAM map
        mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)
        # Get SLAM map
        SLAMObject.getmap(mapbytes)
        # Convert map into numpy array
        SLAMMap = np.frombuffer(mapbytes, dtype='B')
        #cv2.imwrite('afterbuffer.jpg', SLAMMap)
        SLAMMap.shape =  (-1,MAP_SIZE_PIXELS)

        # Transform SLAMMap - threshold to get walls,reduce size,  dilate to expand walls for safety
        # threshold to find all areas that are walls
        _,tempMap = cv2.threshold(SLAMMap,126,255,cv2.THRESH_BINARY)
        # Invert to make walls white for dilation
        tempMap = cv2.bitwise_not(tempMap)
        # Make kernal for dilation
        kernel = np.ones((self.dilateAmount,self.dilateAmount),np.uint8)
        # Dilate to give safety margin to walls
        tempMap = cv2.dilate(tempMap, kernel)
        # Convert walls back to black 
        tempMap = cv2.bitwise_not(tempMap)
        # Resize to make pathfinding quicker
        # Rotate map
        rows,cols = tempMap.shape
        M = cv2.getRotationMatrix2D((cols/2,rows/2),90,1)
        tempMap = cv2.warpAffine(tempMap,M,(cols,rows))
        rows,cols = SLAMMap.shape
        M = cv2.getRotationMatrix2D((cols/2,rows/2),90,1)
        SLAMMap = cv2.warpAffine(SLAMMap,M,(cols,rows))
        #tempPathMap = cv2.resize(tempMap, (self.pathMapLimits["xMax"],self.pathMapLimits["yMax"]))
        #tempMap = cv2.transpose(tempMap)

        with threadLock :
            navMap = tempMap
            detailedMap = SLAMMap
    
    
    def getNeighbors(self, id, dest) :
        file = open("getNeighbors.log","a+")
        file.write("New#########################" + "\n")
        edges = {}
        x = id[0]
        y = id[1]
        a = id[2]
        file.write("id: " + str(id) + "\n")
        # Add links to same node at different angles
        # Distance from this square to the destination
        for aLink in range(0,8) :
            # Calc the new cell location
            nx = x + self.angleOffsetLookup[aLink][0]*self.mapGridResolution
            ny = y + self.angleOffsetLookup[aLink][1]*self.mapGridResolution
            file.write("n: " + str((nx,ny)) + "\n")
            file.write("n: " + str(self.isWall((nx,ny))) + "\n")
        
            if not self.isWall((nx,ny)) :
                # Calc distance to goal
                dist = getLengthSquared([dest[0],dest[1],nx,ny]) # math.sqrt(( dest[0] - nx )**2 + ( dest[1] - ny )**2)
                # Calc distance between the two cells (weight)
                weight = (abs(self.angleOffsetLookup[aLink][0])+abs(self.angleOffsetLookup[aLink][1]))*self.mapGridResolution #math.sqrt(( x - nx )**2 + ( y - ny )**2)
                if aLink != a :
                    weight += self.cornerPenaltyWeight + self.angleWeight[abs(aLink-a)]*self.cornerAngleWeight

                edges[(nx, ny, aLink)] = {  "weight"     : weight,
                                        "distToGoal" : dist}

        return edges
        
        
    def isWall(self, coord) :
        global navMap
        #print("isWall? ----------")
        coord = (coord[1],coord[0])
        #print(coord)
        if coord[0] < self.pathMapLimits["xMin"] or coord[0] > self.pathMapLimits["xMax"] or coord[1] < self.pathMapLimits["yMin"] or coord[1] > self.pathMapLimits["yMax"] :
            print("failed boundary test")
            return True
        
        #print(navMap[coord[0]][coord[1]])
        if navMap[coord[0]][coord[1]] < 127 :
            #print("valid")
            return True
 
        #print("not equal to zero")
        return False
        
        
    # Similar to lidar code
    # Check every cell along line given starting point, distance and angle
    # Return true if there is no collsion or false if collision is detected
    def checkLine(self, x0, y0, dist, angle) :
        # Generate a list of points on line going from centre of porter out at angle given
        linePoints = list()
        # Absolute angle
        a = constrainAngle360(angle, 360,0)
        # End points of line 
        xMax = x0 + int(round(dist*math.cos(math.radians(a))))
        yMax = y0 + int(round(dist*math.sin(math.radians(a))))
        # List of points along line
        linePoints = list(bresenham(x0,y0,xMax, yMax))
        linePoints = [(roundBase(point[0],self.mapGridResolution),roundBase(point[1],self.mapGridResolution)) for point in linePoints]
        linePoints = set(linePoints)

        # Find collision location
        for point in linePoints :
            if self.isWall(point) :
                # If there os a collision then the line is not valid
                return False
                
        # After scanning all points the line must be valid
        return True
        
        
    def getDataMap(self) :
        return self.wallMap

# https://github.com/ivmech/ivPID/blob/master/PID.py
class PID:
    """PID Controller
    """

    def __init__(self, P=0.2, I=0.0, D=0.0):

        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.sample_time = 0.00
        self.current_time = time.time()
        self.last_time = self.current_time

        self.clear()

    def clear(self):
        """Clears PID computations and coefficients"""
        self.SetPoint = 0.0

        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0

        # Windup Guard
        self.int_error = 0.0
        self.windup_guard = 20.0

        self.output = 0.0

    def update(self, feedback_value):
        """Calculates PID value for given reference feedback
        .. math::
            u(t) = K_p e(t) + K_i \int_{0}^{t} e(t)dt + K_d {de}/{dt}
        .. figure:: images/pid_1.png
           :align:   center
           Test PID with Kp=1.2, Ki=1, Kd=0.001 (test_pid.py)
        """
        error = self.SetPoint - feedback_value
        if error < 180 :
            error = error
        else :
            error = error - 360

        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if (delta_time >= self.sample_time):
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time

            if (self.ITerm < -self.windup_guard):
                self.ITerm = -self.windup_guard
            elif (self.ITerm > self.windup_guard):
                self.ITerm = self.windup_guard

            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / delta_time

            # Remember last time and last error for next calculation
            self.last_time = self.current_time
            self.last_error = error

            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)
            return self.output

    def setKp(self, proportional_gain):
        """Determines how aggressively the PID reacts to the current error with setting Proportional Gain"""
        self.Kp = proportional_gain

    def setKi(self, integral_gain):
        """Determines how aggressively the PID reacts to the current error with setting Integral Gain"""
        self.Ki = integral_gain

    def setKd(self, derivative_gain):
        """Determines how aggressively the PID reacts to the current error with setting Derivative Gain"""
        self.Kd = derivative_gain

    def setWindup(self, windup):
        """Integral windup, also known as integrator windup or reset windup,
        refers to the situation in a PID feedback controller where
        a large change in setpoint occurs (say a positive change)
        and the integral terms accumulates a significant error
        during the rise (windup), thus overshooting and continuing
        to increase as this accumulated error is unwound
        (offset by errors in the other direction).
        The specific problem is the excess overshooting.
        """
        self.windup_guard = windup

    def setSampleTime(self, sample_time):
        """PID that should be updated at a regular interval.
        Based on a pre-determined sampe time, the PID decides if it should compute or return immediately.
        """
        self.sample_time = sample_time

                  
class controlClass() :
    
    def __init__(self, scale):
        self.scale = scale
        self.angleOffsetLookup       = { 0 : [0,-1,0],
                                    1 : [1,-1,45],
                                    2 : [1,0,90],
                                    3 : [1,1,135],
                                    4 : [0,1,180],
                                    5 : [-1,1,225],
                                    6 : [-1,0,270],
                                    7 : [-1,-1,315],
                                  }
        
        
    def getDesOrientation(self, start, end) :
        #print("getDesOrient: start: " + str(start) + ", end: " + str(end))
        #print("orientation: " + str(constrainAngle360(-90-math.degrees(math.atan2(end[1] - start[1], end[0] - start[0])), 180, -180)))
        return +90+math.degrees(math.atan2(end[1] - start[1], end[0] - start[0]))
        
        
    # Maintains the orientation using a PID loop and moves a distance in that direction
    # REVISIT : Could add adjustment to heading if off course
    def moveStraight(self, distance) :
        global porterLocation
        global porterOrientation
        global porterOrientationUnConst
        global speedVector
        global enduserloop
        global threadLock
        
        origLocation = porterLocation
        desOrientation = porterOrientation
        # desLocation = [porterLocation[0] - distance * math.cos(math.radians(desOrientation+90)),
                             # porterLocation[1] - distance * math.sin(math.radians(desOrientation+90))]

        # orientation is critical as any error will severely affect navigation
        # A PID loop will be used to adjust the speedVector to keep the orientation consistent
        # Stop when within Xcm of final destination OR when travelled distance
        orientationPID = PID(0.05, 0.1, 0)
        orientationPID.SetPoint=0
        orientationPID.setSampleTime(0.05)
        # maintain porterOrientation whilst moving
        atDest = False 
        atObstacle = False
        orientationAdjust = 0
        prevOrientationAdjust = 9999999
        while (not atDest) and (not atObstacle) and (not enduserloop):
            time.sleep(0.01)
            speed = speedVectorMaxSpeed
            if speed == 0 :
                return False
            #print(porterOrientation-desOrientation)
            orientationAdjust = orientationPID.update(porterOrientation-desOrientation)
            if (orientationAdjust != None) and (abs(orientationAdjust - prevOrientationAdjust)>0.0001) :
                with threadLock :
                    speedVector = [0.9*(1-orientationAdjust)*speed,0.9*(1+orientationAdjust)*speed]
            distSq = (porterLocation[0]-origLocation[0])**2 + (porterLocation[1]-origLocation[1])**2

            # until at destination                
            if (distSq >= distance**2) :
                atDest = True
                
        # final speed is the previous speed this must be dealt with later
        return atDest        
    # Maintains the orientation using a PID loop and moves a distance in that direction
    # REVISIT : Could add adjustment to heading if off course
    def moveTo(self, dest, f) :
        global porterLocation
        global porterOrientation
        global porterOrientationUnConst
        global speedVector
        global enduserloop
        global threadLock
        #print("in moveTo, origLoc: " + str(porterLocation) + ", dest: " + str(dest))
        f.write("\n\nin moveTo, origLoc: " + str(porterLocation) + ", dest: " + str(dest) + "\n")
        origLocation = porterLocation
        desOrientation = constrainAngle360(self.getDesOrientation(origLocation, dest), porterOrientationUnConst + 180, porterOrientationUnConst - 180)
        destPID = (dest[0] + self.angleOffsetLookup[dest[2]][0]*100 , dest[1] + self.angleOffsetLookup[dest[2]][1]*100 , dest[2])
        
        #print("origOrientation: " + str(porterOrientation) + ", desOrientation: " + str(desOrientation))
        f.write("\torigOrientation: " + str(porterOrientation) + ", desOrientation: " + str(desOrientation) + "\n")

        # orientation is critical as any error will severely affect navigation
        # A PID loop will be used to adjust the speedVector to keep the orientation consistent
        # Stop when within Xcm of final destination OR when travelled distance
        orientationPID = PID(0.05, 0.01, 0)
        orientationPID.SetPoint=0
        orientationPID.setSampleTime(0.01)
        # Using angle of initial travel to determine whether final condition tests x or y values
        if (abs(origLocation[0]-dest[0]) > abs(origLocation[1]-dest[1])) : # if line is more vertical than horizontal then # REVISIT : this check is not correct
            # set offset value to select y coordinates
            yOffset = 0
        else :
            # set offset value to select x coordinates
            yOffset = 1
        f.write("\tyOffset: " + str(yOffset) + "\n")

        # Find whether final condition must be greater than or less than
        if origLocation[yOffset] < dest[yOffset] : 
            testSign = 1
        else :
            testSign = -1
        f.write("\ttestSign: " + str(testSign) + "\n")
        # maintain porterOrientation whilst moving
        atDest = False 
        atObstacle = False
        orientationAdjust = 0
        prevOrientationAdjust = 9999999
        while (not atDest) and (not atObstacle) and (not enduserloop):
            desOrientation = constrainAngle360(self.getDesOrientation(porterLocation, dest), porterOrientationUnConst + 180, porterOrientationUnConst - 180)
            f.write("\tloop start" + "\n")
            f.write("\tporterLocation: " + str(porterLocation) + "\n")
            f.write("\tdesOrientation: " + str(desOrientation) + "\n")
            speed = speedVectorMaxSpeed
            if speed == 0 :
                return False
            f.write("\tspeed: " + str(speed) + "\n")
            
            f.write("\tporterOrientationUnConst: " + str(porterOrientationUnConst) + "\n")
            #print("desOrientation: " + str(desOrientation))
            f.write("\terror: " + str(porterOrientationUnConst-desOrientation) + "\n")
            f.write("\terrorconstr: " + str(constrainAngle360(porterOrientationUnConst-desOrientation,180,-180)) + "\n")
            orientationAdjust = constrainFloat(orientationPID.update(constrainAngle360(porterOrientationUnConst-desOrientation,180,-180)),1,-1)
            f.write("\torientationAdjust: " + str(orientationAdjust) + "\n")
            
            #print("adjustment: " + str(orientationAdjust))
            if (orientationAdjust != None) and (abs(orientationAdjust - prevOrientationAdjust)>0.0001) :
                with threadLock :
                    speedVector = [max(0.9*(1-orientationAdjust)*speed, 0),min(0.9*(1+orientationAdjust)*speed, speed)]
                    
                f.write("\tspeedVector: " + str(speedVector) + "\n")
            # until at destination  
            #print("dest: " + str(dest) + ", current: " + str(porterLocation) + ", angle: " + str(porterOrientation))
            # REVISIT : THIS SHOULD WORK BUT DOESN'T
            
            f.write("\tdest[yOffset]-porterLocation[yOffset])*testSign" + str((dest[yOffset]-porterLocation[yOffset])*testSign) + "\n")
            #f.write( + "\n")
            if (dest[yOffset]-porterLocation[yOffset])*testSign < 0 :
            #if ((porterLocation[0]-dest[0])**2 + (porterLocation[1]-dest[1])**2) < 100 :
                f.write("\tatDest" + "\n")
                atDest = True
            # wait
            time.sleep(0.01)
                
        # final speed is the previous speed this must be dealt with later
        return atDest
        
    # Positive angles will turn the robot clockwise and vice versa
    # Type controls the point which remains still during the turn
    def moveTurn(self, angle, type, f) :
        global threadLock
        global porterOrientation
        global porterOrientationUnConst
        global speedVector
        f.write("\n\nin moveTurn, origAngle: " + str(porterOrientation) + ", angle: " + str(angle) + "\n")
        #print("\n\nin moveTurn, origAngle: " + str(porterOrientation) + ", angle: " + str(angle) + "\n")
        
        origOrientation = porterOrientationUnConst
        maxSpeed = speedVectorMaxSpeed
        if maxSpeed == 0 :
            return False
        sign = 1 if angle > 0 else -1
        
        desOrientation = origOrientation + angle
        # if angle > 0 :        
            # # Calculate final porterOrientation
            # desOrientation = constrainAngle360(porterOrientation+angle,origOrientation+180,origOrientation-180)
        # else :
            # # Calculate final porterOrientation
            # desOrientation = constrainAngle360(porterOrientation-angle,origOrientation+180,origOrientation-180)

        f.write("\tdesOrientation: " + str(desOrientation) + "\n")
        if type == "onWheel" :
            if angle > 0 :        
                # Turning clockwise therefore right wheel is still
                with threadLock : 
                    speedVector = [0, maxSpeed]
            else :
                # Turning anti clockwise therefore left wheel is still
                with threadLock : 
                    speedVector = [maxSpeed,0]
            
        
        if type == "onCentre" :
            # Even speed from each wheel but opposite
            with threadLock : 
                speedVector = [maxSpeed*sign*-1,maxSpeed*sign]
        
        #if type == "onRadius" :
        #    pass
        
        f.write("\tspeedVector: " + str(speedVector) + "\n")
        # Wait for angle to have been turned
        totalTurn = 0
        prevOrientation = origOrientation
        done = False
        # while (not enduserloop) and (not done) :
            # rot = porterOrientation - prevOrientation
            # prevOrientation = porterOrientation
            # if angle > 0 :
                # # Must be turning clockwise
                # if rot < 0 :
                    # rot +=360
                # totalTurn += rot
                # if totalTurn >= angle :
                    # done = True
            # else :
                # # Must be turning anticlockwise
                # if rot > 0 :
                    # rot -=360
                # totalTurn += rot
                # if totalTurn <= angle :
                    # done = True
                    
            # time.sleep(0.005)
        #print("porterOrientationUn: " + str(porterOrientationUnConst))
        #print("desOrientation: " + str(desOrientation))
        #print("(porterOrientationUnConst-desOrientation)*sign: " + str((porterOrientationUnConst-desOrientation)*sign))
            
            
        while not (desOrientation-porterOrientationUnConst)*sign < 0  and not enduserloop:
            #print("porterOrientationUn: " + str(porterOrientationUnConst))
            #print("desOrientation: " + str(desOrientation))
            #print("(porterOrientationUnConst-desOrientation)*sign: " + str((porterOrientationUnConst-desOrientation)*sign))
            time.sleep(0.05)
            
        f.write("\tporterOrientation" + str(porterOrientation) + "\n")
        f.write( "\tdone turn"+ "\n")
            
        return done    # Positive angles will turn the robot clockwise and vice versa
        
        
    # Type controls the point which remains still during the turn
    def turnTo(self, dest, type, f) :
        global threadLock
        global porterOrientation
        global porterOrientationUnConst
        global speedVector
        
        origOrientation = porterOrientation
        desOrientation = self.getDesOrientation(porterLocation, dest)
        angle = constrainAngle360(desOrientation - porterOrientation,180,-180)
        
        maxSpeed = speedVectorMaxSpeed
        if maxSpeed == 0 :
            return False
        sign = 1 if angle > 0 else -1
        
        if angle > 0 :        
            # Calculate final porterOrientation
            desOrientation = constrainAngle360(porterOrientation+angle,origOrientation+720,origOrientation)
        else :
            # Calculate final porterOrientation
            desOrientation = constrainAngle360(porterOrientation+angle,origOrientation+1,origOrientation-720)

        if type == "onWheel" :
            if angle > 0 :        
                # Turning clockwise therefore right wheel is still
                with threadLock : 
                    speedVector = [0, maxSpeed]
            else :
                # Turning anti clockwise therefore left wheel is still
                with threadLock : 
                    speedVector = [maxSpeed,0]
            
        
        if type == "onCentre" :
            # Even speed from each wheel but opposite
            with threadLock : 
                speedVector = [maxSpeed*sign*-1,maxSpeed*sign]
        
        #if type == "onRadius" :
        #    pass
        
        # Wait for angle to have been turned
        totalTurn = 0
        prevOrientation = origOrientation
        done = False
        while (not enduserloop) and (not done) :
            rot = porterOrientation - prevOrientation
            prevOrientation = porterOrientation
            if angle > 0 :
                # Must be turning clockwise
                if rot < 0 :
                    rot +=360
                totalTurn += rot
                if totalTurn >= angle :
                    done = True
            else :
                # Must be turning anticlockwise
                if rot > 0 :
                    rot -=360
                totalTurn += rot
                if totalTurn <= angle :
                    done = True
                    
            time.sleep(0.005)
            
        return done
        
    
    # Modified from pseudocode on https://en.wikipedia.org/wiki/A*_search_algorithm#Pseudocode
    def aStar(self, start, goal, timeout) :
        global pathMap
        global enduserloop
        global dataMap
        global threadLock
        file = open("aStar.log", "w")
        file.write("start: " + str(start) + ", goal: " + str(goal) + "\n")
        
        start = (roundBase(start[0],pathMap.mapGridResolution),roundBase(start[1],pathMap.mapGridResolution),start[2])
        goal  = (roundBase(goal[0],pathMap.mapGridResolution),roundBase(goal[1],pathMap.mapGridResolution))
        file.write("start: " + str(start) + ", goal: " + str(goal) + "\n")

        with threadLock :
            dataMap = set()
        
        # The set of nodes already evaluated
        closedSet = set()

        # The set of currently discovered nodes that are not evaluated yet.
        # Initially, only the start node is known.
        openSet = set()
        openSet.add(start)

        # For each node, which node it can most efficiently be reached from.
        # If a node can be reached from many nodes, cameFrom will eventually contain the
        # most efficient previous step.
        cameFrom = {}

        # For each node, the cost of getting from the start node to that node.
        #map with default value of Infinity
        gScore = {}
        
        # The cost of going from start to start is zero.
        gScore[start] = 0

        # For each node, the total cost of getting from the start node to the goal
        # by passing by that node. That value is partly known, partly heuristic.
        #map with default value of Infinity
        fScore = {}
        
        # For the first node, that value is completely heuristic.
        # REVISIT : Is this actually used??
        fScore[start] = getLengthSquared(list(start)+list(goal)) #math.sqrt(( start[0] - goal[0] )**2 + ( start[1] - goal[1] )**2) # distance to dest #heuristic_cost_estimate(start, goal)

        startTime = time.time()
        while (len(openSet) != 0) and (not enduserloop) :
            # check timeout
            timeElapsed = time.time() - startTime
            if timeElapsed > timeout :
                # stop aStar
                return (False, self.reconstruct_path(cameFrom, current))
                
            # current = the node in openSet having the lowest fScore[] value
            current = None
            for node in openSet :
                f = fScore[node]
                #file.write("openSet n: " + str(node) + ", f: " + str(f) + "\n")
                if current == None :
                    current = node 
                    #file.write("new" + "\n")
                elif f < fScore[current] :
                    current = node
                    #file.write("updated" + "\n")
                                
            file.write("Current node: " + str(current) + "\n")
            if (current[0],current[1]) == goal :
                path = self.reconstruct_path(cameFrom, current)
                #with threadLock :
                #    dataMap = set(path)
                file.close()
                return (True,path)

            openSet.discard(current)
            closedSet.add(current)
            with threadLock :
                dataMap = set()
                for node in closedSet :
                    dataMap.add((node[0],node[1]))
            
            neighbors = pathMap.getNeighbors(current, goal)
            file.write("neighbors: " + str(neighbors) + "\n")
            for neighbor, data in neighbors.iteritems() :
                #file.write("Neighbor: " + str(neighbor) + "\n")
                #file.write("closedSet: " + str(closedSet) + "\n")
                if neighbor in closedSet :
                    #file.write("Ignoring" + "\n")
                    continue # Ignore the neighbor which is already evaluated.

                if not (neighbor in openSet) : # Discover a new node
                    #file.write("Adding" + "\n")
                    openSet.add(neighbor)
                
                with threadLock :
                    dataMap.add((neighbor[0],neighbor[1]))
                    
                # The distance from start to a neighbor
                # the "dist_between" function may vary as per the solution requirements.
                tentative_gScore = gScore[current] + data["weight"]
                #file.write("tentative_gScor: " + str(tentative_gScore) + "\n")
                if (neighbor in gScore) and (tentative_gScore >= gScore[neighbor]) :
                    #file.write("Not better" + "\n")
                    continue        # This is not a better path.
                #file.write("Better" + "\n")
                # This path is the best until now. Record it!
                cameFrom[neighbor] = current
                gScore[neighbor] = tentative_gScore
                fScore[neighbor] = gScore[neighbor] + data["distToGoal"] 
                #file.write("cameFrom: " + str(neighbor) + ", to: " + str(current) + "\n")
                #file.write("gScore: " + str(tentative_gScore) + "\n")
                #file.write("fScore: " + str(fScore[neighbor]) + "\n")

        file.close()
        return (False, [])

    
    def reconstruct_path(self, cameFrom, current) :
        global pathMap
        totalPath = [current]
        navPath = [("end", current)]
        lastKeyNode = current
        #navPath.append(("moveTo", current))
        while current in cameFrom.keys() and not enduserloop:
            previous = current
            current = cameFrom[current]
            if current[2] != previous[2] : # if the angle has changed
                # Append the end of the straight line, the turn and the start of the next
                totalPath.append(previous)
                totalPath.append((previous[0],previous[1],current[2]))
                totalPath.append(current)
                
                #navPath.append(("moveStraight", getLength([lastKeyNode[0],lastKeyNode[1], previous[0],previous[1]])))
                navPath.append(("moveTo", lastKeyNode))
                #navPath.append(("turn", constrainAngle360((previous[2]-current[2])*45,180,-180)))
                navPath.append(("turnTo", previous))
                lastKeyNode = current
        navPath.append(("start", current))
        return list(reversed(navPath))
    
    
    # Uses A* and navigation algorithms to navigate to the given dest = [x, y]
    def goTo(self, dest, timeout, goPartial) :
        global porterLocation
        global speedVector
        global pathMap
        global lidarRun
        f=open("nav.log", "w")    
        while not enduserloop :
            success, path = self.aStar((porterLocation[0],porterLocation[1],0), dest, timeout)
            if not (success or goPartial) :
                with threadLock :
                    speedVector = [0,0]
                return False
            f.write(str(path) +"\n")
            try :
                if len(path) > 1 :
                    for instruction in path :
                        success = True
                        if enduserloop :
                            break
                        if instruction[0] == "turn" :
                            with threadLock :
                                lidarRun = "s"
                                
                            success = self.moveTurn(instruction[1], "onCentre", f)
                            
                            with threadLock :
                                lidarRun = "a"
                        elif instruction[0] == "moveStraight" :
                            success = self.moveStraight(instruction[1], f)
                        elif instruction[0] == "moveTo" :
                            success = self.moveTo(instruction[1], f)
                        elif instruction[0] == "turnTo" :
                            with threadLock :
                                lidarRun = "s"
                            success = self.turnTo(instruction[1], "onCentre", f)
                            with threadLock :
                                lidarRun = "a"
                        
                        if not success :
                            break
                        
            except TypeError :
                print("No path found")
                with threadLock :
                    speedVector = [0,0]
                return False
        with threadLock :
            speedVector = [0,0]
        f.close()
    
    
class mappingClass(controlClass):
    
    def getScanLocations(self) :
        global SLAMObject
        # Map size, scale
        MAP_SIZE_METERS          = 25 # 32
        MAP_SIZE_PIXELS          = MAP_SIZE_METERS * 50 #800
        # Make array for SLAM map
        mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)
        
        # Get map from SLAM
        SLAMObject.getmap(mapbytes)
        
        # Convert map into numpy array
        numpyMap = np.frombuffer(mapbytes, dtype='B')
        numpyMap.shape =  (-1,MAP_SIZE_PIXELS)
        cv2.imwrite('numpyMapRaw.jpg', numpyMap)
        
        # Theshhold low to find the walls then dilate to expand the walls - used as mask to avoid scanning walls
        a, numpyMapThreshLow  = cv2.threshold(numpyMap,126,255,cv2.THRESH_BINARY) 
        # Make kernal for dilation
        kernel = np.ones((pathMap.dilateAmount,pathMap.dilateAmount),np.uint8)
        numpyMapThreshLow = cv2.dilate(cv2.bitwise_not(numpyMapThreshLow), kernel)
        cv2.imwrite('numpyMapThreshLow.jpg', numpyMapThreshLow)
        
        # Smooth image, threshold high to find clear areas, dilate to smooth lines, canny to find edges of clear area, dilate to smooth
        numpyMapThreshHigh = cv2.GaussianBlur(numpyMap, (5, 5), 2)
        cv2.imwrite('numpyMapBlur.jpg', numpyMapThreshHigh)
        b, numpyMapThreshHigh = cv2.threshold(numpyMapThreshHigh,128,255,cv2.THRESH_BINARY)
        numpyMapThreshHigh = cv2.dilate(numpyMapThreshHigh, np.ones((2, 2)))
        numpyMapThreshHigh = cv2.Canny(numpyMapThreshHigh,100,200)
        numpyMapThreshHigh = cv2.dilate(numpyMapThreshHigh, np.ones((5, 5)))
        cv2.imwrite('numpyMapThreshHigh.jpg', numpyMapThreshHigh)
        
        # Use walls as a mask over clear area edges
        numpyMapThreshBand = numpyMapThreshHigh - numpyMapThreshLow
        cv2.imwrite('numpyMapThreshBand.jpg', numpyMapThreshBand)
        
        # Setup SimpleBlobDetector parameters.
        params = cv2.SimpleBlobDetector_Params()
        params.blobColor = 255
        params.minThreshold = 10;
        params.maxThreshold = 255;
        params.filterByArea = False
        params.filterByCircularity = False
        params.filterByConvexity = False
        params.filterByInertia = False
        # Set up the detector
        detector = cv2.SimpleBlobDetector_create(params)
        # Detect blobs.
        keypoints = detector.detect(numpyMapThreshBand)        
        keyPointTuples = []
        for keyPoint in keypoints  :
            keyPointTuples.append(keyPoint.pt)
            
        return keyPointTuples
    
    
    def sortLocations(self, locations) :
        distances = []
        for location in locations :
            dist = (location[0] - porterLocation[0])**2 + (location[1] - porterLocation[1])**2
            distances.append(dist)
        
        return [loc for _, loc in sorted(zip(distances,locations), key=lambda pair: pair[0])]

   
    def run(self):   
        # May not need all of these
        global USAvgDistances
        global wheelSpeeds
        global obstruction
        global maxSpeeds
        global porterLocation #vector holding local location [x,y] in cm
        global porterOrientation #angle from north (between -180,180) in degrees
        global porterOrientationUnConst
        global realPorterLocation # CHEATING ONLY
        global realPorterOrientation # CHEATING ONLY
        global realPorterSize
        global lastCommand
        global speedVector
        global dataReady
        global threadLock
        global enduserloop
        global USThreashholds
        global stoppingDistance
        global lidarReady
        global lidarMap
        global lidarAngles
        global lidarRun    
        global pathMap
        global realPorterSize
        global dataMap
        global realPorterRadius
        global totalR
        global totalWd
        global SLAMObject
        global SLAMRunning
                    
        logging.info("Starting auto mapping...")
        with threadLock :
            SLAMRun = "start"
            lidarRun = "a"
        
        time.sleep(5)
        
        locations = self.getScanLocations()
        for loc in locations :
            dataMap.add(loc)
        while len(locations) > 1  and not enduserloop:
            locations = self.sortLocations(locations)
            success = self.goTo(locations[0], 20, True)
            # REVISIT : If not success - blacklist the area?
            time.sleep(2)
            locations = self.getScanLocations()
           
        logging.info("Done auto mapping...") 

    
class navigationClass(controlClass):
    def run(self, destination) :
        global threadLock
        global SLAMRun
        global lidarRun
        logging.info("Starting navigation...")
        with threadLock :
            SLAMRun = "start"
            lidarRun = "a"
        
        time.sleep(5)

        self.goTo(destination, 20, False)
        
        logging.info("Done navigation")
        
        


    
#Retrieve map from database
def recieve_map(filename):
    query = "SELECT data FROM `%u` ORDER BY id DESC LIMIT 1" %filename
    cursor.execute(query)
    result = cursor.fetchone() ; 
    img_string = base64.b64decode(result[0])
    nparray = np.fromstring(img_string,np.uint8) ; 
    image = cv2.imdecode(nparray, cv2.CV_LOAD_IMAGE_GRAYSCALE)
    return image


######################################################
######################################################
######Start of the Main Thread!

if __name__ == '__main__':
    # Say Hello
    print("Hello")
    
    #Connect to map database
    connection = MySQLdb.connect(host='192.168.0.1',db='roboporter',user='admin',passwd='password', port=3306)
    cursor = connection.cursor()

    logging.info("Starting system...")
    sysRunning = True

    #Try to strat thread to recieve data from UI
    try:
        logging.info("Trying to Run Recieve Thread")
        recieveChannel = datafromUI(6, "Recieve Data ")
        #recieveChannel.daemon = True
        recieveChannel.start()
        
        threads.append(recieveChannel)
        logging.info('Recieve Thread Running - %s', str(recieveChannel))
        logging.info("Running Recieve Server")
        #time.sleep(1)

    except Exception as e:
        logging.error("%s", str(e))


    #Try to start thread to send data from UI
    try:
        logging.info("Trying to Run Debug Server")
        debugChannel = debugThread(3, "Debug Thread")
        #debugChannel.daemon = True
        debugChannel.start()
        
        threads.append(debugChannel)
        logging.info('Debug Thread Running - %s', str(debugChannel))
        logging.info("Running Debug Server")
        #time.sleep(1)

    except Exception as e:
        logging.error("%s", str(e))

    # setup serial connection to motor controller
    logging.info("Trying to connect to serial devices")

    if Motor_Enable: 
        logging.info("Trying to connect to motor controller")
        try: #try to connect
            if (platform == "linux") or (platform == "linux2"):
                MotorConn = serial.Serial('/dev/ttyACM1', 19200,timeout=5)
            elif (platform == "win32"):
                MotorConn = serial.Serial('COM7', 19200)

            logging.info('Connected to Motors %s', str(MotorConn))
            serialThread = motorDataThread(14, "Motor thread")
            #serialThread.daemon = True
            serialThread.start()
            
            threads.append(serialThread)
            logging.info('Motor Thread Running - %s', str(serialThread))

        except Exception as e: #if something goes wrong... tell the user
            logging.error("Unable to establish serial comms to port /dev/ttyACM0")
            logging.error("%s", str(e))

    # Setup serial conn to US controller
    if US_Enable: 
        logging.info("Trying to connect to Ultrasonic controller")
        try:
            if (platform == "linux") or (platform == "linux2"):
                USConn1 = serial.Serial('/dev/ttyACM2', 9600)
                logging.info("Connected to Ultrasonic sensors at %s", str(USConn1))
                if UShosts == 2:
                    USConn2 = serial.Serial('/dev/ttyACM3', 9600)
                    logging.info("Connected to Ultrasonic sensors at %s", str(USConn2))

            elif (platform == "win32"):
                USConn1 = serial.Serial('COM3', 9600)

            USthread = usDataThread(5, "Ultrasonic thread")
            #USthread.daemon = True
            USthread.start()
            
            threads.append(USthread)
            logging.info('Ultrasonic Thread Running - %s', str(USthread))
            
        except Exception as e:
            print ('Unable to establish serial comms to US device')
            logging.error("%s", str(e))
            # USConnected = False
            
            
    # Setup serial conn to LidarInterface
    if Lidar_Enable: 
        logging.info("Trying to connect to LidarInterface")
        try: #try to connect
            if (platform == "linux") or (platform == "linux2"):
                LIDARConn = serial.Serial('/dev/ttyACM0', 2000000,timeout=5)
            elif (platform == "win32"):
                LIDARConn = serial.Serial('COM5', 2000000)

            logging.info('Connected to Lidar %s', str(LIDARConn))
            lidarThread = lidarInterfaceThread(7, "Lidar thread")
            lidarThread.start()
            
            threads.append(lidarThread)
            logging.info('Lidar Thread Running - %s', str(lidarThread))

        except Exception as e: #if something goes wrong... tell the user
            logging.error("Unable to establish lidar comms to port /dev/ttyACM3")
            logging.error("%s", str(e))

    if SLAM_Enable:
        slamthread1 = SLAMThread(8, "SLAM thread")
        slamthread1.start()
        threads.append(slamthread1)
        logging.info('SLAM Thread Running - %s', str(slamthread1))

    # Create pathMap and SLAMObject
    try:
        pathMapTemp = pathMapClass(realPorterSize[0])
        with threadLock :
            pathMap = pathMapTemp
            SLAMObject = RMHC_SLAM(roboPorterLaser(), MAP_SIZE_PIXELS, MAP_SIZE_METERS, random_seed=0)
    except:
        pass 
    # Main Control Loop
    while sysRunning: #while the main loop is not in shutdown mode...
        time.sleep(1)
        currentcommand = commandqueue.get()
        if currentcommand["Type"] == "UserCommand": 
            logging.info("Entering User Command Mode")
            with threadLock:
                system_status = "UserCommand"
                enduserloop = False
                runSLAM = "start"
                lidarRun = "a"
                time.sleep(1)
            with commandqueue.mutex:
                speedsqueue.queue.clear()
            while not enduserloop:
                currentspeed = speedsqueue.get()
                with threadLock:
                    speedVector[0] = int(currentspeed["Left"])
                    speedVector[1] = int(currentspeed["Right"])
            with threadLock:
                runSLAM = "stop" # Stop slap and lidar
                lidarRun = "s"
                system_status == "AwaitingCommands"


        elif currentcommand["Type"] == "MappingCommand":
            logging.info("Entering Mapping Mode")
            with threadLock:
                system_status = "Mapping"
                enduserloop = False

            #Mapping Code Goes Here
            mappingObj = mappingClass(2)
            mappingObj.run()
            with threadLock:
                system_status = "AwaitingCommands"


        elif currentcommand["Type"] == "NavigationCommand":
            logging.info("Entering Navigation Mode")
            try :
                loadMapTemp = recieve_map(currentcommand["Map_Filename"])   # REVISIT : Load in map
                with threadLock :
                    loadMap = loadMapTemp
                    runSLAM = "loadMap"
            except KeyError :
                pass
            
            with threadLock:
                system_status = "Navigation"
                enduserloop = False

            #Navigation Code Goes Here
            navigationObj = navigationClass(2)
            navigationObj.run((currentcommand["X"], currentcommand["Y"]))

            with threadLock:
                system_status = "AwaitingCommands"
        else:
            pass

    with threadLock:
        speedVector = [0, 0]
        exitFlag = True #instruct all the threads to close

    logging.info("Waiting for threads to close...")
    for t in threads:
        logging.info("Closing %s thread", t)
        t.join()

    # Closing serial connections
    USConn1.close()
    USConn2.close()
    MotorConn.close()
    LIDARConn.close()
    logging.info("Exiting main Thread... BYEEEE")

#######END of Program