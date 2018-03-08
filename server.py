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
import datetime
import glob
import json
import math
import multiprocessing
import numpy as np
import pickle
import pyttsx
import random 
import serial
import socket
import struct
import sys
import threading
import time

#import zbar
#import time
from PIL import Image

try:
    import zbar
except Exception as e:
    print str(e)

#from vpLib import *

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

try:
    import vpLib
except Exception as e:
     logging.error("Cant import vpLib")

###---Global Variables-------------

##--Setup

US_Enable = True
Motor_Enable = True
Cam_Enable = True
Speech_Enable = True
Debug_Enable = True

##--Motor Commands
global lastCommand #holds the last command that was sent to the motor controllers
global speedVector #demanded wheel speeds (left, right)
global dataReady #boolean to let the threads know that data is ready to be sent to the motors
global wheelSpeeds #measured wheel speeds


lastCommand = ""
speedVector = [0, 0]
setSpeedVector = [0,0]


dataReady = False


##-- MYSQL DB variables
global connection
global cursor

##--Serial Connections
global MotorConn #serial handle for the motor controller
global USConn1 #serial handle for the Ultrasonic controller 1
global USConn2 #serial handle for the Ultrasonic controller 2

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
USAvgDistances = [0, 0, 0, 0, 0, 0, 0, 0]
obstruction = False
usCaution = False
USThresholds = [30, 20, 30] #threasholds for treating objects as obstacles [front,side,back]
#make sure stopping distance is > usThresholds
stoppingDistance = 15
UShosts = 2
maxSpeeds = [0, 0, 0, 0, 0, 0, 0, 0]

##--Multi-Threading/Muti-processing
global threadLock #lock to be used when changing global variables
threadLock = threading.Lock()
threads = [] #Array holding information on the currently running threads
processes = [] #Array holding information on the currently running Processes


##--Auto Pilot
global autoPilot #Boolean for turning on/off autopilot
autoPilot = False #autopilot turned off
global pidEnable
pidEnable = False
global avoidingObstacle
avoidingObstacle = False
global AHRSmode
AHRSmode = "wheel"

# Command Queue

global commandqueue 
commandqueue = Queue.Queue()

global speedsqueue 
speedsqueue = Queue.Queue()

# -Porter Localisation
global porterLocation_Global #vector holding global location [x,y] in cm
global porterLocation_Local #vector holding local location [x,y] in cm
global porterOrientation #angle from north (between -180,180) in degrees
global targetDestination  #Target location in global coordinates in cm
global distanceToGoal #Distance to goal in cm
global porterLocation_IMU
global roaming
global localised
localised = multiprocessing.Value("b", True)
roaming = multiprocessing.Value("b", False)
targetDestination = [0,0]
#porterLocation_Global = [3250,3500]
porterOrientation = multiprocessing.Value('d',0.0)  # from north heading (in degrees?)
distanceToGoal = 0
orientationIMU = multiprocessing.Value("d", 0.0)
orientationWheels = multiprocessing.Value("d", 0.0)

##-System State Variables
localCtrl = True #Local control = commands sent through SSH not TCP/IP
sysRunning = False #
cmdExpecting = False #
#exitFlag = False #set exitFlag = True initiate system shutdown
dataInput = "" #Data string from the user (SSH/TCP)
exitFlag = multiprocessing.Value('b', False) #multiprocessing Exit flag (can be combined with the others)
global motor_portAddr
global US1_portAddr
global US2_portAddr
motor_portAddr = ""
US1_portAddr = ""
US2_portAddr = ""
global h_scores
h_scores = [0.,0.,0.,0.] 
global system_status
system_status = "AwaitingCommands"
global motordata
motordata = 0 

enduserloop = False

#lastSent = [0, 0]

def setExitFlag(status):
    global exitFlag
    global exitFlag

    exitFlag = status
    exitFlag.value = status

###---Class Definitions

class MultiThreadBase(threading.Thread): #Parent class for threading
    def __init__(self, threadID, name): #Class constructor
        threading.Thread.__init__(self) #Initiate thread
        self.threadID = threadID #Thread number
        self.name = name #Readable thread name

        #thread loop performance profiling
        self.avgRuntime = 0. #loop average runtime
        self.startTime = 0. #loop start time
        self.endTime = 0. #loop end time
        self.avgCounter = 0 #number of loops
        self.loopProfilingInterval = 10 #loop averaging interval
        self.profiling = False #profiling state. set true in to calculate run times globally.
            #if you want to profile a single thread, change this in the child thread
            #Note - the system will run much slower when profiling
            #STILL NOT FULLY IMPLEMENTED. DO NOT SET TO TRUE

    def loopStartFlag(self): #run at the start of the loop
        self.startTime = time.localtime() #record the start time of the loop

    def loopEndFlag(self):
        self.startTime = time.localtime()

    def loopRunTime(self):
        self.avgRuntime += ((self.endTime - self.startTime) - self.avgRuntime) / self.loopProfilingInterval
        self.avgCounter += 1

        if self.avgCounter == self.loopProfilingInterval:
            # print (self.name + " Avg loop Runtime - " + str(self.avgRuntime))
            logging.debug("Avg loop Runtime - %s", str(self.avgRuntime))
            self.avgCounter = 0

        score = []

        for i in range(0, 4):
            if distScore[i] != 0:
                score.append(self.alpha * envScore[i] + self.beta / distScore[i])
            else:
                score.append(self.alpha * envScore[i] + 1)

        #NO MOMENTUM BONUS FOR NOW
        # if len(self.pathmemory) != 0:
        #     print("momentum bonus in the ", self.pathmemory[len(self.pathmemory) - 1], "direction")
        #     score[self.pathmemory[len(self.pathmemory) - 1]] = score[self.pathmemory[
        #         len(self.pathmemory) - 1]] + self.momentumBonus
        # else:
        #     print('start of run, no momentum bonus')

        self.bestScoreIndex = score.index(max(score))

        print("heuristic score is " + str(score))
        print("max heuristic score is " + str(max(score)) + " and its at " + str( self.bestScoreIndex))

        return score

class debugThread(MultiThreadBase):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.avgRuntime = 0.
        self.startTime = 0.
        self.endTime = 0.
        self.avgCounter = 0
        self.loopProfilingInterval = 10
        self.profiling = False
        self.loopsdone = 0
        self.debugServer = False
        self.debugClient = False
        self.dataStore = ""

    def run(self):
        logging.info("Starting %s", self.name)
        while not exitFlag.value:#
            debuginfo = 0 
            self.loopStartFlag()
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
                        'Battery': str("50")
                        # 'Left Pulses' : str(motordata[1]),
                        # 'Right Pulses' : str(motordata[2]),
                        # 'Battery Current 1' : str(motordata[3])
                    }
                except:
                    pass
                try:
                    datatosend = json.dumps(debuginfo)
                    self.clientConnection.send(datatosend)
                    self.loopsdone += 1 #increment loops done by 1
                    time.sleep(0.5)

                except Exception as e:
                    logging.error("%s", str(e))
                    self.debugClient = False
                    # individual thread states
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

    def logOverNetwork(self):
        rootLogger = logging.getLogger('')
        rootLogger.setLevel(logging.DEBUG)
        socketHandler = logging.handlers.SocketHandler('localhost',
                                                       logging.handlers.DEFAULT_TCP_LOGGING_PORT)
        # # don't bother with a formatter, since a socket handler sends the event as
        # # an unformatted pickle
        rootLogger.addHandler(socketHandler)
        #
        # # Now, we can log to the root logger, or any other logger. First the root...
        # logging.info('Jackdaws love my big sphinx of quartz.')
        #
        # # Now, define a couple of other loggers which might represent areas in your
        # # application:
        #
        # logger1 = logging.getLogger('myapp.area1')
        # logger2 = logging.getLogger('myapp.area2')
        #
        # logger1.debug('Quick zephyrs blow, vexing daft Jim.')
        # logger1.info('How quickly daft jumping zebras vex.')
        # logger2.warning('Jail zesty vixen who grabbed pay from quack.')
        # logger2.error('The five boxing wizards jump quickly.')

class motorDataThread(MultiThreadBase):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.actionState = 0
        self.profiling = False
        self.oldVector = [0, 0]
        self.hexData = True
        self.lastRandomCode = "R"
        self.inputBuf = ""
        self.pulses = ["",""]
        self.obs = False
        self.smoothBrake = True
         
    def run(self):
        global speedVector
        global threadLock
        global dataReady
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
        
        while not exitFlag.value:
            self.loopStartFlag()
         
            if obstruction:
                # if autoPilot and self.obs:
                #     if lastSent != [0, 0]:  # ie still moving
                #         # pause motion
                #         logging.info("Obstacle Detected. Path needs to be recalculated")
                #        
                #         self.send_serial_data([0, 0])
                #     # else:
                #     #     while obstruction:
                #     #         time.sleep(0.1)
                #     #     
                #     #     logging.info("Obstacle removed. Proceeding to destination")
                #     #     self.send_serial_data(speedVector)
                # elif autoPilot:
                #     logging.info("autopilot command to motor")
                #     if lastSent != [0, 0]:  # ie still moving
                #         # pause motion
                #         logging.info("Obstacle Detected. Waiting for it to go away")
                #         self.send_serial_data([0, 0])
                #     else:
                #         while obstruction and autoPilot and not exitFlag.value:
                #             time.sleep(0.1)
                #         if not obstruction and autoPilot and not exitFlag.value:
                #             
                #             logging.info("Obstacle removed. Proceeding to destination")
                #             self.send_serial_data(speedVector)
                if safetyOn: #if not autopilot and the safety is on
                    if lastSent != [0, 0]:
                        logging.debug("Setting Speed Vector")
                        with threadLock:
                            speedVector = [0, 0]
                            dataReady = False
                        self.send_serial_data(speedVector)
                elif lastSent != speedVector: #otherwise...
                    logging.warning("Obstacle Detected But Safety OFF...") #give a warning, but dont do anything to stop
                    self.send_serial_data(speedVector)

            elif (speedVector != lastSent) or (last_time_sent >= 1):
                last_time_sent = 0 
                # (self.last_time_sent >= self.time_between_send)
                print "Sending Data"
                logging.debug("Data Ready")
                logging.info("Data Ready")
                try:
                    if not safetyOn:
                        try:
                            logging.info("Trying to send data no safety")
                            self.send_serial_data(speedVector)
                        except Exception as e:
                            logging.error("%s", str(e))
                            if MotorConn.closed:
                                try:
                                    logging.info("Trying to open serial port")
                                    MotorConn.open()
                                except Exception as e:
                                    logging.error("%s", str(e))
                                finally:
                                    logging.info("No Motor Comms... Looping back to listening mode")
                        finally:
                            if dataReady != False:
                                with threadLock:
                                    dataReady = False
                    elif (safetyOn and not USConn1.closed):
                        try:
                            logging.info("Trying to send data")
                            self.send_serial_data(speedVector)
                        except Exception as e:
                            logging.error("%s", str(e))
                            if MotorConn.closed:
                                try:
                                    logging.info("Trying to open serial port")
                                    MotorConn.open()
                                except Exception as e:
                                    logging.error("%s", str(e))
                                finally:
                                    logging.info("No Motor Comms... Looping back to listening mode")
                        finally:
                            if dataReady != False:
                                with threadLock:
                                    dataReady = False
                    else:
                        logging.info("No ultrasonic Comms... HALTING")
                        if speedVector != [0, 0]:
                            logging.debug("Reseting Speed Vector")
                            with threadLock:
                                speedVector = [0, 0]
                                dataReady = False
                        self.send_serial_data(speedVector)
                except Exception as e:
                    logging.error("%s", str(e))
            
            #READ FROM ARDUINO?
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
                    elif self.inputBuf[0] == "%":
                        try:
                            self.inputBuf = self.inputBuf.lstrip("%") 
                            print self.inputBuf
                        except Exception as e:
                            logging.error("%s", str(e))
                except Exception as e:
                    logging.error("%s", str(e))
            time.sleep(0.001)
            last_time_sent +=0.001
        logging.info("Exiting")

    def send_serial_data(self, sendCommand):
        global lastSent
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
            lastSent = sendCommand[:]
            # time.sleep(10)
            # print MotorConn.read(MotorConn.inWaiting())
        except Exception as e:
            logging.error("Sending to Motor failed - %s", str(e))
            # self.actionState = 4

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
        self.profiling = False

        self.usHistory = 0
        self.medFiltSize = 5

    def getUSvector1(self):
        while not exitFlag.value:
            try:
                self.inputBuf1 = USConn1.readline()
                self.inputBuf1 = self.inputBuf1.rstrip(",\n")
                self.rawUSdata_1 = self.inputBuf1.split(",")
                #print ("rw1 ", self.inputBuf1)
            except Exception as e:
                logging.error("%s", e)

    def getUSvector2(self):
        while not exitFlag.value:
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

        #aMax = (2*np.pi*1000)/(60*0.12)
        aMax = 872.664626 # Calculated from line above which was supplied by a previous group 
        #radToRPM = 60/(2*np.pi)
        for i in range(0,len(usData)):
            if int(usData[i]) >= stoppingDistance:
                maxSpeeds[i] = int(np.sqrt(aMax*(((usData[i]/100) -(stoppingDistance/100)))))
            else:
                maxSpeeds[i] = 0
        #print "max speeds = " + str(maxSpeeds)

    def run(self):
        global speedVector
        logging.info("Starting")
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

        while not exitFlag.value:
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

            if self.profiling:
                time.sleep(0.1)
                self.loopEndFlag()
                self.loopRunTime()
            else:
                time.sleep(0.01) # change to vary frequency of US data interupts 

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
                if self.profiling:
                    print ("\t" + self.name + " Avg Vector - " + str(USAvgDistances))
          
        #print USAvgDistances
        #logging.info(USAvgDistances) 

    def US_safety_interrupt(self):
        global USAvgDistances
        global lastCommand
        global dataReady
        global obstruction
        global threadLock
        global usCaution

        if safetyOn:
            try:
                if (speedVector[0]> 0 ) and (speedVector[1] > 0):
                    if (int(USAvgDistances[2]) < USThresholds[0]) or (int(USAvgDistances[3]) < USThresholds[0]):
                        logging.warning("FRONT TOO CLOSE. STOPPPPP!!!")
                        if obstruction != True:
                            with threadLock:
                                obstruction = True
                        lastCommand = "" 
                        print ("\t" + self.name + " Avg Vector - " + str(int(USAvgDistances[2])) + ", " + str(int(USAvgDistances[2])))
                    elif obstruction != False:
                        with threadLock:
                            obstruction = False

                    if (maxSpeeds[2] > abs(speedVector[0]) or (maxSpeeds[3] > abs(speedVector[0]))):
                        # print("CAUTION\n")
                        usCaution = True
                    else:
                        usCaution = False

                elif (speedVector[0] < 0) and (speedVector[1] < 0 ):
                    if (int(USAvgDistances[6]) < USThresholds[2]) or (int(USAvgDistances[7]) < USThresholds[2]):
                        logging.warning("BACK TOO CLOSE. STOPPPPP!!!")
                        if obstruction != True:
                            with threadLock:
                                obstruction = True
                        lastCommand = "" 
                        print ("\t" + self.name + " Avg Vector - " + str(int(USAvgDistances[6])) + ", " + str(int(USAvgDistances[7])) )
                    elif obstruction != False:
                        with threadLock:
                            obstruction = False

                    if (maxSpeeds[6] > abs(speedVector[0]) or (maxSpeeds[7] > abs(speedVector[0]))):
                        usCaution = True
                        # print("CAUTION\n")
                    else:
                        usCaution = False

                elif (speedVector[0]< 0 ) and (speedVector[1] > 0):
                    if (int(USAvgDistances[0]) < USThresholds[1]) or (int(USAvgDistances[1]) < USThresholds[1]):
                        logging.warning("LEFT SIDE TOO CLOSE. STOPPPPP!!!")
                        if obstruction != True:
                            with threadLock:
                                obstruction = True
                        lastCommand = "" 
                        print ("\t" + self.name + " Avg Vector - " + str(int(USAvgDistances[0])) + ", " + str(int(USAvgDistances[1])))
                    elif obstruction != False:
                        with threadLock:
                            obstruction = False

                elif (speedVector[0] > 0) and (speedVector[1] < 0) :
                    if (int(USAvgDistances[5]) < USThresholds[1]) or (int(USAvgDistances[6]) < USThresholds[1]):
                        logging.warning("RIGHT SIDE TOO CLOSE. STOPPPPP!!!")
                        if obstruction != True:
                            with threadLock:
                                obstruction = True
                        lastCommand = "" 
                        print ("\t" + self.name + " Avg Vector - " + str(int(USAvgDistances[5])) + ", " + str(int(USAvgDistances[6])))
                    elif obstruction != False:
                        with threadLock:
                            obstruction = False
                
                elif lastCommand == "x":
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
        self.actionState = 0
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
            dataInput = self.clientConnection.recv(1024)
            print ("Client says - " + dataInput)
            dataInput = ""
            self.clientconnect = True
        except Exception as e:
            logging.error("%s", str(e))

    def run(self):
        global lastCommand
        global sysRunning
        global safetyOn
        global exitFlag
        global threadLock
        global enduserloop
        logging.info("Starting %s", self.name)
        while not exitFlag.value:
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
                                lastCommand = "x"
                            sysRunning = False
                            commandqueue.put("Close")
                        elif dataInput['Command'] == "s":
                            with threadLock:
                                safetyOn = not safetyOn ; 
                    elif dataInput['Type'] == "Cancel": # If the user wants to cancel the mapping or nav command and move onto the next item in the queue this is ran
                        with threadLock:
                            enduserloop = True

                        # add code to end loops


                    elif dataInput['Type'] == "CancelEnterUserCommand": # When the user wants to enter a command immediatly it clears the queue. This is called straight before a switch to the User Control mode occurs
                        with commandqueue.mutex:
                            commandqueue.queue.clear()

                        # add code to end loops

                    elif(dataInput['Type'] == "UserSpeed"): # If the user sent a speed command it is added to the speed queue
                        print "UserSpeed"
                        speedsqueue.put(dataInput)
                    else:
                        commandqueue.put(dataInput)
                except Exception as e:
                    logging.error("%s", str(e))

                          
                    # dataInput = self.clientConnection.recv(1024)
                    # if (dataInput[0] == "f" or dataInput[0] == "b" or dataInput[0] == "r" \
                    #             or dataInput[0] == "l" or dataInput[0] == "x" or dataInput[0] == "m") and not autoPilot: #simple commands
                    #     logging.info("Valid Command")
                    #     system_status = "Manual Control"
                    #     with threadLock:
                    #         lastCommand = dataInput[0]
                    #         speedVector = cmdToSpeeds(dataInput)
                    #         setSpeedVector = copy.deepcopy(speedVector)
                    #         dataReady = True

                    #         if lastCommand == "m":
                    #             logging.info("MANUAL OVERRIDE!\n")

                    # elif dataInput[0] == "s" and not autoPilot: #Toggle safety
                    #     if safetyOn:
                    #         safetyOn = False
                    #     else:#if safety is off...turn it on
                    #         safetyOn = True
                    #     system_status = "Manual Control"
                    #     dataInput = ""
                    #     if dataReady != False:
                    #         with threadLock:
                    #             dataReady = False
                    
                    # elif dataInput == "q": #initiate system shutdown
                    #     cmdExpecting = False
                    #     sysRunning = False
                    # else:
                    #     if dataReady != False:
                    #         with threadLock:
                    #             dataReady = False
                    #     logging.info("Invalid Command")    
                # except:
                #     print "Error" 

def cmdToSpeeds(inputCommand): #convert commands to speed vectors for manual control
    mSpeed = 0
    validateBuffer = (0, 0)
    LMax = 100
    RMax = 100
    LMin = -100
    RMin = -100

    if inputCommand[0] == "x":
        return 0, 0
    elif inputCommand[0] == "m":
        inputCommand.rstrip("\n")
        validateBuffer = str(inputCommand[1:len(inputCommand)]).split(",")
        validateBuffer[0] = int(validateBuffer[0])
        validateBuffer[1] = int(validateBuffer[1])

        if validateBuffer[0] > LMax:
            validateBuffer[0] = LMax
        elif validateBuffer[0] < LMin:
            validateBuffer[0] = LMin

        if validateBuffer[1] > RMax:
            validateBuffer[1] = RMax
        elif validateBuffer[1] < RMin:
            validateBuffer[1] = RMin

        return validateBuffer

    if len(inputCommand) > 1:
        try:
            inputCommand.rstrip("\n")
            mSpeed = int(inputCommand[1:len(inputCommand)])
        except Exception as e:
            logging.error("%s", e)
            mSpeed = 0
    else:
        mSpeed = 20

    if inputCommand[0] == "f":
        return mSpeed, mSpeed  # Left, Right
    elif inputCommand[0] == "b":
        return -mSpeed, -mSpeed
        logging.info("Converted to Speed")
    elif inputCommand[0] == "r":
        return (mSpeed-5), -(mSpeed-5)
    elif inputCommand[0] == "l":
        return -(mSpeed-5), (mSpeed-5)

def cmdToDestination(inputCommand):
    #validateBuffer = [0,0]
    print "input command is " + str(inputCommand)
    if len(inputCommand) > 4:
        try:
            inputCommand.rstrip("\n")
            validateBuffer = str(inputCommand[1:len(inputCommand)]).split(",")
            print "valdate Buffer is " + str(validateBuffer)
            return int(validateBuffer[0]), int(validateBuffer[1])
        except Exception as e:
            logging.error("%s", e)
            return porterLocation_Global[0],porterLocation_Global[1]
    else:
        logging.error("Invalid destination format")
        return porterLocation_Global[0],porterLocation_Global[1]

# Sends Maps to the temporary table in the database
def send_map(map_to_send):
    img_string = cv2.imencode('.jpg',map_to_send)[1].tostring()
    img_string = base64.b64encode(img_string)
    query = "INSERT INTO temp (data) VALUES ('%s')" %img_string
    cursor.execute(query)

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

    multiprocessing.freeze_support()
    mpManager = multiprocessing.Manager()
    wheelSpeeds = mpManager.list([0, 0])
    porterLocation_Global = mpManager.list([3250, 4600])  # set location to "undefined"
    porterLocation_Local = mpManager.list([0, 0])
    porterLocation_IMU = mpManager.list([0,0])

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
        dataInput = "" #Reset the variable
        logging.info("Trying to connect to motor controller")
        try: #try to connect
            if (platform == "linux") or (platform == "linux2"):
                MotorConn = serial.Serial('/dev/ttyACM0', 19200,timeout=5)
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
        dataInput = ""
        logging.info("Trying to connect to Ultrasonic controller")
        try:
            if (platform == "linux") or (platform == "linux2"):
                USConn1 = serial.Serial('/dev/ttyACM1', 9600)
                logging.info("Connected to Ultrasonic sensors at %s", str(USConn1))
                if UShosts == 2:
                    USConn2 = serial.Serial('/dev/ttyACM2', 9600)
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


    # Main Control Loop
    while sysRunning: #while the main loop is not in shutdown mode...
        time.sleep(1)
        currentcommand = commandqueue.get()
        if currentcommand["Type"] == "UserCommand": 
            with threadLock:
                system_status = "UserCommand"
            with commandqueue.mutex:
                speedsqueue.queue.clear()
            with threadLock:
                enduserloop = False
            while not enduserloop:

                # Add new user code here and delete code below

                currentspeed = speedsqueue.get()
                with threadLock:
                    speedVector[0] = int(currentspeed["Left"])
                    speedVector[1] = int(currentspeed["Right"])

                # Delete code above and replace with any new usermode code

            with threadLock:
                system_status == "AwaitingCommands"
        elif currentcommand["Type"] == "MappingCommand":
            print "Running a Mapping Command"
            with threadLock:
                system_status = "Mapping"

            #Mapping Code Goes Here

            with threadLock:
                system_status = "AwaitingCommands"
        elif currentcommand["Type"] == "NavigationCommand":
            recieve_map(currentcommand["Map_Filename"])
            print "Running a Navigation Command"
            with threadLock:
                system_status = "Navigation"

            #Navigation Code Goes Here

            with threadLock:
                system_status = "AwaitingCommands"
        else:
            pass

    with threadLock:
        lastCommand = "x"
        speedVector = [0, 0]
        dataReady = True

    #exitFlag.value = True #instruct all the threads to close

    # if not localCtrl:
    #     logging.info("Shutting down the server at %s...", HOST)
    #     s.close()
    # else:
    #     logging.info("Shutting Down")

    # logging.info("Waiting for threads to close...")
    # for t in threads:
    #     logging.info("Closing %s thread", t)
    #     t.join()

    # for p in processes:
    #     logging.info("Closing %s process", p)
    #     p.join()

    # print "processes = " + str(processes)
    # print "Threads = " + str(threads)

    logging.info("Exiting main Thread... BYEEEE")

#######END of Program