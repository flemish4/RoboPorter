#
# Module name - R4_server
# Module Description -

# Author     - C. Samarakoon
# Created    - 21/03/2017
# Modified   - 01/04/2017
#
###---Imports-------------------

import socket
import serial
import struct
import threading
import Queue
import time
import numpy
import random
import pyttsx
import math
import multiprocessing
import glob
import datetime
import copy
import cv2

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
logging.basicConfig(format='%(asctime)s - (%(threadName)s) %(levelname)s: %(message)s', datefmt='%m/%d/%Y %I:%M:%S %p',
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
lastSent = [0, 0]

dataReady = False

# global USConnected
# motorConnected = False

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
    #OLD
    #F_bot, F_left, F_right, L_mid, R_mid, B_mid - F_top, L_front, R_front, L_back, R_back, B_left, B_right


    #ft,fb,fl,fr,bl,bm,br
    #lf,lc,lb,rf,rc,rb

USAvgDistances = [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.]
obstruction = False
usCaution = False
USThresholds = [30, 20, 30] #threasholds for treating objects as obstacles [front,side,back]
#make sure stopping distance is > usThresholds
stoppingDistance = 20
UShosts = 2
maxSpeeds = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

##--Multi-Threading/Muti-processing
global threadLock #lock to be used when changing global variables
global speechQueue #Queue holding sentences to be spoken
global pulsesQueue #Queue holding the measured wheel encoder pulses
global serverFeedbackQueue
global nodeQueue

threadLock = threading.Lock()
speechQueue = multiprocessing.Queue()
pulsesQueue = multiprocessing.Queue()
serverFeedbackQueue = multiprocessing.Queue()
nodeQueue = multiprocessing.Queue()

threads = [] #Array holding information on the currently running threads
processes = [] #Array holding information on the currently running Processes

# -QR Codes
global QRdetected #Boolean for the QRcode detection status
global QRdata #String read from the QR code
global QRLocation

QRdetected = multiprocessing.Value('b', False)
QRdata = ""

global qrDict
global vpDict
global lastQR
# -Camera Functions

grid_size = [0,0]
global cam
global vanish
global vpValid
global expectedFPS
vpValid = multiprocessing.Value('b', False)
vanish = multiprocessing.Value("d",0)
expectedFPS = multiprocessing.Value("d", 0)

#fps = [0]
#frameNumber = [0]

# -IMU data
#global imu #Handle for the IMU
global imuEnable #Boolean holding whether IMU is connected
imuEnable = multiprocessing.Value('b', False)

##--Auto Pilot
global autoPilot #Boolean for turning on/off autopilot
autoPilot = False #autopilot turned off
global pidEnable
pidEnable = False
global avoidingObstacle
avoidingObstacle = False

global AHRSmode
AHRSmode = "wheel"
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

def serialDetect():
    #This fuction will ping all serial connections to find out what each device is

    global motor_portAddr
    global US1_portAddr
    global US2_portAddr

    if platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
        print "ports are " + ports
    elif platform == "linux" or platform == "linux2":
        #check this
        ports = glob.glob('/dev/tty[A-Za-z]*')
        print "ports are " + ports
    else:
        logging.error("Unsupported Platform")

    for port in ports:
        try:
            testConn = serial.Serial(port, 19200)
            logging.info('Checking device at ', port)
            testConn.write("whois?\n")
            response = testConn.readline()
            if response == "motor\n":
                logging.info('Motor Controller at ', port)
                motor_portAddr = port
            elif response == "us1\n":
                logging.info('US 1 at ', port)
                US1_portAddr = port
            elif response == "us2\n":
                logging.info('US 2 at ', port)
                US2_portAddr = port
        except Exception as e:
            logging.error("Error Checking Serial ports - %s", e)

def ConnectToSerial():
    global MotorConn
    global USConn1
    global USConn2

    serialDetect()

    logging.info("Trying to connect to Serial Devices")
    if motor_portAddr != "":
        try:
            MotorConn = serial.Serial(motor_portAddr, 19200)
        except Exception as e:
            logging.error("%s", e)

    if US1_portAddr != "":
        try:
            USConn1 = serial.Serial(US1_portAddr, 19200)
        except Exception as e:
            logging.error("%s", e)

    if US2_portAddr != "":
        try:
            USConn2 = serial.Serial(US2_portAddr, 19200)
        except Exception as e:
            logging.error("%s", e)

def setExitFlag(status):
    global exitFlag
    global exitFlag

    exitFlag = status
    exitFlag.value = status

###---Class Definitions
# create a class for the data to be sent over ait

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


##---------Mapping process



##----------IMU process

def porterTracker(exitFlag,imuEnable, porterLocation_Global, porterOrientation, wheelSpeeds, pulsesQueue, speechQueue):
    
    print "IMU Code Deleted as package could not be found"print "IMU Code Deleted as package could not be found"
    
def movePorter(lPulses, rPulses, porterLocation_Global, porterOrientation,wheelSpeeds,orientationWheels):  # function for calculating the distance moved
    # Assume that it doesnt "move" when rotating
    # Need to check if this function is mathematically sound for all r, theta.

    timeInterval = 0.1  # sampling interval of the motor controller
    wheelRadius = 12
    pulsesPerRev = 360
    wheelBaseWidth = 62
    # find the wheel speeds
    wheelSpeeds[0] = ((lPulses / timeInterval) * 60) / pulsesPerRev
    wheelSpeeds[1] = ((rPulses / timeInterval) * 60) / pulsesPerRev
    # print(str(wheelSpeeds))

    # find the distance moved
    lDist = (2 * numpy.pi * lPulses / pulsesPerRev) * wheelRadius
    rDist = (2 * numpy.pi * rPulses / pulsesPerRev) * wheelRadius

    r = (lDist + rDist) / 2  # take the average of the distances for now.
    distDelta = (lDist - rDist)

    orientationWheels.value += distDelta/wheelBaseWidth

    if orientationWheels.value > numpy.pi:
        orientationWheels.value -= 2*numpy.pi
    elif orientationWheels.value < -numpy.pi:
        orientationWheels.value += 2 * numpy.pi

        # # r is the staight line distance traveled... so find x,y components
    #if lastCommand == "f":
    if not qrDict['visible'] :
        porterLocation_Global[0] += r * numpy.cos(numpy.pi/2 - porterOrientation.value)  # x component
        porterLocation_Global[1] += r * numpy.sin(numpy.pi/2 - porterOrientation.value)  # y component

    #print "Location is " + str(porterLocation_Global)
    # elif lastCommand == "b":
    #     porterLocation_Global[0] = porterLocation_Global[0] - r * numpy.cos(numpy.pi/2 - porterOrientation.value)  # x component
    #     porterLocation_Global[1] = porterLocation_Global[1] - r * numpy.sin(numpy.pi/2 - porterOrientation.value)  # y component

    #print (str(porterLocation_Global) + " __ " + str(porterOrientation.value)+ " __ " + str(orientationWheels.value))

    return porterLocation_Global,wheelSpeeds,orientationWheels.value


class autoPilotThread(MultiThreadBase):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name

        self.angleToGoal = 0 #angle between the goal and the robot
        self.angleChange = 0 #angle change required

        self.dX = 0
        self.dY = 0

        #path-finding parameters
        self.recPenalty = 0.2 #penalty for recursion
        self.momentumBonus = 0.5 #bonus for momentum conservation
        self.alpha = 20 #
        self.beta = 10000

        #Autopilot Control parameters
        self.looping = True #DO NOT CHANGE THIS
        self.obs = False
        self.distQuantise = 30  # path re-calculation distance
        self.autoSpeed = 20  # autopilot movement speed
        self.turningSpeed = 10
        self.alignmentThreshold = 10  # alignment error in degrees
        self.hScores = []
        self.bestScoreIndex = 0
        self.distThreshold = 20

        self.vanishAngles = []

        self.PID_Tuning = (0.01, 0.01, 0)
        self.vanishsum = 0


    def run(self):
        global threadLock
        global dataReady
        global speedVector
        global setSpeedVector
        global distanceToGoal
        global porterLocation_Global
        global porterLocation_Local
        global lastCommand
        global targetDestination
        global autoPilot
        global h_scores
        global pidEnable
        global nodeQueue
        global avoidingObstacle
        #global porterOrientation

        while not exitFlag.value: #while the system isn't in shutdown mode
            if not autoPilot: #if autopilot is disabled...

                #targetDestination = [100,0]
                #self.hScores = self.checkquad()
                #h_scores = self.hScores

                time.sleep(1) #...sleep for a bit ...zzzzzzz
            elif AHRSmode is "imu" and not imuEnable.value:
                logging.warning("IMU not available. Can't turn on Autopilot... Soz...")
                speechQueue.put("IMU not available. Can't turn on Autopilot...")
                autoPilot = False
            # elif self.looping and self.obs:
            #     #porterLocation_Global = [0,0]  # for now assume local position is the same as global
            #     # otherwise put robot into exploration more till it finds a QR code, then position. FOR LATER
            #     logging.info("Iteratve Autopilot mode with Obstacle Avoidance")
            #     speechQueue.put("Iteratve Autopilot mode with Obstacle Avoidance")  # vocalise
            #
            #     self.dX = targetDestination[0] - porterLocation_Global[0]
            #     self.dY = targetDestination[1] - porterLocation_Global[1]
            #
            #     while numpy.sqrt(numpy.square(self.dX) + numpy.square(self.dY)) > self.distThreshold and autoPilot and not exitFlag.value:
            #     # find the angle of the goal from north
            #
            #         speechQueue.put("Quantising the environment")
            #         logging.info("Quantising The environment")
            #         time.sleep(1)
            #
            #         self.hScores = self.checkquad()
            #         h_scores = self.hScores
            #
            #         if self.bestScoreIndex == 0:
            #             speechQueue.put("Best way to go is forward")
            #             logging.info("Best way to go is forward")
            #             self.moveStraight(dist=30,direction="f")
            #         elif self.bestScoreIndex == 1:
            #             speechQueue.put("Best way to go is backwards")
            #             logging.info("Best way to go is backwards")
            #             self.moveStraight(dist=30, direction="b")
            #         elif self.bestScoreIndex == 2:
            #             speechQueue.put("Best way to go is left")
            #             logging.info("Best way to go is left")
            #             self.turn(angleChange= -90)
            #         elif self.bestScoreIndex == 3:
            #             speechQueue.put("Best way to go is right")
            #             logging.info("Best way to go is right")
            #             self.turn(angleChange= 90)
            #
            #         self.dX = targetDestination[0] - porterLocation_Global[0]
            #         self.dY = targetDestination[1] - porterLocation_Global[1]
            #
            #         if obstruction:
            #             time.sleep(5)
            #             speechQueue.put("Obstruction, sleeping for a bit")
            #
            #     with threadLock:
            #         lastCommand = "x"
            #         speedVector = [0, 0]
            #         dataReady = True
            #
            #     speechQueue.put("Successfully arrived at the target")
            #     logging.info("Successfully arrived at the target")
            #     autoPilot = False  # turn off autopilot at the end of the maneuver
            elif not localised.value:

                pidEnable = False
                self.vanishAngles = []
                turnAngle = numpy.rad2deg(porterOrientation.value)
                print ("Current Location = ", porterLocation_Global)
                logging.warning("Not Localised. Motion path unreliable. Trying to find a QR code")
                speechQueue.put("I dont know where I am. I might run into some trouble.")

                #rotate until qrFound
                if not qrDict['visible'] and autoPilot and not exitFlag.value :
                    if USAvgDistances[7] > USAvgDistances[10]:  # more space on the left
                        with threadLock:
                            lastCommand = "l"
                            setSpeedVector = [-self.turningSpeed, self.turningSpeed]
                            speedVector = [-self.turningSpeed, self.turningSpeed]
                            dataReady = True
                    else:
                        with threadLock:
                            lastCommand = "r"
                            setSpeedVector = [self.turningSpeed, -self.turningSpeed]
                            speedVector = [self.turningSpeed, -self.turningSpeed]
                            dataReady = True

                self.angleToGoal = 360
                #turn for 360'
                while not qrDict['visible'] and autoPilot and not exitFlag.value and (abs(self.angleChange) > 1): #and <360'

                    self.angleChange = self.angleToGoal - numpy.rad2deg(porterOrientation.value)
                    print "angle change = " + str(self.angleChange)
                    print "Orientation = " + str(numpy.rad2deg(porterOrientation.value))

                    if vpValid.value:
                        if len(self.vanishAngles)>0 and \
                                abs(self.vanishAngles[len(self.vanishAngles)-1] - porterOrientation.value)>5:
                            print "vp found"
                            self.vanishAngles.append(porterOrientation.value)
                            print (self.vanishAngles)
                    time.sleep(0.1)

                with threadLock:
                    lastCommand = "x"
                    setSpeedVector = [0, 0]
                    speedVector = [0, 0]
                    dataReady = True

                if not qrDict['visible'] :

                    while not qrDict['visible'] and autoPilot and not exitFlag.value:
                        # align to vp

                        if not qrDict['visible'] and not vpValid.value and autoPilot and not exitFlag.value:
                            if USAvgDistances[7] > USAvgDistances[10]:  # more space on the left
                                with threadLock:
                                    lastCommand = "l"
                                    setSpeedVector = [-self.turningSpeed, self.turningSpeed]
                                    speedVector = [-self.turningSpeed, self.turningSpeed]
                                    dataReady = True
                            else:
                                with threadLock:
                                    lastCommand = "r"
                                    setSpeedVector = [self.turningSpeed, -self.turningSpeed]
                                    speedVector = [self.turningSpeed, -self.turningSpeed]
                                    dataReady = True

                        self.vanishAngles = []
                        while not qrDict['visible'] and not vpValid.value and autoPilot and not exitFlag.value:  # and <360'
                            time.sleep(0.1)
                            if vpValid.value:
                                self.vanishAngles.append(porterOrientation.value)

                        with threadLock:
                            lastCommand = "x"
                            setSpeedVector = [0, 0]
                            speedVector = [0, 0]
                            dataReady = True

                        # move to the end of the corridor (until obstruction)
                        if lastCommand != "f":
                            with threadLock:
                                pidEnable = True
                                lastCommand = "f"
                                setSpeedVector = [self.autoSpeed, self.autoSpeed]
                                speedVector = [self.autoSpeed, self.autoSpeed]
                                dataReady = True

                        while not obstruction and not qrDict['visible'] and autoPilot and not exitFlag.value:
                            time.sleep(0.1)

                        with threadLock:
                            lastCommand = "x"
                            setSpeedVector = [0, 0]
                            speedVector = [0, 0]
                            dataReady = True

                        #autoPilot = False
                        time.sleep(2)

                    with threadLock:
                        lastCommand = "x"
                        setSpeedVector = [0, 0]
                        speedVector = [0, 0]
                        dataReady = True

                if qrDict['visible']:
                    #qrDict['location']
                    #qrDict['orientation']
                    # UPDATE LOCATION WHEN SEEN.
                    if (0 <= qrDict['orientation'] < 90):
                        porterLocation_Global = qrDict['location']
                    elif (90 <= qrDict['orientation'] < 180):
                        porterLocation_Global = qrDict['location']
                    elif (180 <= qrDict['orientation'] < 270):
                        porterLocation_Global = qrDict['location']
                    elif (270<= qrDict['orientation'] < 360):
                        porterLocation_Global = qrDict['location']
                    else:
                        logging.error("Something is wrong with the QR code I found. It makes no sense")
                    localised.value = True
                    print "Found a QR code! I am LOCALISED!"
                    print qrDict
                    print ("Current Location = ", porterLocation_Global)
                else :
                    print "I cant find a QR code :( "

                autoPilot = False
                pidEnable = False

            elif self.looping: #iterative mode

                if not nodeQueue.empty():
                    print "Fetching Next Destination... "
                    targetDestination = nodeQueue.get()
                    print ("Destination is " + str(targetDestination))

                    if not localised.value:
                        logging.warning("Not Localised. Motion path unreliable")
                        speechQueue.put("I cant find where I am. I might run into some trouble")

                    #porterLocation_Global = porterLocation_Local  # for now assume local position is the same as global
                    # otherwise put robot into exploration more till it finds a QR code, then position. FOR LATER

                    logging.info("Iteratve Autopilot mode")
                    speechQueue.put("Iteratve Autopilot mode enabled")  # vocalise

                    speechQueue.put("Looking for the target destination")
                    logging.info("Looking for the target destination")

                    self.dX = targetDestination[0] - porterLocation_Global[0]
                    self.dY = targetDestination[1] - porterLocation_Global[1]
                    print "target = " + str(targetDestination)
                    print "location = " + str(porterLocation_Global)


                    while numpy.sqrt(numpy.square(self.dX) + numpy.square(self.dY)) > self.distThreshold and autoPilot and not exitFlag.value:
                    # find the angle of the goal from north

                        self.dX = targetDestination[0] - porterLocation_Global[0]
                        self.dY = targetDestination[1] - porterLocation_Global[1]

                        print "dx = " + str(self.dX) + " dy = " + str(self.dY) + "\n"
                        if self.dX != 0:
                            if (self.dX > 0 and self.dY >= 0):
                                self.angleToGoal = numpy.pi/2 - numpy.arctan(self.dY / self.dX)
                            elif (self.dX < 0 and self.dY >= 0):
                                self.angleToGoal = -(numpy.pi/2 + numpy.arctan(self.dY / self.dX))
                            elif (self.dX < 0 and self.dY <= 0):
                                self.angleToGoal = -(numpy.pi/2 + numpy.arctan(self.dY / self.dX))
                            elif (self.dX > 0 and self.dY <= 0):
                                self.angleToGoal = numpy.pi/2  - numpy.arctan(self.dY / self.dX)
                        elif self.dY >= 0:
                            self.angleToGoal = 0
                        elif self.dY < 0 :
                            self.angleToGoal = numpy.pi

                        self.angleToGoal = numpy.rad2deg(self.angleToGoal)

                        print "Angle to goal is " + str(self.angleToGoal)
                        # same as before, finding the angle that needs to be changed

                        self.angleChange = self.angleToGoal - numpy.rad2deg(porterOrientation.value)

                        if self.angleChange < -180:
                            self.angleChange += 360
                        if self.angleChange > 180:
                            self.angleChange -= 360

                        print "Angle change is " + str(self.angleChange)

                        if autoPilot and not exitFlag.value:
                            # while avoidingObstacle:
                            #     time.sleep(0.2)

                            if (abs(self.angleChange) > self.alignmentThreshold):
                                pidEnable = False
                                print "PID Disabled"

                                with threadLock:
                                    if self.angleChange > self.alignmentThreshold:
                                        lastCommand = "r"
                                        setSpeedVector = [self.turningSpeed, -self.turningSpeed]
                                        speedVector = [self.turningSpeed, -self.turningSpeed]
                                        dataReady = True
                                    elif self.angleChange < self.alignmentThreshold:
                                        lastCommand = "l"
                                        setSpeedVector = [-self.turningSpeed, self.turningSpeed]
                                        speedVector = [-self.turningSpeed, self.turningSpeed]
                                        dataReady = True

                                while (abs(self.angleChange) > self.alignmentThreshold) and autoPilot and not exitFlag.value:  # Add boundaries
                                    # keep checking the angle
                                    self.angleChange = self.angleToGoal - numpy.rad2deg(porterOrientation.value)
                                    print "angle change = " + str(self.angleChange)
                                    print "Orientation = " +  str(numpy.rad2deg(porterOrientation.value))
                                    if self.angleChange < -180:
                                        self.angleChange += 360
                                    if self.angleChange > 180:
                                        self.angleChange -= 360
                                    time.sleep(0.01)

                                # stop turning
                                with threadLock:
                                    lastCommand = "x"
                                    setSpeedVector = [0, 0]
                                    speedVector = [0, 0]
                                    dataReady = True

                                pidEnable = True
                                print "PID Enabled"
                            else:
                            #if True:
                                pidEnable = True
                                print "PID Enabled"

                            # wait till its aligned


                        speechQueue.put("Aligned to the target destination")
                        logging.info("Aligned to the target destination")
                        time.sleep(0.2)

                        # find distance to Goal
                        distanceToGoal = numpy.sqrt(numpy.square(self.dX) + numpy.square(self.dY))
                        print "distance to Goal is " + str(distanceToGoal)
                        # move to destination
                        speechQueue.put("Moving to the target")
                        logging.info("Moving to Target")

                        with threadLock:
                            lastCommand = "f"
                            setSpeedVector = [self.autoSpeed, self.autoSpeed]
                            speedVector = [self.autoSpeed, self.autoSpeed]
                            dataReady = True

                        initLoc = copy.deepcopy(porterLocation_Global)
                        self.distTravelled = 0

                        while (self.distTravelled < self.distQuantise) and (distanceToGoal > self.distThreshold) and autoPilot and not exitFlag.value: #change this to allow for looping the whole block from else till at goal.
                            self.dX = targetDestination[0] - porterLocation_Global[0]
                            self.dY = targetDestination[1] - porterLocation_Global[1]
                            distanceToGoal = numpy.sqrt(numpy.square(self.dX) + numpy.square(self.dY))
                            #print "Porter Location is " + str(porterLocation_Global)
                            self.dX = porterLocation_Global[0] - initLoc[0]
                            self.dY = porterLocation_Global[1] - initLoc[1]
                            self.distTravelled = numpy.sqrt(numpy.square(self.dX) + numpy.square(self.dY))
                            #print "distance Travelled is " + str(self.distTravelled)
                            time.sleep(0.1)

                        self.dX = targetDestination[0] - porterLocation_Global[0]
                        self.dY = targetDestination[1] - porterLocation_Global[1]

                        logging.info("Iteration complete... Recalculating...")

                    with threadLock:
                        lastCommand = "x"
                        setSpeedVector = [0, 0]
                        speedVector = [0, 0]
                        dataReady = True

                    speechQueue.put("Successfully arrived at the target")
                    logging.info("Successfully arrived at the target")

                else:
                    speechQueue.put("No Nodes available. Stopping Autopilot")
                    logging.info("No Nodes available. Stopping Autopilot")

                if nodeQueue.empty():
                    autoPilot = False  # turn off autopilot at the end of the maneuver
                    pidEnable = False

            else: #if in autopilot mode...
                porterLocation_Global = porterLocation_Local # for now assume local position is the same as global
                    # otherwise put robot into exploration more till it finds a QR code, then position. FOR LATER

                # find the required angle change to align to global grid
                if autoPilot and not exitFlag.value:
                    logging.info("Looking for north")
                    speechQueue.put("Looking for north") #vocalise
                    time.sleep(1) #sleep for presentation purposes

                # Orienting to X-axis...
                self.angleChange = 90 - numpy.rad2deg(porterOrientation.value) #find required angle change
                    # right if +ve left if -ve
                if self.angleChange > 180: # if >180 turn left instead of right
                    self.angleChange -= 360

                if autoPilot and not exitFlag.value:
                    with threadLock: #with exclusive global variable access...
                        if self.angleChange > self.alignmentThreshold: #if need to turn right
                            lastCommand = "r" #set the command to be right (for obstacle avoidance purposes)
                            speedVector = [self.autoSpeed, -self.autoSpeed] #set the required speeds
                            dataReady = True #this signals the motor control thread that there is data to be sent
                        elif self.angleChange < self.alignmentThreshold: #if need to turn left
                            lastCommand = "l"
                            speedVector = [-self.autoSpeed, self.autoSpeed]
                            dataReady = True

                # wait till its aligned
                while (abs(self.angleChange) > self.alignmentThreshold) and autoPilot and not exitFlag.value:  #while not aligned
                    # keep calculating the angle change required
                    self.angleChange = 90 - numpy.rad2deg(porterOrientation.value)  # right if +ve left if -ve
                    if self.angleChange > 180:
                        self.angleChange -= 360  # if >180 turn left instead of right
                    time.sleep(0.01) #sleep a bit so that the CPU isnt overloaded

                # stop turning
                with threadLock:
                    lastCommand = "x"
                    speedVector = [0, 0]
                    dataReady = True

                # aligned to x axis... YAY!
                if autoPilot and not exitFlag.value:
                    speechQueue.put("Aligned to X axis")
                    logging.info("Aligned to X axis")
                    time.sleep(1)

                #find the angle of the goal from north
                self.dX = targetDestination[0] - porterLocation_Global[0]
                self.dY = targetDestination[1] - porterLocation_Global[1]

                if self.dX != 0:
                    self.angleToGoal = numpy.arctan(self.dY / self.dX)
                else:
                    self.angleToGoal = numpy.pi / 2

                self.angleToGoal = numpy.rad2deg(self.angleToGoal)

                # convert angle to 0,2pi
                if self.dX >= 0:  # positive x
                    if self.dY >= 0:
                        self.angleToGoal = numpy.rad2deg(porterOrientation.value) - self.angleToGoal
                    if self.dY < 0:  # negative y
                        self.angleToGoal = numpy.rad2deg(porterOrientation.value) - self.angleToGoal

                elif self.dX < 0:  # negative x
                    if self.dY >= 0:  # positive y
                        self.angleToGoal = numpy.rad2deg(porterOrientation.value) - (180 + self.angleToGoal)
                    if self.dY < 0:  # negative y
                        self.angleToGoal = numpy.rad2deg(porterOrientation.value) - (180 + self.angleToGoal)

                # at this point angleToGoal is defined relative to north
                # if this works remove redundant IF statements. Currently used only for debugging.

                #same as before, finding the angle that needs to be changed
                self.angleChange = self.angleToGoal - numpy.rad2deg(porterOrientation.value)
                if self.angleChange < -180:
                    self.angleChange += 360

                if autoPilot and not exitFlag.value:
                    with threadLock:
                        if self.angleChange > self.alignmentThreshold:
                            lastCommand = "r"
                            speedVector = [self.autoSpeed, -self.autoSpeed]
                            dataReady = True
                        elif self.angleChange < self.alignmentThreshold:
                            lastCommand = "l"
                            speedVector = [-self.autoSpeed, self.autoSpeed]
                            dataReady = True

                if autoPilot and not exitFlag.value:
                    speechQueue.put("Looking for the target destination")
                    logging.info("Looking for the target destination")
                    time.sleep(1)

                # wait till its aligned
                while (abs(self.angleChange) > self.alignmentThreshold) and autoPilot and not exitFlag.value:  # Add boundaries
                    # keep checking the angle
                    self.angleChange = self.angleToGoal - numpy.rad2deg(porterOrientation.value)
                    if self.angleChange < -180:
                        self.angleChange += 360
                    time.sleep(0.01)

                # stop turning
                with threadLock:
                    lastCommand = "x"
                    speedVector = [0, 0]
                    dataReady = True
                    # aligned to x axis
                if autoPilot and not exitFlag.value:
                    speechQueue.put("Aligned to the target destination")
                    logging.info("Aligned to the target destination")
                    time.sleep(1)

                # find distance to Goal
                distanceToGoal = numpy.sqrt(numpy.square(self.dX) + numpy.square(self.dY))
                # move to destination
                if autoPilot and not exitFlag.value:
                    speechQueue.put("Moving to the target")
                    logging.info("Moving to Target")

                #COMPLETE THIS BIT
                # with threadLock:
                #     lastCommand = "f"
                #     speedVector = [self.autoSpeed, self.autoSpeed]
                #     dataReady = True

                # while (distanceToGoal > 30) and autoPilot and not exitFlag.value: #change this to allow for looping the whole block from else till at goal.
                #     self.dX = targetDestination[0] - porterLocation_Global[0]
                #     self.dY = targetDestination[1] - porterLocation_Global[1]
                #     distanceToGoal = numpy.sqrt(numpy.square(self.dX) + numpy.square(self.dY))

                with threadLock:
                    lastCommand = "x"
                    speedVector = [0, 0]
                    dataReady = True

                if autoPilot and not exitFlag.value:
                    speechQueue.put("Successfully arrived at the target")
                    logging.info("Successfully arrived at the target")
                    autoPilot = False #turn off autopilot at the end of the maneuver

        with threadLock:
            lastCommand = "x"
            speedVector = [0, 0]
            dataReady = True

        autoPilot = False
        pidEnable = False

    def turn(self,angleChange):
        global lastCommand
        global speedVector
        global dataReady

        initOrientation = numpy.rad2deg(porterOrientation.value)

        if autoPilot and not exitFlag.value and not obstruction:
            with threadLock:
                if angleChange > self.alignmentThreshold:
                    lastCommand = "r"
                    speedVector = [self.autoSpeed, -self.autoSpeed]
                    dataReady = True
                elif angleChange < self.alignmentThreshold:
                    lastCommand = "l"
                    speedVector = [-self.autoSpeed, self.autoSpeed]
                    dataReady = True

        while (abs(initOrientation - numpy.rad2deg(porterOrientation.value)) < (abs(angleChange) + self.alignmentThreshold)) and autoPilot and not exitFlag.value and not obstruction:  # while not aligned
            time.sleep(0.01)  # sleep a bit so that the CPU isnt overloaded

        # stop turning
        with threadLock:
            lastCommand = "x"
            speedVector = [0, 0]
            dataReady = True

    def moveStraight(self,dist,direction): #direction can be "f" or "b"
        global lastCommand
        global speedVector
        global dataReady

        with threadLock:
            if direction == "f":
                lastCommand = "f"
                speedVector = [self.autoSpeed, self.autoSpeed]
                dataReady = True
            elif direction == "b":
                lastCommand = "b"
                speedVector = [-self.autoSpeed, -self.autoSpeed]
                dataReady = True

        initLoc = porterLocation_Global
        self.distTravelled = 0

        while (self.distTravelled < dist) and autoPilot and not exitFlag.value and not obstruction:  # change this to allow for looping the whole block from else till at goal.
            self.dX = porterLocation_Global[0] - initLoc[0]
            self.dY = porterLocation_Global[1] - initLoc[1]
            self.distTravelled = numpy.sqrt(numpy.square(self.dX) + numpy.square(self.dY))

        with threadLock:
            lastCommand = "x"
            speedVector = [0, 0]
            dataReady = True

    ###TO BE IMPLEMENTED
    ##AutoPilot motion to be quantised to distQuantise

    def checkquad(self):
        distScore = []  # relative to the current orientation f,b,l,r
        envScore = []
        distScore = self.checkdist(distScore)
        envScore = self.envCheck()
        return self.calcHeuristic(distScore, envScore)

    def checkdist(self, distScore):

        directions = [numpy.rad2deg(porterOrientation.value), 180 - numpy.rad2deg(porterOrientation.value),
                      90 - numpy.rad2deg(porterOrientation.value), 90 + numpy.rad2deg(porterOrientation.value)]

        for direction in directions:
            tempXpos = porterLocation_Global[0] + self.distQuantise * numpy.cos(90 - direction)
            tempYpos = porterLocation_Global[1] + self.distQuantise * numpy.sin(90 - direction)

            tempDX = targetDestination[0] - tempXpos
            tempDY = targetDestination[1] - tempYpos

            tempDist = numpy.sqrt(numpy.square(tempDX) + numpy.square(tempDY))
            distScore.append(tempDist)

        print("dist score is " + str(distScore) + "min dist score is " + str(min(distScore)) + " at " +
              str(distScore.index(min(distScore))))

        return distScore

    def envCheck(self):
        #self.envScore = []

        envScore = [-200/USAvgDistances[0] + 10,
                    -5000 / USAvgDistances[5] + 10,
                    -500 / USAvgDistances[8] + 10,
                    -500 / USAvgDistances[11] + 10]
        #Exponential mapping of distance

        print("environment score is " + str(envScore) + " max env score is " + str(max(envScore)) + " at " +
              str(envScore.index(max(envScore))))

        return envScore

    def calcHeuristic(self,distScore, envScore):

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

        self.debugServer = False
        self.debugClient = False
        self.dataStore = ""

    def run(self):
        logging.info("Starting %s", self.name)
        while not exitFlag.value:
            self.loopStartFlag()
            if not self.debugServer:
                self.runServer()
            if self.debugServer and not self.debugClient:
                self.waitForClient()
            if self.debugClient:
                try:
                    logging.debug("Sending info to Logger")
                    # Start Flag
                    self.clientConnection.send("#") #0
                    # Time
                    self.clientConnection.send(time.ctime() + ',')
                    # Ultrasonic data
                    self.clientConnection.send(str(USAvgDistances[1]) + ",") #1
                    self.clientConnection.send(str(USAvgDistances[2]) + ",")
                    self.clientConnection.send(str(USAvgDistances[3]) + ",")
                    self.clientConnection.send(str(USAvgDistances[8]) + ",")
                    self.clientConnection.send(str(USAvgDistances[11]) + ",")
                    self.clientConnection.send(str(USAvgDistances[5]) + ",") #6
                    # motor speeds
                    # demanded motor speeds
                    self.clientConnection.send(str(speedVector[0]) + ",") #7
                    self.clientConnection.send(str(speedVector[1]) + ",")
                    #actual speeds
                    self.clientConnection.send(str(wheelSpeeds[0]) + ",") #9
                    self.clientConnection.send(str(wheelSpeeds[1]) + ",")
                    #POSITIONS
                    #target
                    self.clientConnection.send(str(targetDestination[0]) + ",") #11
                    self.clientConnection.send(str(targetDestination[1]) + ",")
                    #Porter_global
                    self.clientConnection.send(str(porterLocation_Global[0]) + ",") #13
                    self.clientConnection.send(str(porterLocation_Global[1]) + ",")
                    #Porter_local
                    self.clientConnection.send(str(porterLocation_Local[0]) + ",") #15
                    self.clientConnection.send(str(porterLocation_Local[1]) + ",")
                    #porterOrientation
                    self.clientConnection.send(str(porterOrientation.value) + ",") #17
                    # AHRS # FOR VALIDATION ONLY
                    # yaw
                    self.clientConnection.send(str(orientationIMU.value) + ",") #18


                    # thread life status
                    self.clientConnection.send(str(threadLock.locked()) + ",") #19
                    # current lock

                    # safety staus
                    self.clientConnection.send(str(safetyOn) + ",") #20
                    # obstruction status
                    self.clientConnection.send(str(obstruction) + ",") #21
                    # data status
                    self.clientConnection.send(str(dataReady)+",") #22

                    self.clientConnection.send(str(orientationWheels.value) + ",")  # 23

                    #US data 2
                    self.clientConnection.send(str(USAvgDistances[0]) + ",")  # 24
                    self.clientConnection.send(str(USAvgDistances[7]) + ",")
                    self.clientConnection.send(str(USAvgDistances[10]) + ",")
                    self.clientConnection.send(str(USAvgDistances[9]) + ",")
                    self.clientConnection.send(str(USAvgDistances[12]) + ",")
                    self.clientConnection.send(str(USAvgDistances[4]) + ",")
                    self.clientConnection.send(str(USAvgDistances[6]) + ",")  # 30

                    self.clientConnection.send(str(h_scores[0]) + "," )  # 31
                    self.clientConnection.send(str(h_scores[1]) + "," )
                    self.clientConnection.send(str(h_scores[2]) + "," )
                    self.clientConnection.send(str(h_scores[3]) + ",")  # 34

                    self.clientConnection.send(str(vpValid.value) + ",") #35
                    self.clientConnection.send(str(vanish.value) + ",")
                    self.clientConnection.send(str(expectedFPS.value)) #37

                    self.clientConnection.send("\n")
                    # self.logOverNetwork()
                    time.sleep(1)

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
        global pulsesQueue

        logging.info("Starting %s", self.name)
        while not exitFlag.value:
            self.loopStartFlag()

            if obstruction:
                if autoPilot and self.obs:
                    if lastSent != [0, 0]:  # ie still moving
                        # pause motion
                        logging.info("Obstacle Detected. Path needs to be recalculated")
                        speechQueue.put("Obstacle Detected. Path needs to be recalculated")
                        self.send_serial_data([0, 0])
                    # else:
                    #     while obstruction:
                    #         time.sleep(0.1)
                    #     speechQueue.put("Obstacle removed. Proceeding to destination")
                    #     logging.info("Obstacle removed. Proceeding to destination")
                    #     self.send_serial_data(speedVector)
                elif autoPilot:
                    logging.info("autopilot command to motor")
                    if lastSent != [0, 0]:  # ie still moving
                        # pause motion
                        logging.info("Obstacle Detected. Waiting for it to go away")
                        speechQueue.put("Obstacle Detected. Waiting for it to go away")
                        self.send_serial_data([0, 0])
                    else:
                        while obstruction and autoPilot and not exitFlag.value:
                            time.sleep(0.1)
                        if not obstruction and autoPilot and not exitFlag.value:
                            speechQueue.put("Obstacle removed. Proceeding to destination")
                            logging.info("Obstacle removed. Proceeding to destination")
                            self.send_serial_data(speedVector)
                elif safetyOn: #if not autopilot and the safety is on
                    if lastSent != [0, 0]:
                        logging.debug("Setting Speed Vector")
                        with threadLock:
                            speedVector = [0, 0]
                            dataReady = False
                        self.send_serial_data(speedVector)

                elif lastSent != speedVector: #otherwise...
                    logging.warning("Obstacle Detected But Safety OFF...") #give a warning, but dont do anything to stop
                    self.send_serial_data(speedVector)
            elif self.smoothBrake and usCaution:
                print "smooth braking"
                if lastSent != [0, 0]:
                    logging.info("Setting smooth speed")
                    with threadLock:
                        if lastCommand == "f":
                            speedVector = [min(maxSpeeds[0:3]),min(maxSpeeds[0:3])]
                            dataReady = False
                        elif lastCommand == "b":
                            speedVector = [min(maxSpeeds[4:6]),min(maxSpeeds[4:6])]
                            dataReady = False
                        else:
                            logging.warning("Smooth Braking is not valid for rotation")
                    self.send_serial_data(speedVector)

            elif dataReady and (lastSent != speedVector): #no obstruction and the new command is different to the last command
                logging.debug("Data Ready")
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

            # READ FROM ARDUINO?
            if MotorConn.inWaiting() > 0:
                #logging.info("data from motor available")
                self.inputBuf = MotorConn.readline()
                self.inputBuf = self.inputBuf.rstrip("\r\n")
                self.pulses = self.inputBuf.split(",")
                pulsesQueue.put(self.pulses)
                #pulsesQueue.get()
                #pulsesQueue.task_done()
                #logging.info("data from motor read")

            if self.profiling:
                time.sleep(0.1)
                # self.read_serial_data()
            else:
                time.sleep(0.001)
                # self.loopEndFlag()
                # self.loopRunTime()


        logging.info("Exiting")

    def send_serial_data(self, sendCommand):
        global lastSent
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
            lastSent = sendCommand
        except Exception as e:
            logging.error("Sending to Motor failed - %s", str(e))
            # self.actionState = 4

    def read_serial_data(self):
        motorInput = MotorConn.readline()

        logging.debug("motor says %s", motorInput)
        if motorInput == "3\r\n":  # recieved
            self.actionState = 3
            logging.info("Command successfully received by motor...")
        elif motorInput == "2\r\n":
            logging.debug("Motor actuating command...")
            self.actionState = 2
        elif motorInput == "1\r\n":
            logging.debug("Motor actuation finished... ready to accept new data :)")
            self.actionState = 1
        elif motorInput == "5\r\n":
            logging.error("ERROR TRYING TO EXECUTE COMMAND :( ...")
            self.actionState = 5

class usDataThread(MultiThreadBase):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.rawUSdata_1 = [0., 0., 0., 0., 0., 0., 0.]
        self.rawUSdata_2 = [0., 0., 0., 0., 0., 0.]

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
                self.inputBuf1 = self.inputBuf1.rstrip("\n")
                self.rawUSdata_1 = self.inputBuf1.split(",")
                #print ("rw1 ", self.inputBuf1)
            except Exception as e:
                logging.error("%s", e)

    def getUSvector2(self):
        while not exitFlag.value:
            try:
                self.inputBuf2 = USConn2.readline()
                self.inputBuf2 = self.inputBuf2.rstrip('\n')
                self.rawUSdata_2 = self.inputBuf2.split(",")
                #print ("rw2 ", self.rawUSdata_2)
            except Exception as e:
                logging.error("%s", e)

    def getMaxSpeeds(self,usData):
        global stoppingDistance
        global maxSpeeds

        #aMax = 500
        aMax = (2*numpy.pi*1000)/(60*0.12)
        #radToRPM = 60/(2*numpy.pi)
        for i in range(0,len(usData)):
            if int(usData[i]) >= stoppingDistance:
                #print "GOOOD!"
                maxSpeeds[i] = int(numpy.sqrt(aMax*(((usData[i]/100) -(stoppingDistance/100)))))
            else:
                #print "BAD :("
                maxSpeeds[i] = 0

        print "max speeds = " + str(maxSpeeds)

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
                #self.getUSvector()
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
                time.sleep(0.01)

        logging.info("Exiting")

    def getUSvector(self):
        logging.debug("inside getUSVector")

        try:
            logging.debug("Trying to get US data 1")
            #USConn1.flushInput()
            self.inputBuf = USConn1.readline()
            self.inputBuf.rstrip("\n")
            self.rawUSdata_1 = self.inputBuf.split(",")
            print ("rw1 ", self.rawUSdata_1)
            if UShosts == 2:
                logging.debug("Trying to get US data 2")
                #USConn2.flushInput()
                self.inputBuf = USConn2.readline()
                self.inputBuf.rstrip("\n")
                self.rawUSdata_2 = self.inputBuf.split(",")
                print ("rw2 ", self.rawUSdata_2)

            print " "

        except Exception as e:
            self.errorCount += 1
            logging.error("error getting us vector - %s", str(e))

    def medianFilter(self):
        pass

    def mAverage(self, n):
        global USAvgDistances
        global threadLock
        i = 0

        rawUSdata = self.rawUSdata_1 + self.rawUSdata_2
        #print rawUSdata

        if len(rawUSdata) == 13:
            with threadLock:
                for i in range(0, len(USAvgDistances)):
                    USAvgDistances[i] += (int(rawUSdata[i]) - USAvgDistances[i]) / n
                if self.profiling:
                    print ("\t" + self.name + " Avg Vector - " + str(USAvgDistances))

    def US_safety_interrupt(self):
        global USAvgDistances
        global lastCommand
        global dataReady
        global obstruction
        global threadLock
        global usCaution

        # if safetyOn:
        try:
            if lastCommand == "f":
                if (int(USAvgDistances[1]) < USThresholds[0]) or (int(USAvgDistances[0]) < USThresholds[0]) or (int(USAvgDistances[2]) < USThresholds[0]) or (int(USAvgDistances[3]) < USThresholds[0]):
                    logging.warning("FRONT TOO CLOSE. STOPPPPP!!!")
                    if obstruction != True:
                        with threadLock:
                            obstruction = True
                    print ("\t" + self.name + " Avg Vector - " + str(int(USAvgDistances[2]))
                           + ", " + str(int(USAvgDistances[1])) + ", " + str(int(USAvgDistances[3])) + ", " + str(int(USAvgDistances[0])))
                elif obstruction != False:
                    with threadLock:
                        obstruction = False

                if (maxSpeeds[1] > abs(speedVector[0])) or (maxSpeeds[0] > abs(speedVector[0])) or (maxSpeeds[2] > abs(speedVector[0])) or (maxSpeeds[3] > abs(speedVector[0])):
                    print("CAUTION\n")
                    usCaution = True
                else:
                    usCaution = False

            elif lastCommand == "b":
                if (int(USAvgDistances[4]) < USThresholds[2]) or (int(USAvgDistances[5]) < USThresholds[2]) or (int(USAvgDistances[6]) < USThresholds[2]):
                    logging.warning("BACK TOO CLOSE. STOPPPPP!!!")
                    if obstruction != True:
                        with threadLock:
                            obstruction = True
                    print ("\t" + self.name + " Avg Vector - " + str(int(USAvgDistances[4])) + ", " + str(int(USAvgDistances[5])) + ", " + str(int(USAvgDistances[6])) )
                elif obstruction != False:
                    with threadLock:
                        obstruction = False

                if (maxSpeeds[6] > abs(speedVector[0])) or (maxSpeeds[5] > abs(speedVector[0])) or (maxSpeeds[6] > abs(speedVector[0])):
                    usCaution = True
                    print("CAUTION\n")
                else:
                    usCaution = False

            elif lastCommand == "l":
                if (int(USAvgDistances[7]) < USThresholds[1]) or (int(USAvgDistances[8]) < USThresholds[1]) or (int(USAvgDistances[12]) < USThresholds[1]/2):
                    logging.warning("LEFT SIDE TOO CLOSE. STOPPPPP!!!")
                    if obstruction != True:
                        with threadLock:
                            obstruction = True
                    print ("\t" + self.name + " Avg Vector - " + str(int(USAvgDistances[7])) + ", " + str(int(USAvgDistances[8])) + ", " + str(int(USAvgDistances[9])))
                elif obstruction != False:
                    with threadLock:
                        obstruction = False

            elif lastCommand == "r":
                if (int(USAvgDistances[10]) < USThresholds[1]) or (int(USAvgDistances[11]) < USThresholds[1]) or (int(USAvgDistances[9]) < USThresholds[1]/2):
                    logging.warning("RIGHT SIDE TOO CLOSE. STOPPPPP!!!")
                    if obstruction != True:
                        with threadLock:
                            obstruction = True
                    print ("\t" + self.name + " Avg Vector - " + str(int(USAvgDistances[10])) + ", " + str(int(USAvgDistances[11])) + ", " + str(int(USAvgDistances[12])) + ", ")
                elif obstruction != False:
                    with threadLock:
                        obstruction = False
            elif lastCommand == "x":
                with threadLock:
                    obstruction = False


        except Exception as e:
            self.errorCount += 1
            logging.error("error in US interrupt function - %s", str(e))

class ttsThread(MultiThreadBase): #text to speech thread
    def run(self):

        #initialise speech engine
        engine = pyttsx.init()
        rate = engine.getProperty('rate')
        engine.setProperty('rate', rate - 50)

        while not exitFlag.value:
            #talk while there are things to be said
            if not speechQueue.empty():
                engine.say(speechQueue.get())
                engine.runAndWait()

            time.sleep(1)

        engine.say("Shutting down")
        engine.runAndWait()

def porterSpeech(speechQueue,exitFlag):
    # initialise speech engine
    engine = pyttsx.init()
    rate = engine.getProperty('rate')
    engine.setProperty('rate', rate - 50)

    while not exitFlag.value:
        # talk while there are things to be said
        if not speechQueue.empty():
            engine.say(speechQueue.get())
            engine.runAndWait()
            #speechQueue.task_done()
        else:
            time.sleep(0.5)

        #time.sleep(0.1)

    engine.say("Shutting down")
    engine.runAndWait()

# def ():  # QR codes and other camera related stuff if doing optical SLAM use a different thread
#     pass

class PIDThreadClass(MultiThreadBase):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.vp_PID_Tuning = (0.04, 0.0, 0)
        self.qr_PID_Tuning = (0.04, 0.0, 0)
        self.obs_PID_Tuning = (0.01, 0.01, 0)
        self.side_PID_Tuning = (0.02, 0.0, 0)
        self.vanishSum = 0
        self.qrSum = 0

        self.usError_front_Sum = 0
        self.usError_front= 0
        self.usError_front_last= 0
        self.usError_sideL_Sum = 0
        self.usError_sideR_Sum = 0
        self.usError_side= 0
        self.usError_sideL_last= 0
        self.usError_sideR_last= 0


    def run(self):
        global setSpeedVector
        global speedVector
        global pidEnable
        global vpValid
        global vanish
        global qrDict

        if not exitFlag.value:
            threading.Timer(0.2, self.run).start()

            if pidEnable:
                self.usError_front = USAvgDistances[3] - USAvgDistances[2]

                # if USAvgDistances[8] < USThresholds[1] or USAvgDistances[11] < USThresholds[1]:
                #     logging.info("side adjustment... ")
                #
                #     if True: #for left
                #         logging.info("adjusting for left")
                #         self.usError_side = USAvgDistances[8] - USThresholds[1] -10
                #         self.usError_sideL_Sum += self.usError_side
                #         #self.usError_sideR_Sum = 0
                #
                #         offset = (self.side_PID_Tuning[0] * self.usError_side) + (self.side_PID_Tuning[1] * self.usError_sideL_Sum)\
                #                  + (self.side_PID_Tuning[2]*(self.usError_side-self.usError_sideL_last))
                #         offset = (offset * ((setSpeedVector[0] + setSpeedVector[1]) / 2)) / 1
                #
                #         if offset >= 0:
                #             lastCommand = "l"
                #             speedVector = [int(setSpeedVector[0]-offset), int(setSpeedVector[1])]
                #         else:
                #             lastCommand = "r"
                #             speedVector = [int(setSpeedVector[0]), int(setSpeedVector[1]+offset)]
                #
                #         self.usError_sideL_last = self.usError_side

                    # if USAvgDistances[11] < USThresholds[1]: #for left
                    #     logging.info("adjusting for right")
                    #     self.usError_side = USThresholds[1] - USAvgDistances[11]
                    #     #self.usError_sideR_Sum += self.usError_side
                    #     #self.usError_sideL_Sum = 0
                    #
                    #     offset = (self.side_PID_Tuning[0] * self.usError_side) + (self.side_PID_Tuning[1] * self.usError_sideR_Sum) \
                    #              + (self.side_PID_Tuning[2] * (self.usError_side - self.usError_sideR_last))
                    #     offset = (offset * ((setSpeedVector[0] + setSpeedVector[1]) / 2)) / 1
                    #
                    #     lastCommand = "l"
                    #     speedVector = [int(setSpeedVector[0] - offset), int(setSpeedVector[1])]
                    #
                    #     self.usError_sideR_last = self.usError_side

# elif abs(self.usError_front) > 30:
                #     self.usError_front_Sum += self.usError_front
                #     print "us Error = " + str(self.usError_front)
                #     avoidingObstacle = True
                #     offset = (self.obs_PID_Tuning[0] * self.usError_front) + (self.obs_PID_Tuning[1] * self.usError_front_Sum)
                #     offset = (offset * ((setSpeedVector[0] + setSpeedVector[1]) / 2)) / 1
                #
                #     if offset >= 0:
                #         lastCommand = "r"
                #         speedVector = [int(setSpeedVector[0]), int(setSpeedVector[1] - offset)]
                #     else:
                #         lastCommand = "l"
                #         speedVector = [int(setSpeedVector[0] + offset), int(setSpeedVector[1])]


                # qrDict['visible'] = False
                # qrDict['string'] = data[0]
                # qrDict['location'] = [0, 0]
                # qrDict['distance'] = D
                # qrDict['centre'] = [x_centre, y_centre]
                #
                #
                # if qrDict['visible']:
                #     #print qrDict
                #     print ("qr Code dictionary = ", qrDict)
                #     self.vanishSum = 0
                #     logging.info("QR Code adjustment")
                #     avoidingObstacle = False
                #     self.qrSum += qrDict['centre'][0]
                #     offset = (self.qr_PID_Tuning[0] * qrDict['centre'][0]) + (self.qr_PID_Tuning[1] * self.qrSum)
                #
                #     offset = (offset * ((setSpeedVector[0] + setSpeedVector[1]) / 2)) / 10
                #
                #     if offset >= 0:
                #         lastCommand = "r"
                #         speedVector = [int(setSpeedVector[0]), int(setSpeedVector[1] - offset)]
                #     else:
                #         lastCommand = "l"
                #         speedVector = [int(setSpeedVector[0] + offset), int(setSpeedVector[1])]

                if vpValid.value:

                    print ("vp Code dictionary = ", vpDict)
                    self.qrSum = 0
                    # if QRdetected.value:
                    #     vanish.value = qrDict['xLoc']

                    #logging.info("VP Valid")
                    logging.info("VP adjustment")
                    avoidingObstacle = False
                    self.vanishSum += vanish.value
                    offset = (self.vp_PID_Tuning[0] * vanish.value) + (self.vp_PID_Tuning[1] * self.vanishSum)

                    offset = (offset * ((setSpeedVector[0] + setSpeedVector[1])/2)) / 10

                    if offset >= 0:
                        lastCommand = "r"
                        speedVector = [int(setSpeedVector[0]), int(setSpeedVector[1] - offset)]
                    else:
                        lastCommand = "l"
                        speedVector = [int(setSpeedVector[0] + offset), int(setSpeedVector[1])]
                else:
                    logging.info("VP INVALID")
                    self.vanishSum = 0
                    speedVector = setSpeedVector
                    lastCommand = "f"
                    avoidingObstacle = False

                with threadLock:
                    dataReady = True



        #print datetime.datetime.now(), adjustedSpeed

# class CameraThreadClass(MultiThreadBase):
#     def __init__(self, threadID, name):
#         threading.Thread.__init__(self)
#         self.threadID = threadID
#         self.name = name
#         self.pulseData = ["",""] #pulses counted by the motor controller
#         self.vanishx = [0, 0, 0, 0, 0]
#         self.vanishy = [0, 0, 0, 0, 0]
#         self.vanishxVar = []
#         self.vanishyVar = []
#         self.profiling = True
#
#     def run(self):
#         global cam
#         self.vpFromCAM()

def qrCalibrate(qrProcessor):
    # Instruct user
    print 'To calibrate the RoboPorter set camera to 5 cm away from QR Code'
    print 'Click on the window when ready to continue...'

    # Waiting for user input
    qrProcessor.user_wait()

    # Process one QR Code
    qrProcessor.process_one()

    # Calibrate Camera Focal Length Function
    # When sysmbol detected..
    F = 0

    for symbol in qrProcessor.results:
        # Seperate code into useful data
        data = symbol.data.split(',')
        location = data[0]
        size = int(data[1])

        # Save code corners
        x0 = symbol.location[0][0]
        x1 = symbol.location[1][0]
        x2 = symbol.location[2][0]
        x3 = symbol.location[3][0]
        y0 = symbol.location[0][1]
        y1 = symbol.location[1][1]
        y2 = symbol.location[2][1]
        y3 = symbol.location[3][1]

        # Calculate x and y centre points
        x_centre = (x0 + x1 + x2 + x3) / 4
        y_centre = (y0 + y1 + y2 + y3) / 4

        # Calculate average pixel width
        P_x = (abs(x_centre - x0) + abs(x_centre - x1) + abs(x_centre - x2) + abs(x_centre - x3)) / 2
        P_y = (abs(y_centre - y0) + abs(y_centre - y1) + abs(y_centre - y2) + abs(y_centre - y3)) / 2
        P = (P_x + P_y) / 2

        # Set Code actual size in mm
        W = size

        # Set distance to 1 metre
        D = 50

        # Calculate Focal length of camera
        F = (P * D) / W

        # Print result
        print 'Focal length is', F
        time.sleep(1)

    return F

def scanQRCode(inputImage, qrDict, lastQR):

    updateEnabled = True
    # create a Scanner
    scanner = zbar.ImageScanner()

    # configure the Scanner
    scanner.parse_config('enable')

    cv2_image = cv2.cvtColor(inputImage, cv2.COLOR_BGR2GRAY)
    pil = Image.fromarray(cv2_image)

    #pil = inputImage #.open('cam.jpg').convert('L')
    width, height = pil.size
    raw = pil.tobytes()

    # wrap image data
    image = zbar.Image(width, height, 'Y800', raw)
    #print "Ready..."

    # scan the image for QR Codes
    try: #try to scan the image
        scanner.scan(image)
    except Exception as e:
        logging.error("%s", e)

    # When symbol detected
    i = 0
    for symbol in image:
        i+=1
        #print "symbol obtained"

        # Seperate code into useful data
        data = symbol.data.split(',')
        #print ("data = ", data)

        if len(data) == 5:
            location = data[0]
            size = float(data[1])
            # Save code corners
            x0 = symbol.location[0][0]
            x1 = symbol.location[1][0]
            x2 = symbol.location[2][0]
            x3 = symbol.location[3][0]
            y0 = symbol.location[0][1]
            y1 = symbol.location[1][1]
            y2 = symbol.location[2][1]
            y3 = symbol.location[3][1]

            # Calculate x and y centre points
            x_centre = (x0 + x1 + x2 + x3) / 4
            y_centre = (y0 + y1 + y2 + y3) / 4

            # Calculate average pixel width
            P_x = (abs(x_centre - x0) + abs(x_centre - x1) + abs(x_centre - x2) + abs(x_centre - x3)) / 2
            P_y = (abs(y_centre - y0) + abs(y_centre - y1) + abs(y_centre - y2) + abs(y_centre - y3)) / 2
            P = (P_x + P_y) / 2

            # Set Code actual size in mm
            F = 618
            W = size
            D = ((F * W) / P) / 10
            # print 'F = ', F,'W = ', W, 'P = ', P, 'D = ', ((F * W) / P)
            #print 'QR Code scanned:', symbol.data
            #print 'RoboPorter is', "%.2fcm" % D, 'away from ' '%s' % data[0]

            qrDict['visible'] = True
            qrDict['string'] = data[0]
            qrDict['locX'] = int(data[2]) * 50
            qrDict['locY'] = int(data[3]) * 50
            qrDict['distance'] = int(D)
            qrDict['orientation'] = int(data[4])
            qrDict['centre'] = [float(x_centre),float(y_centre)]

            print ("QR Code data = " + str (qrDict))
            if updateEnabled and lastCommand != "r" and lastCommand != "l":
                print ("updating location")
                if qrDict['orientation'] >=0 and qrDict['orientation'] < 90:
                    porterLocation_Global[0] = qrDict['locX'] + qrDict['distance']*numpy.cos(numpy.deg2rad(90 - qrDict['orientation']))
                    porterLocation_Global[1] = qrDict['locY'] + qrDict['distance']*numpy.sin(numpy.deg2rad(90 - qrDict['orientation']))
                    if qrDict['last']  != qrDict['string']:
                        porterOrientation.value = numpy.deg2rad((180+qrDict['orientation'])-360)
                        orientationWheels.value = porterOrientation.value
                elif qrDict['orientation'] >= 90 and qrDict['orientation'] < 180:
                    porterLocation_Global[0] = qrDict['locX'] + qrDict['distance'] * numpy.cos(numpy.deg2rad(qrDict['orientation'] - 90))
                    porterLocation_Global[1] = qrDict['locY'] - qrDict['distance'] * numpy.sin(numpy.deg2rad(qrDict['orientation'] - 90))
                    if qrDict['last']  != qrDict['string']:
                        porterOrientation.value = numpy.deg2rad((180+qrDict['orientation'])-360)
                        orientationWheels.value = porterOrientation.value
                elif qrDict['orientation'] >= 180 and qrDict['orientation'] < 270:
                    porterLocation_Global[0] = qrDict['locX'] - qrDict['distance'] * numpy.cos(numpy.deg2rad(270 - qrDict['orientation']))
                    porterLocation_Global[1] = qrDict['locY'] - qrDict['distance'] * numpy.sin(numpy.deg2rad(270 - qrDict['orientation']))
                    if qrDict['last']  != qrDict['string']:
                        porterOrientation.value = numpy.deg2rad(qrDict['orientation']-180)
                        orientationWheels.value = porterOrientation.value
                elif qrDict['orientation'] >= 270 and qrDict['orientation'] < 360:
                    porterLocation_Global[0] = qrDict['locX'] - qrDict['distance'] * numpy.cos(numpy.deg2rad(qrDict['orientation'] - 270))
                    porterLocation_Global[1] = qrDict['locY'] + qrDict['distance'] * numpy.sin(numpy.deg2rad(qrDict['orientation'] - 270))
                    if qrDict['last']  != qrDict['string']:
                        porterOrientation.value = numpy.deg2rad(qrDict['orientation'] - 180)
                        orientationWheels.value = porterOrientation.value
                else:
                    print ("ERROR in QR code orientation")

            if qrDict['last'] != qrDict['string']:
                qrDict['last'] = qrDict['string']

    if i==0:
        qrDict['visible'] = False

def CameraFunction(vpDict, qrDict,lastQR):
    global vanish
    global vpValid
    global expectedFPS

    frameCount = 0


    profiling = True
    verbose = False
    vanishx = [0,0,0]
    vanishy = [0,0,0]
    varx = [0, 0, 0, 0, 0]
    vary = [0, 0, 0, 0, 0]
    capVid = cv2.VideoCapture(0)

    #capVid = cv2.VideoCapture('testOutsideLab.avi')  # declare a VideoCapture object and associate to webcam, 0 => use 1st webcam

    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    outvid = cv2.VideoWriter('output.avi', fourcc, 5.0, (640/2,480/2))

    # while not capVid.isOpened():
    #     print ("Error Opening Camera")

    if capVid.isOpened() == False:  # check if VideoCapture object was associated to webcam successfully
        print "error: capVid not accessed successfully\n\n"  # if not, print error message
        #logging.error("error: capWebcam not accessed successfully\n\n")
        os.system("pause")
    else:
        centre_point = capVid.read()[1].shape[1] / 4

    while capVid.isOpened() and not exitFlag.value :
        blnFrameReadSuccessfully, img = capVid.read()
        #frameCount += 1

        if True :#frameCount > 8:
            #print "trying to get QR"
            scanQRCode(img,qrDict,lastQR)
            #frameCount = 0


        img = cv2.resize(img, (0,0), fx=0.5, fy=0.5)
        #origimg = img
        #print "frame read"
        startTime = time.time()
        sumTime = 0
        #cv2.imshow("input from cam", img)

        #print "Image Loaded: " + str(startTime)

        # if len(frameNumber) > 0:
        #     frameNumber.append(frameNumber[len(frameNumber) - 1] + 1)
        # else:
        #     frameNumber[0] = 1

        try: #try to find vanishing point
            #print "hough Lines"
            hough_lines, startTime, sumTime = vpLib.hough_transform(img, False, startTime, profiling, verbose)  #calculate hough lines

            if hough_lines: #if lines found
                random_sample = vpLib.getLineSample(hough_lines, 30)  # take a sample of 100 line
                intersections = vpLib.find_intersections(random_sample, img)  # Find intersections in the sample

                if profiling:
                    duration = time.time() - startTime
                    if verbose:
                        print "Intersection Time :" + str(duration)
                    sumTime += duration
                    startTime = time.time()

                #print str(intersections)
                if intersections:  # if intersections are found
                    grid_size[0] = img.shape[0] // 8 #set the grid size to be 20 by 20
                    grid_size[1] = img.shape[1] // 20
                    #find vanishing points
                    vanishing_point = vpLib.vp_candidates(img, grid_size, intersections)
                    #returns the best cell
                    #print ("vanishing_point ", vanishing_point)
                    vanish2, vanishx, vanishy = medianFilter(vanishing_point[0], 3, vanishx, vanishy)
                    varx, vary = varianceFilter(vanishing_point[0], 5, varx, vary)

                    if vpValid.value == True:
                        cv2.circle(img, (vanish2[0], vanish2[1]), 5, (210, 255, 10), thickness=2)
                    else:
                        cv2.circle(img, (vanish2[0], vanish2[1]), 5, (10, 10, 255), thickness=2)

                    vanish.value = int(vanish2[0]-centre_point)
                    vpDict['xValue'] = int(vanish2[0]-centre_point)

                    #print  ("vanishing point = ", vanish.value)
                else:
                    vpValid.value = False
                    vpDict['valid'] = False
            else:
                vpValid.value = False
                vpDict['valid'] = False
            #
            # print "valid vanishing point? " + str(vpValid.value)
            # print "vanishing point = " + str(vanish.value)

            #cv2.imshow('vp Image', img)

            #time.sleep(0.25)


            if profiling:
                duration = time.time() - startTime
                if verbose:
                    print "Finish Time :" + str(duration)
                sumTime += duration
                startTime = time.time()

            if profiling:
                expectedFPS.value = 1/sumTime
                if verbose:
                    print "Expected FPS: " + str(expectedFPS.value)
            else:
                expectedFPS.value = 0

            #fps.append(expectedFPS)
            # plt.plot(frameNumber, fps)
            # plt.pause(0.05)

            if profiling and verbose:
                print "----------------------------------------------"

        except Exception as e:
            pass
            #logging.error("%s", e)

            #time.sleep(1)

        outvid.write(img)

    capVid.release()
    outvid.release()

    cv2.destroyAllWindows()

def medianFilter(vpCoord, n, vanishx, vanishy):
    i = 0
    #global vpValid

    vanish = [0.,0.]
    #print len(vpCoord)
    if len(vpCoord) == 2:
        #print vpCoord
        #print vanishx, vanishy
        vanishx[0] = vanishx[1]
        vanishx[1] = vanishx[2]
        vanishx[2] = vpCoord[0]

        vanishy[0] = vanishy[1]
        vanishy[1] = vanishy[2]
        vanishy[2] = vpCoord[1]

        sortedx = sorted(vanishx)
        sortedy = sorted(vanishy)

        medVanish = (sortedx[1],sortedy[1])


    return medVanish, vanishx, vanishy

def varianceFilter(vpCoord, n, varx, vary):
    global vpValid
    i = 0

    while (len(varx) < n):
        varx.append(0)

    while (len(vary) < n):
        vary.append(0)

    for i in range(0, n - 1):
        varx[i] = varx[i + 1]
        vary[i] = vary[i + 1]

    varx[n - 1] = vpCoord[0]
    vary[n - 1] = vpCoord[1]

    medVar = numpy.var(varx[0:n-1]), numpy.var(vary[0:n-1])

    if (medVar[0] > 1000) or (medVar[1] > 150):
        vpValid.value = False
        vpDict['valid'] = False

    else:
        vpValid.value = True
        vpDict['valid'] = True

    return varx, vary


###---Function Definitions

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
    elif inputCommand[0] == "r":
        return (mSpeed-5), -(mSpeed-5)
    elif inputCommand[0] == "l":
        return -(mSpeed-5), (mSpeed-5)

def get_ip_address(ifname): #Who is this code based on?
    if (platform == "linux") or (platform == "linux2"):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        logging.info("Resolving ip address")
        return socket.inet_ntoa(fcntl.ioctl(
            s.fileno(),
            0x8915,  # SIOCGIFADDR
            struct.pack('256s', ifname[:15])
        )[20:24])
    else:
        logging.error("Not linux... cant find ipAddress")

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


######################################################
######################################################
######Start of the Main Thread!

if __name__ == '__main__':

    multiprocessing.freeze_support()

    mpManager = multiprocessing.Manager()

    wheelSpeeds = mpManager.list([0, 0])
    porterLocation_Global = mpManager.list([3250, 4600])  # set location to "undefined"
    porterLocation_Local = mpManager.list([0, 0])
    porterLocation_IMU = mpManager.list([0,0])

    qrDict = mpManager.dict()
    vpDict = mpManager.dict()

    lastQR =  multiprocessing.Value('c', "-")

    qrDict['visible'] = False
    qrDict['string'] = ""
    qrDict['last'] = ""
    qrDict['locX'] = 0
    qrDict['locY'] = 0
    qrDict['distance'] = 0
    qrDict['orientation'] = 0
    qrDict['centre'] = [0,0]



    logging.info("Starting system...")
    sysRunning = True

    # #Start Speech Process
    # logging.info("Starting speech Process...")
    # speechProcess = multiprocessing.Process(target=porterSpeech,name="Speech Process",args=(speechQueue,exitFlag,))
    # speechProcess.start()
    # processes.append(speechProcess)

    logging.info("Starting speech Thread...")
    speechThread = ttsThread(1, "Speech Thread")
    speechThread.start()
    threads.append(speechThread)
    logging.info('Speech Thread Running - %s', str(speechThread))
    #time.sleep(1)

    # #logging.info("Starting speech Thread...")
    # camThread = CameraThreadClass(2, "cam Thread")
    # camThread.start()
    # threads.append(camThread )

    speechQueue.put("initialising")
    #time.sleep(2)
    #speechQueue.put("trying to run Tracker")

    #Start IMU data Process
    logging.info("Starting Porter Tracker")
    trackerProcess = multiprocessing.Process(name="IMU Process",target=porterTracker, args=(exitFlag,imuEnable, porterLocation_Global, porterOrientation, wheelSpeeds, pulsesQueue,speechQueue,))
    trackerProcess.start()
    processes.append(trackerProcess)
    logging.info('Tracker Process Running - %s', str(trackerProcess))
    speechQueue.put("Tracker Running")
    #time.sleep(1)


    #Start Autopilot thread
    logging.info("Starting Autopilot Thread")
    autoPilotThread = autoPilotThread(2, "Auto Pilot Thread")
    autoPilotThread.start()
    threads.append(autoPilotThread)
    logging.info('Autopilot Thread Running - %s', str(autoPilotThread))
    speechQueue.put("Autopilot Waiting")
    #time.sleep(1)


    # try to run Debug server if the user wants it
    #dataInput = raw_input("Start Debug server... ? (y/n)")
    if Debug_Enable: #dataInput == "y":
        try:
            logging.info("Trying to Run Debug Server")
            debugChannel = debugThread(3, "Debug Thread")
            debugChannel.start()
            threads.append(debugChannel)
            logging.info('Debug Thread Running - %s', str(debugChannel))
            logging.info("Running Debug Server")
            #time.sleep(1)

            # speechQueue.put("Debug Server Running")
        except Exception as e:
            logging.error("%s", str(e))

    # setup serial connection to motor controller
    logging.info("Trying to connect to serial devices")
    #dataInput = raw_input("Connect to motor Controller... ? (y/n)")
    if Motor_Enable: #dataInput == "y": #if user wants to connect to the motor controller...
        dataInput = "" #Reset the variable
        logging.info("Trying to connect to motor controller")
        try: #try to connect
            if (platform == "linux") or (platform == "linux2"):
                MotorConn = serial.Serial('/dev/ttyACM0', 19200)
            elif (platform == "win32"):
                MotorConn = serial.Serial('COM7', 19200)

            logging.info('Connected to Motors %s', str(MotorConn))
            serialThread = motorDataThread(4, "Motor thread")
            serialThread.start()
            threads.append(serialThread)
            logging.info('Motor Thread Running - %s', str(serialThread))

        except Exception as e: #if something goes wrong... tell the user
            logging.error("Unable to establish serial comms to port /dev/ttyACM0")
            logging.error("%s", str(e))

    # Setup serial conn to US controller
    #dataInput = raw_input("Connect Ultrasonic Controller... ? (y/n)")
    if US_Enable: #dataInput == "y":
        dataInput = ""
        logging.info("Trying to connect to Ultrasonic controller")
        try:
            if (platform == "linux") or (platform == "linux2"):
                USConn1 = serial.Serial('/dev/ttyACM1', 19200)
                logging.info("Connected to Ultrasonic sensors at %s", str(USConn1))
                if UShosts == 2:
                    USConn2 = serial.Serial('/dev/ttyACM2', 19200)
                    logging.info("Connected to Ultrasonic sensors at %s", str(USConn2))

            elif (platform == "win32"):
                USConn1 = serial.Serial('COM3', 19200)

            USthread = usDataThread(5, "Ultrasonic thread")
            USthread.start()
            threads.append(USthread)
            logging.info('Ultrasonic Thread Running - %s', str(USthread))
            # speechQueue.put("Ultrasonic Sensors Connected")
        except Exception as e:
            print ('Unable to establish serial comms to US device')
            logging.error("%s", str(e))
            # USConnected = False

    if Cam_Enable:
        logging.info("Starting Cam Process")
        cameraProcess = multiprocessing.Process(name="CAM Process", target=CameraFunction, args=(vpDict, qrDict,lastQR,))
        cameraProcess.start()
        processes.append(cameraProcess)
        logging.info("Camera Process Running - %s", str(cameraProcess))
        speechQueue.put("Camera Process Running")

    #Start PID thread
    logging.info("Starting PID Thread")
    PIDThread = PIDThreadClass(6, "PID Pilot Thread")
    PIDThread.start()
    threads.append(PIDThread)
    logging.info("PID Thread Running - %s", str(PIDThread))

    # speechQueue.put("Setup Complete")

    # chose input method
    dataInput = "n" #raw_input("Local Comms...? (y/n)")
    if dataInput == "n":
        logging.debug("Trying to Setup Remote control")
        localCtrl = False
        # set the server address and port
        logging.info("Setting up sockets...")
        try:
            HOST = ""  # socket.gethostbyname(socket.gethostname()) #socket.gethostname()
            PORT = 5002

            # create a socket to establish a server
            logging.info("Binding the socket...")
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.bind((HOST, PORT))

            # listen to incoming connections on PORT
            logging.info("Socket opened at %s listening to port %s", HOST, PORT)
            s.listen(1)
        except Exception as e:
            # print ("EXCEPTION trying to open Socket - " +str(e))
            logging.error("%s", str(e))
        finally:
            # print ("Local Control")
            logging.debug("Local Control")
            #localCtrl = True
    else:
        logging.debug("Local Control mode")

    # Main Control Loop
    while sysRunning: #while the main loop is not in shutdown mode...
        logging.debug("System is running")
        if not localCtrl: #if remote control, wait for client to connect.
            # for each connection received create a tunnel to the client
            logging.info("Ready for a new client to connect...")
            clientConnection, address = s.accept()
            logging.info('Connected by %s', address)
            print 'Connected by', address

            # send welcome message
            print ("Sending welcome message...")
            clientConnection.send('Connection ack')
            dataInput = clientConnection.recv(1024)
            print ("Client says - " + dataInput)
            dataInput = ""

        cmdExpecting = True

        while cmdExpecting:
            print ("Motor commands are expecting...")
            if not localCtrl:
                dataInput = clientConnection.recv(1024) #maybe make this non blocking?
            else:
                dataInput = raw_input("Please Enter Commands on local terminal...\n")
            if dataInput == "e": #shutdown command server
                cmdExpecting = False
                break
            elif dataInput == "q": #initiate system shutdown
                cmdExpecting = False
                sysRunning = False
            else:
                logging.info("Input Command = %s", dataInput)
                if len(dataInput) > 0: #if more than one character read...
                    if (dataInput[0] == "f" or dataInput[0] == "b" or dataInput[0] == "r" \
                                or dataInput[0] == "l" or dataInput[0] == "x" or dataInput[0] == "m") and not autoPilot: #simple commands
                        logging.info("Valid Command")

                        with threadLock:
                            lastCommand = dataInput[0]
                            # commandToTrans()
                            speedVector = cmdToSpeeds(dataInput)
                            setSpeedVector = copy.deepcopy(speedVector)
                            dataReady = True

                        if lastCommand == "m":
                            logging.info("MANUAL OVERRIDE!\n")

                    elif dataInput[0] == "s" and not autoPilot: #Toggle safety
                        if safetyOn:
                            dataInput = raw_input("Do you solemnly swear you're up to no good!?..")

                            if len(dataInput) > 0 and dataInput[0] == "Y":
                                print ("May the force be with you... you are going to need it...")
                                print ("(just don't tell Ms Webster about it... >,< )")
                                safetyOn = False
                        else:#if safety is off...turn it on
                            safetyOn = True
                            print ("Mischief managed ;)")

                        dataInput = ""
                        if dataReady != False:
                            with threadLock:
                                dataReady = False

                    elif dataInput[0] == "a": # Engage/Disengage Auto Pilot
                        if autoPilot:
                            logging.info("Turning OFF Autopilot")
                            speechQueue.put("Turning Off Autopilot")
                            autoPilot = False
                            with threadLock:
                                lastCommand = "x"
                                # commandToTrans()
                                speedVector = cmdToSpeeds("x")
                                dataReady = True
                        else:
                            logging.info("Turning ON Autopilot. Send letter 'a' for emergency stop")
                            speechQueue.put("Turning On Autopilot")
                            autoPilot = True
                            safetyOn = True
                            #targetDestination = [-100, 100]  # go to global XY = -100,100
                            if dataReady != False:
                                with threadLock:
                                    dataReady = False
                    elif dataInput[0] == "n" :
                        nodeQueue.put(cmdToDestination(dataInput))
                        logging.info("Current Node list = %s", nodeQueue)

                        # if not autoPilot:
                        #     logging.info("Turning ON Autopilot. Send letter 'a' for emergency stop")
                        #     speechQueue.put("Turning On Autopilot")
                        #     autoPilot = True
                        #     safetyOn = True
                        #     logging.info("Setting Destination")
                        #     with threadLock:
                        #
                        #         #targetDestination =
                        #     print ("Destination is " + str(targetDestination))
                        #     if dataReady != False:
                        #         with threadLock:
                        #             dataReady = False

                    elif dataInput[0] == "o":
                        if not autoPilot:
                            logging.info("Resetting Orientation and location")
                            orientationWheels.value = 0
                            porterLocation_Global[0] = 0
                            porterLocation_Global[1] = 0
                        else:
                            logging.info("Cant reset localisation. Autopilot engaged.")
                    elif dataInput[0] == "p":
                        print "Porter Location is " + str(porterLocation_Global)
                        print "Porter Orientation is " + str(porterOrientation)
                        print "Target is " + str(targetDestination)
                        print "US sensors is " + str(USAvgDistances)

                    else:
                        if dataReady != False:
                            with threadLock:
                                dataReady = False
                        logging.info("Invalid Command")


            dataInput = ""
            print ("")

        if not localCtrl and clientConnection:
            # shut down the server
            clientConnection.close()
            logging.info("Client at %s closed the connection", str(address))

        else: #
            print ("Use q to shutdown system... ")
            print ("Looping back to the start...")

    with threadLock:
        lastCommand = "x"
        speedVector = [0, 0]
        dataReady = True

    exitFlag.value = True #instruct all the threads to close

    if not localCtrl:
        logging.info("Shutting down the server at %s...", HOST)
        s.close()
    else:
        logging.info("Shutting Down")

    logging.info("Waiting for threads to close...")
    for t in threads:
        logging.info("Closing %s thread", t)
        t.join()

    for p in processes:
        logging.info("Closing %s process", p)
        p.join()

    # print "processes = " + str(processes)
    # print "Threads = " + str(threads)

    logging.info("Exiting main Thread... BYEEEE")

#######END of Program