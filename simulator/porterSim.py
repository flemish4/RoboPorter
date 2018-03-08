import sys, pygame
import time
import cv2
import numpy as np
import pickle
import glob
import math
import random 
from bresenham import bresenham
import multiprocessing
import threading

# SLAM imports


from breezyslam.algorithms import Deterministic_SLAM, RMHC_SLAM

from roboPorterTestFunctions import roboPorterLaser
from pgm_utils import pgm_save



# Porter global variables
global USAvgDistances
global wheelSpeeds
global obstruction
global maxSpeeds
global pathMap
global porterLocation #vector holding local location [x,y] in cm
global porterOrientation #angle from north (between -180,180) in degrees
global porterOrientationUnConst #angle from north no constraint
global lastCommand
global speedVector
global dataReady
global manualControl
global threadLock
global exitFlag
global USThreashholds
global stoppingDistance
global lidarRun
global lidarAngles
global lidarAnglesList
global lidarMap
global lidarReady
global porterImuOrientation
global realPorterSize
global realPorterRadius
global dataMap
global realDataMap
global wheelError
global navMap
global totalR
global totalWd
global encoderVectorAbs
global encoderVectorAbsReady
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
encoderVectorAbsReady = False
lidarPolarList = []
navMap = np.zeros(SLAMOffset)
detailedMap = np.zeros(SLAMOffset)
wheelError          = 0
totalR          = 0
totalWd          = 0
manControl          = False    
dataMap             = set()
realDataMap         = set()
exitFlag            = False #multiprocessing Exit flag
pathMap             = {}
wheelSpeeds         = [0,0]    
lidarReady          = True
lidarAngles         = {}
lidarAnglesList     = []
lidarMap            = set()
lidarRun            = ""
obstruction         = False
lastCommand         = ""
speedVector         = [0, 0]
dataReady           = False
threadLock = threading.Lock()
USThreashholds      = {"front":30,"back":30,"side":20}
stoppingDistance    = 20
porterOrientation   = 0
porterOrientationUnConst = 0
porterImuOrientation = porterOrientation
realPorterSize      = (73,70)
realPorterRadius    = math.sqrt(realPorterSize[0]**2+realPorterSize[1]**2)
realPorterWheelOffsetY = realPorterSize[1]/4

# RealPorter global variables
global realObstruction
global realCollision
global realPorterLocation
global realPorterOrientation
global simThread
realPorterOrientation = 0
realObstruction         = False
realCollision         = False

def getRange(x,y) : # Library export
    if x > y :
        numRange = reversed(range(int(round(y)), int(round(x)+1)))
    else :
        numRange = range(int(round(x)), int(round(y)+1))
    return numRange


def rotate(origin, point, angle): # Library export
    """
    Rotate a point counterclockwise by a given angle around a given origin.

    The angle should be given in radians.
    """
    ox, oy = origin
    px, py = point

    qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
    qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
    return (qx, qy)


def constrainAngle360(angle, max, min) : # Library export
    while angle >= max :
        angle -= 360
    while angle < min :
        angle += 360
    return angle
    
    
def constrainAngle180(angle, max, min) :
    while angle >= max :
        angle -= 180
    while angle < min :
        angle += 180
    return angle    
    
    
# def constrainInt(i, max, min) :
    # while i >= max :
        # i -= max
    # while i < min :
        # i += max
        
    # # REVISIT : solution above is dumb but I'm not changing to below until I can test it
    # # if i >= max :
        # # return max -1
    # # if i < min :
        # # return min
        
    # return i   
    
def constrainFloat(d, max, min) :
    if d >= max :
        return max
    if d < min :
        return min
    return d

  
def roundBase(x, base):
    return int(base * round(float(x)/base))
     
     
def printVars(vars, title="") :
    oStr = title + "::: "
    for name, data in vars.iteritems() :
        oStr += str(name) + ": " + str(data) + ", "
        
    print oStr
    
    
# Returns length of line given in format [x0, y0, x1, y1]
def getLength(line) :
    return math.sqrt((line[2]-line[0])**2+(line[3]-line[1])**2)   
    
# Returns length^2 of line given in format [x0, y0, x1, y1]
def getLengthSquared(line) : 
    return (line[2]-line[0])**2+(line[3]-line[1])**2

class MultiThreadBase(threading.Thread): #Parent class for threading
    def __init__(self, threadID, name): #Class constructor
        threading.Thread.__init__(self) #Initiate thread
        self.threadID = threadID #Thread number
        self.name = name #Readable thread name


class Button(pygame.sprite.Sprite):
    """Class used to create a button, use setCords to set 
        position of topleft corner. Method pressed() returns
        a boolean and should be called inside the input loop."""
    def __init__(self, s, font, text, colour, x, y, w, h):
        pygame.sprite.Sprite.__init__(self)
        self.bSurface = pygame.Surface((w, h))
        self.bSurface.fill(colour)
        self.rect = self.bSurface.get_rect().move(x, y)
        text1 = font.render(text, True, (255,255,255))
        self.bSurface.blit(text1, (5, 3))
        s.blit(self.bSurface, self.rect)

    def pressed(self,mouse):
        if mouse[0] > self.rect.topleft[0]:
            if mouse[1] > self.rect.topleft[1]:
                if mouse[0] < self.rect.bottomright[0]:
                    if mouse[1] < self.rect.bottomright[1]:
                        return True
                    else: return False
                else: return False
            else: return False
        else: return False 
        

class porterSim() :
    def __init__(self): 
        global exitFlag
        global threadLock
        global realPorterSize
        pygame.init()
        pygame.font.init()  
        self.simSpeed           = 1 # Scale time 
        self.simFrameTime       = 0.05 
        self.simFrameRate       = 1/self.simFrameTime*self.simSpeed
        self.scale              = 2 # 1pixel = 2cm
        self.wheelSpeedError    = 0.05
        self.orientationError   = 0.5
        self.encoderError       = 0.05
        self.lidarError         = 0.02
        self.pixelPorterSize    = (realPorterSize[0]/self.scale,realPorterSize[1]/self.scale)
        self.font               = pygame.font.SysFont('Arial', 20)
        self.winX, self.winY    = 1600, 843
        self.views          =    {  "realmap"   :   {   "relSize": (0.5,0.95),
                                                        "colour" : pygame.Color(245,245,245),
                                                        "relPos" : (0,0.05),
                                                },
                                    "portermap" :   {   "relSize": (0.5,0.95),
                                                        "colour" : pygame.Color(230,230,230),
                                                        "relPos" : (0.5,0.05),
                                                }
                            }
        for name, view in self.views.iteritems():
            view["absSize"] = (int(round(view["relSize"][0]*self.winX)),int(round(view["relSize"][1]*self.winY)))
            view["absPos"]  = (int(round(view["relPos"][0]*self.winX)),int(round(view["relPos"][1]*self.winY)))

        self.tileRealW          = 20 # Real tile width(=height) in cm
        self.tileW              = self.tileRealW / self.scale # tile size in pixels, 2 pixels per square
        self.nTileX             = int(round(math.floor(self.views["realmap"]["absSize"][0]/self.tileW))) # number of tiles in x
        self.nTileY             = int(round(math.floor(self.views["realmap"]["absSize"][1]/self.tileW))) # number of tiles in y
        self.tilePorterSize     = (self.pixelPorterSize[0]/self.tileW,self.pixelPorterSize[1]/self.tileW)
        self.longestR           = math.sqrt((self.nTileX*self.tileW)**2+(self.nTileY*self.tileW)**2) # Length of a line that will always cover the whole map for lidar use
        self.runMode            = ""
        self.nextRunMode        = ""
        self.lidarRpm           = 1 #0.66 # This is approximate as per the calculation of self.lidarNSamples
        self.lidarTotalNSamples = 360 #767
        self.lidarNSamples      = 18 #int(round(math.ceil(self.lidarTotalNSamples*self.lidarRpm*self.simFrameTime))) # This calculates the number of lidar samples to be taken each frame - it rounds up so the realRPM will be slightly higher than given 
        #self.lidarTotalNSamples = int(round(self.lidarNSamples * self.simFrameRate))
        #print self.lidarNSamples
        #print self.lidarTotalNSamples
        self.porterAdded        = False # Has the porter been placed at the start of simRun
        self.simRunning         = False # Is the simulation running
        self.draw_on            = 0 # Is drawing enabled in mapBuilder
        self.lidarPos           = 0     # Stores lidar position so that it can continue scanning next frame
        self.black              = 0,0,0 # The colour black
        self.map                = set() # A set of tuples each (x,y) of a wall tile  
        self.fineMap            = set() # A set of tuples each (x,y) of a wall pixel hence fine
        self.rectMap            = []    # A list of rects(pygame object) each equivilent to a tile in self.map - used for collision detection
        with threadLock :
            exitFlag          = False # Is the current task stopping - allows variables to be reset when ending sim etc.
        
        #print "scale: " + str(self.scale) + "cm/p, tileW: " + str(self.tileW) + "p, tileRealW: " + str(self.tileRealW) + "cm, nTileX: " + str(self.nTileX) + ", nTileY: " + str(self.nTileY) + ", maxX: " + str(self.scale*self.tileW*self.nTileX/100) + "m, maxY: " + str(self.scale*self.tileW*self.nTileY/100) + "m" 
        
        self.simClock           = pygame.time.Clock()
        self.genBlankMap()
        self.screen             = pygame.display.set_mode((self.winX, self.winY))
        self.screen.fill(self.black)
        self.createViews()
        self.createMenuButtons()
    

    def createMenuButtons(self) :
        # Create menu
        self.menuButtons = {    "mapBuilder" :  {"button" : Button(self.screen, self.font, "mapBuilder", (100,0,0), 0, 0, 100, int(round(0.05*self.winY))),
                                                },
                                "selectmap"  :  {"button" : Button(self.screen, self.font, "selectmap", (0,100,0), 100, 0, 100, int(round(0.05*self.winY))),
                                                },
                                "runMapSim"  :  {"button" : Button(self.screen, self.font, "runMapSim", (0,0,100), 200, 0, 100, int(round(0.05*self.winY))),
                                                },
                                "runNavSim"  :  {"button" : Button(self.screen, self.font, "runNavSim", (0,100,100), 300, 0, 100, int(round(0.05*self.winY))),
                                                },
                                "stop"       :  {"button" : Button(self.screen, self.font, "stop", (255,0,0), 400, 0, 100, int(round(0.05*self.winY))),
                                                },
                            }
          
          
    def clearMainMenu(self) :
        # Draw over the existing menu items 
        pygame.draw.rect(self.screen, (0,0,0), [0,0,self.winX,0.05*self.winY])
        # Remove the menu button objects
        for name, button in self.menuButtons.iteritems() :
            try :
                button["button"].kill()
                del button["button"]
            except KeyError :
                pass

                
    def drawMap(self) :
        # Draw faint grid
        for nx in range(0, self.nTileX+1) :
            pygame.draw.line(self.views["realmap"]["surface"], (230,230,230), (nx*self.tileW,0), (nx*self.tileW, self.tileW*self.nTileY))
        for ny in range(0, self.nTileY+1) :
            pygame.draw.line(self.views["realmap"]["surface"], (230,230,230), (0,self.tileW*ny), (self.tileW*self.nTileX, self.tileW*ny))
        
        # Iterate over self.map and fill squares as required
        for tile in self.map :
            self.views["realmap"]["surface"].fill(self.black, pygame.Rect(self.tileW*tile[0],self.tileW*tile[1],self.tileW,self.tileW))

            
    def createViews(self) :
        for name, view in self.views.iteritems() :
            view["surface"] = pygame.Surface((view["absSize"][0], view["absSize"][1]))
            view["surface"].fill(view["colour"])
            view["rect"]    = view["surface"].get_rect().move(view["absPos"])
        
        self.drawMap()

        
    def addTile(self, e) :
        tile = ((e.pos[0] + self.views["realmap"]["absPos"][0]) / self.tileW, (e.pos[1] - self.views["realmap"]["absPos"][1]) / self.tileW)
        if tile[0] < self.nTileX and tile[0] >= 0 and tile[1] < self.nTileY and tile[1] >= 0 :
            if self.draw_on == 1 :
                # add tile to map
                self.map.add(tile)

            elif self.draw_on == 2 :
                # remove tile
                self.map.discard(tile)


    def genFineMap(self) :
        self.fineMap = set()
        for tile in self.map :
            for x in range(tile[0]*self.tileW, (tile[0]*self.tileW) + self.tileW) :
                for y in range(tile[1]*self.tileW, (tile[1]*self.tileW) + self.tileW) :
                    self.fineMap.add((x,y))            
            
    
    def genRectMap(self):
        self.rectMap = []
        for tile in self.map :
            self.rectMap.append(pygame.Rect(self.tileW*tile[0],self.tileW*tile[1],self.tileW,self.tileW))
    
    
    def genBlankMap(self) :
        # Generate a blank map with a boarder around the edge
        self.map = set()
        for x in range(0,self.nTileX) :
            self.map.add((x,0))
            self.map.add((x,self.nTileY-1))
            
        for y in range(0,self.nTileY) :
            self.map.add((0,y))
            self.map.add((self.nTileX-1,y))
            
        self.genRectMap()
        self.genFineMap()
    
    
    def mapBuilder(self, e) :
        global exitFlag
        global threadLock
        saveQuit = False
        if exitFlag == False :
            if self.firstRun : 
                self.createViews()
                self.firstRun = False
            if e.type == pygame.MOUSEBUTTONDOWN:
                if e.button == 1 :
                    tile = ((e.pos[0] + self.views["realmap"]["absPos"][0]) / self.tileW, (e.pos[1] - self.views["realmap"]["absPos"][1]) / self.tileW)
                    if tile in self.map :
                        self.draw_on = 2
                    else :
                        self.draw_on = 1
                    self.addTile(e)
                if e.button == 2 :
                    saveQuit = True
                    
            if e.type == pygame.KEYDOWN :
                if e.key == pygame.K_s :
                    saveQuit = True
                    
            if saveQuit :
                # SAVE and quit
                with open(time.strftime("%Y-%m-%d %H%M%S", time.gmtime()) + '.map', 'wb') as f:
                    pickle.dump(self.map, f)
                self.runMode = ""
                self.genFineMap()
                self.genRectMap()
                self.draw_on = 0
            if e.type == pygame.MOUSEBUTTONUP:
                self.draw_on = 0
            if e.type == pygame.MOUSEMOTION:
                if self.draw_on:
                    self.addTile(e)
        else :
            self.runMode = self.nextRunMode
            with threadLock:
                exitFlag = False
            self.genRectMap()
            self.genFineMap()
            self.firstRun = True
        self.createViews()
            
            
    def selectmap(self) :
        options = ["Current", "Blank"]
        fileNames = glob.glob("*.map")
        
        options = options + fileNames
        self.clearMainMenu()
        # Create temporary menu
        self.menuButtons = {}
        n = len(options)
        for i,option in enumerate(options)  :
            self.menuButtons[option] = {"button" : Button(self.screen, self.font, option, (255*i/n,255-(255*i/n),255+(100*-i/n)), i*150, 0, 150, int(round(0.05*self.winY))) }
        
        pygame.display.flip()  
        done = False
        while not done:
            e = pygame.event.wait()
            #Check for quit event
            if e.type == pygame.QUIT:
                quit()
                
            # Check for menu event
            if e.type == pygame.MOUSEBUTTONDOWN:
                if e.pos[1] < self.winY*0.05 :
                    #must have clicked in menuBar
                    for name, button in self.menuButtons.iteritems() :
                        if button["button"].pressed(e.pos) :
                            # This button was clicked!
                            selectedMap = name
                            done = True

        self.clearMainMenu()
        self.createMenuButtons()
        
        #load 
        if selectedMap == "Blank" :
            self.genBlankMap()
        elif selectedMap != "Current" :
            try: 
                with open(selectedMap, "r") as f :
                    self.map = pickle.load(f)
            except IOError : 
                pass
        
        self.createViews()
        self.genRectMap()
        self.genFineMap()
                    

    def rot_center(self, image, angle):
        #rotate an image while keeping its center and size - doesn't work perfectly REVISIT 
        loc = image.get_rect().center  #rot_image is not defined 
        rot_sprite = pygame.transform.rotate(image, angle)
        rot_sprite.get_rect().center = loc #(x,y)
        return rot_sprite

        
    def createPorter(self, s, r, x, y, porterOrientation) :
        porter = pygame.Surface(self.pixelPorterSize)
        porter.fill(123456)
        porter.set_colorkey((255,255,255))
        pygame.draw.rect(porter, (255,0,0), [0,0,self.pixelPorterSize[0],0.15*self.pixelPorterSize[1]])
        return porter
        
        
    def undrawPorter(self, s, r, x, y, porter) :
        # REVISIT : Perhaps this should draw over the current location of the porter
        self.createViews()
        
        
    def drawPorter(self, s, r, x, y, porterOrientation, porter) :
        porter = self.rot_center(porter, -porterOrientation)
        porterRect = porter.get_rect()
        porterRect.move_ip(x,y)
        s.blit(porter, porterRect) 
    
    def checkPorterCollision(self) :
        global realCollision 
        global realPorterLocation
        porterRect = self.porterReal["surface"].get_rect()
        porterRect = porterRect.move((realPorterLocation[0]-(self.pixelPorterSize[0]/2),realPorterLocation[1]-(self.pixelPorterSize[1]/2)))
        with threadLock :
            if porterRect.collidelist(self.rectMap) != -1 :
                realCollision = True
            else :
                realCollision = False

        return realCollision
        
    def checkPorterObstruction(self) :
        global realObstruction
        global obstruction
        # REVISIT : Unfortunately a square collision box is the only reasonable way to achieve this using pygame
        #           i.e. if porter is rotated the bounding box will be bigger than it should be
        porterRect = self.porterReal["surface"].get_rect()
        porterRect = porterRect.move(realPorterLocation)
        porterRect.inflate_ip(2*USThreashholds["front"]/self.scale, 2*USThreashholds["front"]/self.scale)
        
        with threadLock :
            if porterRect.collidelist(self.rectMap) != -1 :
                realObstruction = True
                #print "Real Obstruction: " + str(realObstruction)
            else :
                realObstruction = False
    
        # REVISIT : is this representitive?
        obstruction = realObstruction
    
        return realObstruction

    
    def getLidarSample(self, angle) : 
        # Sets lidarAngles{angle} to the distance of the nearest collision at that angle relative to the porterOrientation
        # Also adds a coordinate to lidarMap if a point is detected (which it will be < 40m)
        global realPorterLocation
        global realPorterOrientation
        global realPorterSize
        global lidarMap
        global lidarAngles
        global lidarAnglesList
        global lidarPolarList
        # Generate a list of points on line going from centre of porter out at angle given
        linePoints = list()
        # Centre coordinates of porter
        x0 = int(round(realPorterLocation[0]))
        y0 = int(round(realPorterLocation[1]))
        # Absolute angle
        a    = constrainAngle360(realPorterOrientation + angle - 90, 360,0)
        # End points of line 
        xMax = x0 + int(round(self.longestR*math.cos(math.radians(a))))
        yMax = y0 + int(round(self.longestR*math.sin(math.radians(a))))
        # List of points along line
        linePoints = list(bresenham(x0,y0,xMax, yMax))

        # Find collision location - note even if no colission it will return the end of the line
        for point in linePoints :
            if point in self.fineMap :
                break
                
        # Draw lidar line on realMap
        pygame.draw.line(self.views["realmap"]["surface"],self.black,(x0,y0),point)
              
        # Calculate location relative to porter (not realPorter)
        startPos = (porterLocation[0], porterLocation[1])
        r = getLength([point[0],point[1],x0,y0])*self.scale # math.sqrt((point[0]-x0)**2+(point[1]-y0)**2)*self.scale
        porterAbsAngle = constrainAngle360(porterOrientation -90 + angle, 360, 0)
        endPos = (int(round(startPos[0] + r*math.cos(math.radians(porterAbsAngle)))), int(round(startPos[1] + r*math.sin(math.radians(porterAbsAngle)))))
        
        if r <4000 :  # If within 40m which it definitely will be...
            with threadLock :
                lidarAngles[angle] = r
                lidarAnglesList.append(r)
                lidarPolarList.append({"angle": angle, "dist" : r })
                lidarMap.add((endPos[0],endPos[1]))
            # Draw lidar line and hit point on portermap
            pygame.draw.circle(self.views["portermap"]["surface"],self.black,[n/self.scale for n in endPos],5)
            pygame.draw.line(self.views["portermap"]["surface"], self.black, [n/self.scale for n in startPos], [n/self.scale for n in endPos])
        
            
    def checkLidar(self) :
        global lidarRun
        global lidarReady
        global lidarMap
        global lidarAngles
        global lidarAnglesList
        global lidarPolarList
        global adjustedLidarStore
        #f = open("checkLidar.log", "a+")
        #f.write("New#########################\n")
        if lidarRun == "a":
            # Start 360 scan
            self.lidarPos = -180
            with threadLock :
                lidarReady  = False
                lidarMap    = set()
                lidarAngles = {}
                lidarAnglesList = []
                lidarPolarList = []
        elif lidarRun == "s" :
            # Start 360 scan
            self.lidarPos = -180
            with threadLock :
                lidarReady  = True
                lidarMap    = set()
                lidarAngles = {}
                lidarAnglesList = []
                lidarPolarList = []
                adjustedLidarStore = []
        elif (len(lidarRun)>1) and (lidarRun[0] == "a") :
            # Get single sample
            with threadLock :
                lidarAngles = {}
                #lidarAnglesList = []
                lidarMap    = set()
            try :
                angle = int(round(lidarRun[1:]))
                self.getLidarSample(angle)
            except TypeError:
                pass
            
        if not lidarReady : # must have more samples to get in 360 scan
            # Angle delta
            #angle_delta = float(360) / self.lidarTotalNSamples
            # Get 360 samples
            endPos = self.lidarPos + self.lidarNSamples*angle_delta # int(round(self.lidarPos + self.lidarNSamples*angle_delta))
            if endPos >= 180 :
                endPos = 180
                with threadLock :
                    lidarReady = True # REVISIT : this isn't right for real time
            
            for angle in np.arange(self.lidarPos,endPos, angle_delta) :
                #f.write(str(angle) + "\n")
                self.getLidarSample(angle)
        
            if lidarReady :
                lidarPolarList.append("END")
                #f.write("END\n")
                with threadLock :
                    lidarRun = "a"
                
            self.lidarPos = endPos
        with threadLock :
            lidarRun = "" # stop running?
            
        if lidarReady :
            with threadLock :
                lidarRun = "a"
        #f.close()
                
    def adjustLidar(self) :
        global encoderVectorAbs
        global encoderVectorAbsReady
        global lidarPolarList
        global adjustedLidarStore
        global adjustedLidarList
        global adjustedLidarListReady
        global dataMap
        if encoderVectorAbsReady and len(lidarPolarList) >0 : # this should always be true at this point in simulation
            # allows lidar to continue getting data during this function
            with threadLock :
                lidarData = list(lidarPolarList)
                lidarPolarList = []
        
            # REVISIT : use actual time values in the real porter although it shouldn't generally make much difference...
            sampleTime = self.simFrameTime / len(lidarData)
            sampleTimeFraction = sampleTime / self.simFrameTime
            distFraction = (encoderVectorAbs[0]*sampleTimeFraction,encoderVectorAbs[1]*sampleTimeFraction)
            
            for i, sample in enumerate(lidarData) :
                # do calculations else if sample is end sample then push all stored calculated values into output and set ready 
                if sample != "END" :
                    # convert coordinate to x y 
                    sampleXY = (sample["dist"]*math.cos(math.radians(sample["angle"]-90)),sample["dist"]*math.sin(math.radians(sample["angle"]-90)))
                    # add motion i*distFraction
                    j = len(lidarData) - i # adjust samples to current position
                    #sampleXY = (sampleXY[0]+distFraction[0]*j,sampleXY[1]+distFraction[1]*j)
                    #dataMap.add((int(sampleXY[0]+porterLocation[0]), int(sampleXY[1]+porterLocation[1])))
                    # convert back to polar # REVISIT : ew square root
                    r = math.sqrt(sampleXY[0]**2 + sampleXY[1]**2)
                    # save to store
                    adjustedLidarStore.append(r*10)
                else : 
                    with threadLock :
                        #dataMap = set()
                        adjustedLidarList = adjustedLidarStore[-self.lidarTotalNSamples-1:]
                        #with open("adjustLidar.log","a+") as f:
                        #    f.write("new\n" + str(adjustedLidarList) + "\n" + str(adjustedLidarStore)+"\n")
                        adjustedLidarStore = []
                        adjustedLidarListReady = True
            

            with threadLock :
                encoderVectorAbsready = False
        

     
    def runSLAM(self):
        global adjustedLidarList
        global adjustedLidarListReady
        global SLAMObject
        global SLAMRunning
        global porterLocation
        global porterOrientation
        global porterOrientationUnConst
        global threadLock
        
        self.calculatePorterPosition()
        self.adjustLidar()
        if SLAMRunning and adjustedLidarListReady:
            SLAMObject.update(adjustedLidarList) #, (dxy*20, globalWd, 0.015 ))
            
            # update position
            with threadLock: 
                x, y, newPorterOrientation = SLAMObject.getpos()
                newPorterOrientation = constrainAngle360(newPorterOrientation, 180, -180)
                dOrientation         = constrainAngle360(newPorterOrientation-porterOrientation, 180, -180)
                porterOrientationUnConst += dOrientation
                porterOrientation = newPorterOrientation
                xSim =  y/20#((y-SLAMOffset[1])/(25*0.5))+SLAMSimOffset[0]
                ySim =  1250 -x/20#-((x-SLAMOffset[0])/(25*0.5))+SLAMSimOffset[1](12500*2 -
                porterLocation = (xSim,ySim)
                #dataMap.add((int(round(xSim)),int(round(ySim))))
                adjustedLidarListReady = False

            pathMap.updateMap()
            #print("NEW ------------------------ " )    
            #print("realPos: " + str(realPorterLocation))
            #print("SLAMPos: " + str((x,y)))
            #print("location: " + str(porterLocation))
            #print("orientat: " + str(porterOrientation))
            #print("orientun: " + str(porterOrientationUnConst))
            with open("runSLAM.log", 'a+') as f:
                f.write(str(realPorterLocation) + "," + str((x,y)) + "," + str(porterLocation))

            
     
    def drawLidarGrid(self) :
        global lidarMap
        global threadLock
        with threadLock : 
            for point in lidarMap :
                pygame.draw.circle(self.views["portermap"]["surface"], self.black, (point[0]/self.scale,point[1]/self.scale), 1)
    
    def drawNavMap(self) :
        global navMap
        global threadLock
        porterLocationInt = (int(round(porterLocation[0])), int(round(porterLocation[1])))
        with threadLock : 
            #tempMap = np.array(navMap)
            tempMap = detailedMap
        for i in range(0,20) :
            for j in range(0,20) :
                try:
                    tempMap[porterLocationInt[1]-10+i][porterLocationInt[0]-10+j] = 200
                except :
                    pass
                    
        with threadLock:
            for point in dataMap:
                for i in range(0,6) :
                    for j in range(0,6) :
                        try:
                            tempMap[point[1]-3+i][point[0]-3+j] = 100
                        except :
                            pass

        imS = cv2.resize(tempMap, (900, 900)) 
        cv2.imshow('navMap', imS)
        cv2.waitKey(1)
    
    def drawDataMap(self) :
        global dataMap
        global threadLock
        pass
        #with threadLock :
            #for point in dataMap :
                #pygame.draw.circle(self.views["portermap"]["surface"], (255,0,0), (point[0]/self.scale,point[1]/self.scale), 1)
    
    def drawRealDataMap(self) :
        global realDataMap
        global threadLock
        with threadLock :
            for point in realDataMap :
                pygame.draw.circle(self.views["realmap"]["surface"], (0,0,255), (point[0],point[1]), 2)
    
    
    def placePorter(self, e) : # UI for placing porter when starting sim
        global realPorterLocation
        global porterLocation
        global realPorterOrientation
        global porterOrientation
        global porterOrientationUnConst
        global porterImuOrientation
        global SLAMSimOffset
        startSim = False
        # Configure variables if first loop
        if self.firstRun :
            self.firstRun = False
            self.porter = {}
            self.porterReal = {}
            with threadLock :
                porterOrientation = 0
                porterOrientationUnConst = 0
                porterImuOrientation = porterOrientation
                realPorterOrientation = 0
                porterLocation = (0,0)
                SLAMSimOffset = porterLocation
                realPorterLocation = (0,0)
        # If sim hasn't started - allow porter to be placed
        if not self.simRunning :    
            if e.type == pygame.MOUSEBUTTONDOWN:
                if e.button == 1 :
                    # Check that the porter is within the bounds of the realmap
                    if e.pos[0]>self.views["realmap"]["absPos"][0] and e.pos[1]>self.views["realmap"]["absPos"][0] \
                        and e.pos[0] < (self.views["realmap"]["absPos"][0] + self.views["realmap"]["absSize"][0]) \
                        and e.pos[1] < (self.views["realmap"]["absPos"][1] + self.views["realmap"]["absSize"][1]) :
                        # Calculate new porter position
                        with threadLock :
                            realPorterLocation          = (e.pos[0]+self.views["realmap"]["absPos"][0], e.pos[1]- self.views["realmap"]["absPos"][1])
                            porterLocation              = (realPorterLocation[0]*self.scale, realPorterLocation[1]*self.scale)
                            SLAMSimOffset = porterLocation
                        # Create new porter
                        self.porterReal["surface"]  = self.createPorter(self.views["realmap"]["surface"], self.views["realmap"]["rect"], realPorterLocation[0], realPorterLocation[1], porterOrientation)
                        self.porter["surface"]      = self.createPorter(self.views["portermap"]["surface"], self.views["portermap"]["rect"], porterLocation[0], porterLocation[1], porterOrientation)
                        
                        # Check that the porter does not intersect a wall
                        if not self.checkPorterCollision() :
                            self.porterAdded = True
                        else :
                            # Else remove porters
                            self.porterAdded = False
                            self.porterReal["surface"] = None
                            self.porter["surface"]  = None
                if e.button == 2 :
                    startSim = True
            
                
                if e.button == 3 :
                    with threadLock :
                        # Rotate porter 90
                        if porterOrientation > 90 :
                            porterOrientation -= 270
                        else :
                            porterOrientation += 90
                        porterOrientationUnConst = porterOrientation
                        porterImuOrientation = porterOrientation    
                        realPorterOrientation = porterOrientation
                    
            if e.type == pygame.KEYDOWN :
                if e.key == pygame.K_s :
                    startSim = True
                    
            if startSim :
                    if self.porterAdded  :
                        self.simRunning = True
                        return True
                    
    def movePorter (self,e) :
        global speedVector
        global manControl
        global threadLock
        if e.type == pygame.KEYDOWN:
            with threadLock :
                if e.key == pygame.K_a:
                    speedVector[0]+=40
                elif e.key == pygame.K_d:
                    speedVector[0]-=40
                elif e.key == pygame.K_w:
                    speedVector[0]+=40
                    speedVector[1]+=40
                elif e.key == pygame.K_s:
                    speedVector[0]-=40
                    speedVector[1]-=40
                elif e.key == pygame.K_e:
                    speedVector[0]=0
                    speedVector[1]=0
                    
                manControl = True    

                
    def adjustError (self,e) :
        global wheelError
        global threadLock
        if e.type == pygame.KEYDOWN:
            with threadLock :
                if e.key == pygame.K_UP:
                    wheelError+=5
                elif e.key == pygame.K_DOWN:
                    wheelError-=5
                elif e.key == pygame.K_e:
                    wheelError=0
                        
    def realMovePorter(self) :
        # May not need all of these
        global realPorterLocation
        global realPorterOrientation
        global porterImuOrientation
        global speedVector
        global wheelSpeeds
        global exitFlag
        global threadLock
        global manControl
        global realPorterWheelOffsetY
        global realDataMap
        
        if not realCollision :
            if (speedVector != [0,0]) or manControl :
                realWheelSpeeds = (wheelError + speedVector[0]*(1 +(self.wheelSpeedError*(0.5-random.random())))/self.scale, speedVector[1]*(1 + (self.wheelSpeedError*(0.5-random.random())))/self.scale)
                
                leftDelta  = self.simFrameTime * realWheelSpeeds[0]
                rightDelta = self.simFrameTime * realWheelSpeeds[1]
                orientation = math.radians(realPorterOrientation - 90)
                x   = realPorterLocation[0] - realPorterWheelOffsetY*math.cos(orientation)/self.scale
                y   = realPorterLocation[1] - realPorterWheelOffsetY*math.sin(orientation)/self.scale
                with threadLock :
                    realDataMap.add((int(round(x)),int(round(y))))
                o = realPorterOrientation # save state incase collision
                if (math.fabs(leftDelta - rightDelta) < 1.0e-6) : # basically going straight
                    new_x = x + leftDelta * math.cos(orientation);
                    new_y = y + rightDelta * math.sin(orientation);
                    new_heading = orientation;
                else :
                    R = self.pixelPorterSize[0] * (leftDelta + rightDelta) / (2 * (rightDelta - leftDelta))
                    wd = (rightDelta - leftDelta) / self.pixelPorterSize[0];

                    new_x = x + R * math.sin(wd + orientation) - R * math.sin(orientation);
                    new_y = y - R * math.cos(wd + orientation) + R * math.cos(orientation);
                    new_heading = orientation + wd;
                    porterImuOrientation = porterOrientation + math.degrees(wd)*(1+ (self.orientationError*(0.5-random.random())))

                with threadLock :
                    realPorterOrientation = math.degrees(new_heading) + 90
                    realPorterLocation = (new_x + realPorterWheelOffsetY*math.cos(orientation)/self.scale,new_y + realPorterWheelOffsetY*math.sin(orientation)/self.scale)
                    #realPorterLocation = (new_x,new_y)
                    wheelSpeeds = (realWheelSpeeds[0]*(1 +(self.encoderError*(0.5-random.random())))*self.scale, realWheelSpeeds[1]*(1 + (self.encoderError*(0.5-random.random())))*self.scale)
                    realPorterOrientation = constrainAngle360(realPorterOrientation, 180, -180)
                    
                #collision detection
                if self.checkPorterObstruction() :
                    if self.checkPorterCollision() :
                        with threadLock :
                            realPorterLocation = (x,y)
                            realPorterOrientation = o
                            wheelSpeeds = [0,0]
                            exitFlag = True
                        self.nextRunMode = ""
                        
                manControl = False
            else :
                with threadLock :
                    wheelSpeeds = [0,0]
                    realWheelSpeeds = [0,0]
                    


                    
                    
    def runSim(self, e) :
        global porterLocation 
        global realPorterLocation
        global lidarReady
        global lidarMap
        global lidarAngles
        global dataMap
        global realDataMap
        global wheelSpeeds
        global speedVector
        global exitFlag
        global threadLock
        global pathMap
                    
        if not exitFlag :
            self.createViews()
            if not self.simRunning :
                if self.placePorter(e) :
                    # start thread for algorithm
                    if self.runMode == "runMapSim" :
                        simThread = mappingThread(1, "mappingThread",self.simFrameTime, self.scale, self.pixelPorterSize)
                        simThread.start()

                    elif self.runMode == "runNavSim" : 
                        simThread = navigationThread(1, "navigationThread",self.simFrameTime, self.scale, self.pixelPorterSize)
                        simThread.start()
                        
            if self.porterAdded :
                self.drawPorter(self.views["realmap"]["surface"], self.views["realmap"]["rect"], realPorterLocation[0]-(self.pixelPorterSize[0]/2),realPorterLocation[1]-(self.pixelPorterSize[1]/2), realPorterOrientation, self.porterReal["surface"])
                self.drawPorter(self.views["portermap"]["surface"], self.views["portermap"]["rect"], (porterLocation[0])/self.scale-(self.pixelPorterSize[0]/2),(porterLocation[1])/self.scale-(self.pixelPorterSize[1]/2), porterOrientation, self.porter["surface"])

            if self.simRunning : 
                if threading.activeCount() == 1 :
                    print "stopping"
                    self.simRunning = False
                    exitFlag = True
                else :
                    # user input
                    self.movePorter(e)
                    self.adjustError(e)
                    
                    # real 
                    self.realMovePorter()
                    
                    # threads
                    self.checkLidar()
                    self.runSLAM()      
                    
                    # draw outcome
                    #self.drawLidarGrid()
                    self.drawDataMap()
                    self.drawRealDataMap()
                    self.drawNavMap()
                
            
            if self.simRunning : 
                self.simClock.tick(self.simFrameRate)
        else :
            # Else shutdown clear variables
            self.porterAdded = False
            self.simRunning = False
            self.runMode = self.nextRunMode
            self.firstRun = True
            while threading.activeCount() > 1 :
                time.sleep(0.1)
            with threadLock :
                exitFlag = False
                lidarReady = True
                lidarMap = set()
                dataMap  = set()
                realDataMap = set()
                pathMap  = {}
                lidarAngles = {}
                wheelSpeeds = [0,0]
                speedVector = [0,0]
                    
        
    def stop(self) :
        # Maybe deal with any errors?
        self.firstRun = True
        self.runMode=self.nextRunMode
        pass


    def main(self) :
        global exitFlag
        global threadLock
        while True:
            e = pygame.event.poll()
            #Check for quit event
            if e.type == pygame.QUIT:
                with threadLock :
                    exitFlag = True
                while threading.activeCount() > 1 :
                    print "waiting"
                    time.sleep(0.1)
                quit()
            
            # Check for menu event
            if e.type == pygame.MOUSEBUTTONDOWN:
                if e.pos[1] < self.winY*0.05 :
                    #must have clicked in menuBar
                    for name, button in self.menuButtons.iteritems() :
                        if button["button"].pressed(e.pos) :
                            # This button was clicked!
                            if self.runMode != "" and self.runMode != None: # REVISIT : How is runMode EVER set to None??
                                with threadLock :
                                    exitFlag = True
                                self.nextRunMode  = name
                            else :
                                self.runMode = name
                                self.firstRun = True
                            
            
            # Check run mode - run relevent function
            if   self.runMode == "mapBuilder" :
                self.mapBuilder(e)
            elif self.runMode == "selectmap" :
                self.selectmap()
                self.runMode = ""
            elif self.runMode == "runMapSim" :
                self.runSim(e)
            elif self.runMode == "runNavSim" :
                self.runSim(e)
            elif self.runMode == "stop" :
                self.runMode = self.stop()
            
            self.screen.blit(self.views["realmap"]["surface"], self.views["realmap"]["rect"])
            self.screen.blit(self.views["portermap"]["surface"], self.views["portermap"]["rect"])
            pygame.display.flip()  

        pygame.quit()
        
        
    #####
    # Porter emulation functions here
    #####    
    # In real porter this is achieved using wheel encoders and is already implemented
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
        global encoderVectorAbs
        global encoderVectorAbsReady
        
        encoderVectorAbsReady = True
        if not realCollision :
            if wheelSpeeds != [0,0] :                
                leftDelta  = self.simFrameTime * wheelSpeeds[0]
                rightDelta = self.simFrameTime * wheelSpeeds[1]
                totalR += (leftDelta + rightDelta)
                orientation = math.radians(porterOrientation - 90)
                orientationUnconst = math.radians(porterOrientationUnConst - 90)
                #orientation = math.radians(realPorterOrientation - 90)
                x   = porterLocation[0] - realPorterWheelOffsetY*math.cos(orientation)
                y   = porterLocation[1] - realPorterWheelOffsetY*math.sin(orientation)
                #with threadLock :
                #    dataMap.add((int(round(x)),int(round(y))))

                if (math.fabs(leftDelta - rightDelta) < 1.0e-6) : # basically going straight
                    new_x = x + leftDelta * math.cos(orientation);
                    new_y = y + rightDelta * math.sin(orientation);
                    new_heading = orientation;
                    new_headingUnConst = orientationUnconst;
                else :
                    R = self.pixelPorterSize[0] * (leftDelta + rightDelta) / (2 * (rightDelta - leftDelta))
                    wd = (rightDelta - leftDelta) / self.pixelPorterSize[0] ;
                    new_x = x + R * math.sin(wd + orientation) - R * math.sin(orientation);
                    new_y = y - R * math.cos(wd + orientation) + R * math.cos(orientation);
                    
                    totalWd += wd/ self.scale
                    new_heading = orientation + wd/ self.scale;
                    new_headingUnConst = orientationUnconst + wd/ self.scale;
                
                with threadLock :
                    encoderVectorAbs = (new_x - x, new_y -y)

                    porterLocation = (new_x + realPorterWheelOffsetY*math.cos(orientation),new_y + realPorterWheelOffsetY*math.sin(orientation))
                    porterOrientation = (math.degrees(new_heading) + 90)*0.5 + porterImuOrientation*0.5
                    porterOrientationUnConst = (math.degrees(new_headingUnConst) + 90) # + porterImuOrientation*0.5
                    porterOrientation = constrainAngle360(porterOrientation, 180, -180)



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
        # self.angleOffsetLookup       = { 0 : [0,1,0],
                                    # 1 : [1,1,45],
                                    # 2 : [1,0,90],
                                    # 3 : [1,-1,135],
                                    # 4 : [0,-1,180],
                                    # 5 : [-1,-1,225],
                                    # 6 : [-1,0,270],
                                    # 7 : [-1,1,315],
                                  # }
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
        
        # Make array for SLAM map
        mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)
        # Get SLAM map
        SLAMObject.getmap(mapbytes)
        # Convert map into numpy array
        SLAMMap = np.frombuffer(mapbytes, dtype='B')
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

                    
class simThreadBase(MultiThreadBase) :
    
    def __init__(self, threadID, name, simFrameTime, scale, pixelPorterSize):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.simFrameTime = simFrameTime
        self.scale = scale
        self.pixelPorterSize = pixelPorterSize
        self.angleOffsetLookup       = { 0 : [0,-1,0],
                                    1 : [1,-1,45],
                                    2 : [1,0,90],
                                    3 : [1,1,135],
                                    4 : [0,1,180],
                                    5 : [-1,1,225],
                                    6 : [-1,0,270],
                                    7 : [-1,-1,315],
                                  }
        # self.angleOffsetLookup       = { 0 : [0,1,0],
                                    # 1 : [1,1,45],
                                    # 2 : [1,0,90],
                                    # 3 : [1,-1,135],
                                    # 4 : [0,-1,180],
                                    # 5 : [-1,-1,225],
                                    # 6 : [-1,0,270],
                                    # 7 : [-1,1,315],
                                  # }
        
        
    def getLidar360(self) :
        global lidarReady
        global threadLock
        global lidarRun
        global lidarReady
        global exitFlag
        
        while (not lidarReady) and (not exitFlag):
            time.sleep(0.1)

        with threadLock :
            lidarRun = "a"
            lidarReady = False
                
        while (not lidarReady) and (not exitFlag):
            time.sleep(0.1)

    # REVISIT : this will need modification and mods elsewhere as this is not how it usually works
    def getMaxSpeeds(self) :
        return [20,20]
        # REVISIT : STUB

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
        global exitFlag
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
        while (not atDest) and (not atObstacle) and (not exitFlag):
            time.sleep(0.01)
            maxSpeeds = self.getMaxSpeeds()
            speed = min(maxSpeeds)
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
        global exitFlag
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
        while (not atDest) and (not atObstacle) and (not exitFlag):
            desOrientation = constrainAngle360(self.getDesOrientation(porterLocation, dest), porterOrientationUnConst + 180, porterOrientationUnConst - 180)
            f.write("\tloop start" + "\n")
            f.write("\tporterLocation: " + str(porterLocation) + "\n")
            f.write("\tdesOrientation: " + str(desOrientation) + "\n")
            maxSpeeds = self.getMaxSpeeds()
            speed = min(maxSpeeds)
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
        maxSpeed = min(self.getMaxSpeeds())
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
        # while (not exitFlag) and (not done) :
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
            
            
        while not (desOrientation-porterOrientationUnConst)*sign < 0  and not exitFlag:
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
        
        maxSpeed = min(self.getMaxSpeeds())
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
        while (not exitFlag) and (not done) :
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
        global exitFlag
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
        while (len(openSet) != 0) and (not exitFlag) :
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
                    continue		# This is not a better path.
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
        while current in cameFrom.keys() and not exitFlag:
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
        while not exitFlag :
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
                        if exitFlag :
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
    
    
class mappingThread(simThreadBase):
    
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
        global exitFlag
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
        
        # Create pathMap
        pathMapTemp = pathMapClass(realPorterSize[0])
        with threadLock :
            pathMap = pathMapTemp
            
        # start lidar
        with threadLock :
            SLAMObject = RMHC_SLAM(roboPorterLaser(), MAP_SIZE_PIXELS, MAP_SIZE_METERS, random_seed=0)
            SLAMRunning = True
            lidarRun = "a"
        
        time.sleep(2)
        
        locations = self.getScanLocations()
        for loc in locations :
            dataMap.add(loc)
        while len(locations) > 1  and not exitFlag:
            locations = self.sortLocations(locations)
            print locations
            success = self.goTo(locations[0], 20, True)
            # REVISIT : If not success - blacklist the area?
            time.sleep(2)
            locations = self.getScanLocations()
            
        # print("Finished, waiting...")
        # while not exitFlag :
            # time.sleep(0.1)
        cv2.waitKey(1)
        cv2.destroyAllWindows()
        cv2.waitKey(1)
        print "done"


    
class navigationThread(simThreadBase):
    def run(self):            # May not need all of these
        global USAvgDistances
        global wheelSpeeds
        global obstruction
        global maxSpeeds
        global porterLocation #vector holding local location [x,y] in cm
        global porterOrientation #angle from north (between -180,180) in degrees
        global porterOrientationUnConst
        global lastCommand
        global speedVector
        global dataReady
        global threadLock
        global exitFlag
        global USThreashholds
        global stoppingDistance
        global realObstruction
        global realPorterLocation
        global realPorterOrientation
        global lidarReady
        global lidarMap
        global lidarAngles
        global lidarRun
        global dataMap
        global realDataMap
        global pathMap
        global SLAMObject
        global SLAMRunning
        
       #with threadLock :
        #    dataMap = set()
        #    realDataMap = set()
        #    speedVector = [0,0]
        #    wheelSpeeds = [0,0]
        
        # Create pathMap
        pathMapTemp = pathMapClass(realPorterSize[0])
        with threadLock :
            pathMap = pathMapTemp
        
        with threadLock :
            SLAMObject = RMHC_SLAM(roboPorterLaser(), MAP_SIZE_PIXELS, MAP_SIZE_METERS)
            SLAMRunning = True
            lidarRun = "a"
        
        time.sleep(5)


            
        self.goTo((900,600), 20, False)
        
        cv2.waitKey(1)
        cv2.destroyAllWindows()
        cv2.waitKey(1)
        print "done"
        
if __name__ == "__main__" :
    i_porterSim = porterSim()
    i_porterSim.main()
