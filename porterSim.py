import sys, pygame
import time
import numpy
import pickle
import glob
import math
import random 
from bresenham import bresenham

# Porter global variables
global USAvgDistances
global wheelSpeeds
global obstruction
global maxSpeeds
global LIDARAngles
global LIDARMap
global porterLocation #vector holding local location [x,y] in cm
global porterOrientation #angle from north (between -180,180) in degrees
global lastCommand
global speedVector
global dataReady
global threadLock
global USThreashholds
global stoppingDistance
global lidarRun
global lidarAngles
global lidarMap
global lidarReady
global porterImuOrientation
wheelSpeeds        = [0,0]    
lidarReady          = True
lidarAngles         = {}
lidarMap            = set()
lidarRun            = ""
obstruction         = False
lastCommand         = ""
speedVector         = [0, 0]
dataReady           = False
threadLock          = False
USThreashholds      = {"front":30,"back":30,"side":20}
stoppingDistance    = 20
porterOrientation   = 0
porterImuOrientation = porterOrientation

# RealPorter global variables
global realObstruction
global realCollision
global realPorterLocation
global realPorterOrientation
realPorterOrientation = 0
realObstruction         = False
realCollision         = False


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
        pygame.init()
        pygame.font.init()  
        self.simSpeed           = 1 # Scale time 
        self.simFrameTime       = 0.015 
        self.simFrameRate       = 1/self.simFrameTime
        self.scale              = 2 # 1pixel = 1cm
        self.realPorterSize     = (73,70)
        self.wheelSpeedError    = 0.05
        self.orientationError   = 0.5
        self.encoderError       = 0.05
        self.lidarError         = 0.02
        self.pixelPorterSize    = (self.realPorterSize[0]/self.scale,self.realPorterSize[1]/self.scale)
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
            view["absSize"] = (int(view["relSize"][0]*self.winX),int(view["relSize"][1]*self.winY))
            view["absPos"]  = (int(view["relPos"][0]*self.winX),int(view["relPos"][1]*self.winY))

        self.tileRealW          = 20 # Real tile width(=height) in cm
        self.tileW              = self.tileRealW / self.scale # tile size in pixels, 2 pixels per square
        self.nTileX             = int(math.floor(self.views["realmap"]["absSize"][0]/self.tileW)) # number of tiles in x
        self.nTileY             = int(math.floor(self.views["realmap"]["absSize"][1]/self.tileW)) # number of tiles in y
        self.tilePorterSize     = (self.pixelPorterSize[0]/self.tileW,self.pixelPorterSize[1]/self.tileW)
        self.longestR           = math.sqrt((self.nTileX*self.tileW)**2+(self.nTileY*self.tileW)**2) # Length of a line that will always cover the whole map for lidar use
        self.runMode            = ""
        self.nextRunMode        = ""
        self.lidarRpm           = 1 # This is approximate as per the calculation of self.lidarNSamples
        self.lidarNSamples      = int(math.ceil(360*self.lidarRpm*self.simFrameTime)) # This calculates the number of lidar samples to be taken each frame - it rounds up so the realRPM will be slightly higher than given 
        self.porterAdded        = False # Has the porter been placed at the start of simRun
        self.simRunning         = False # Is the simulation running
        self.draw_on            = 0 # Is drawing enabled in mapBuilder
        self.lidarPos           = 0     # Stores lidar position so that it can continue scanning next frame
        self.black              = 0,0,0 # The colour black
        self.map                = set() # A set of tuples each (x,y) of a wall tile  
        self.fineMap            = set() # A set of tuples each (x,y) of a wall pixel hence fine
        self.rectMap            = []    # A list of rects(pygame object) each equivilent to a tile in self.map - used for collision detection
        self.stopping           = False # Is the current task stopping - allows variables to be reset when ending sim etc.
        
        print "scale: " + str(self.scale) + "cm/p, tileW: " + str(self.tileW) + "p, tileRealW: " + str(self.tileRealW) + "cm, nTileX: " + str(self.nTileX) + ", nTileY: " + str(self.nTileY) + ", maxX: " + str(self.scale*self.tileW*self.nTileX/100) + "m, maxY: " + str(self.scale*self.tileW*self.nTileY/100) + "m" 
        
        self.simClock           = pygame.time.Clock()
        self.genBlankMap()
        self.screen             = pygame.display.set_mode((self.winX, self.winY))
        self.screen.fill(self.black)
        self.createViews()
        self.createMenuButtons()
        

    def createMenuButtons(self) :
        # Create menu
        self.menuButtons = {    "mapBuilder" :  {"button" : Button(self.screen, self.font, "mapBuilder", (100,0,0), 0, 0, 100, int(0.05*self.winY)),
                                                },
                                "selectmap"  :  {"button" : Button(self.screen, self.font, "selectmap", (0,100,0), 100, 0, 100, int(0.05*self.winY)),
                                                },
                                "runMapSim"  :  {"button" : Button(self.screen, self.font, "runMapSim", (0,0,100), 200, 0, 100, int(0.05*self.winY)),
                                                },
                                "runNavSim"  :  {"button" : Button(self.screen, self.font, "runNavSim", (0,100,100), 300, 0, 100, int(0.05*self.winY)),
                                                },
                                "stop"       :  {"button" : Button(self.screen, self.font, "stop", (255,0,0), 400, 0, 100, int(0.05*self.winY)),
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
        if self.stopping == False :
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
            self.stopping = False
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
            self.menuButtons[option] = {"button" : Button(self.screen, self.font, option, (255*i/n,255-(255*i/n),255+(100*-i/n)), i*150, 0, 150, int(0.05*self.winY)) }
        
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
        porterRect = porterRect.move(realPorterLocation)
        if porterRect.collidelist(self.rectMap) != -1 :
            realCollision = True
        else :
            realCollision = False

        return realCollision
        
    def checkPorterObstruction(self) :
        global realObstruction
        # REVISIT : Unfortunately a square collision box is the only reasonable way to achieve this using pygame
        #           i.e. if porter is rotated the bounding box will be bigger than it should be
        porterRect = self.porterReal["surface"].get_rect()
        porterRect = porterRect.move(realPorterLocation)
        porterRect.inflate_ip(2*USThreashholds["front"]/self.scale, 2*USThreashholds["front"]/self.scale)
        
        if porterRect.collidelist(self.rectMap) != -1 :
            realObstruction = True
            print "Real Obstruction: " + str(realObstruction)
        else :
            realObstruction = False
    
        return realObstruction
    
    def getRange(self,x,y) :
        if x > y :
            numRange = reversed(range(int(y), int(x)+1))
        else :
            numRange = range(int(x), int(y)+1)
        return numRange
    
    def rotate(self, origin, point, angle):
        """
        Rotate a point counterclockwise by a given angle around a given origin.

        The angle should be given in radians.
        """
        ox, oy = origin
        px, py = point

        qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
        qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
        return int(round(qx)), int(round(qy))

    
    def constrainAngle(self, angle, max, min) :
        while angle >= max :
            angle -= 360
        while angle < min :
            angle += 360
        return angle
    
    
    def getLidarSample(self, angle) : 
        # Sets lidarAngles{angle} to the distance of the nearest collision at that angle relative to the porterOrientation
        # Also adds a coordinate to lidarMap if a point is detected (which it will be < 40m)
        global realPorterLocation
        global realPorterOrientation
        global lidarMap
        global lidarAngles
        # Generate a list of points on line going from centre of porter out at angle given
        linePoints = list()
        # Centre coordinates of porter
        x0 = int(round(realPorterLocation[0]) + self.pixelPorterSize[0]/2)
        y0 = int(round(realPorterLocation[1]) + self.pixelPorterSize[1]/2)
        # Absolute angle
        a    = self.constrainAngle(realPorterOrientation + angle - 90, 360,0)
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
        startPos = (porterLocation[0] + self.pixelPorterSize[0]/2, porterLocation[1] + self.pixelPorterSize[1]/2)
        r = math.sqrt((point[0]-x0)**2+(point[1]-y0)**2)
        porterAbsAngle = self.constrainAngle(porterOrientation -90 + angle, 360, 0)
        endPos = (int(round(startPos[0] + r*math.cos(math.radians(porterAbsAngle)))), int(round(startPos[1] + r*math.sin(math.radians(porterAbsAngle)))))
        
        if r <(4000/self.scale) :  # If within 40m which it definitely will be...
            lidarAngles[angle] = r
            lidarMap.add((endPos))
            # Draw lidar line and hit point on portermap
            pygame.draw.circle(self.views["portermap"]["surface"],self.black,endPos,5)
            pygame.draw.line(self.views["portermap"]["surface"], self.black, startPos, endPos)
        
            
    def checkLidar(self) :
        global lidarRun
        global lidarReady
        global lidarMap
        global lidarAngles
        if lidarRun == "a":
            # Start 360 scan
            self.lidarPos = -180
            lidarReady  = False
            lidarMap    = set()
            lidarAngles = {}
            
        elif (len(lidarRun)>1) and (lidarRun[0] == "a") :
            # Get single sample
            lidarAngles = {}
            lidarMap    = set()
            try :
                angle = int(lidarRun[1:])
                self.getLidarSample(angle)
            except TypeError:
                pass
            
        if not lidarReady : # must have more samples to get in 360 scan
            # Get 360 samples
            endPos = int(round(self.lidarPos + self.lidarNSamples))
            if endPos >= 180 :
                endPos = 180
                lidarReady = True
            for angle in range(self.lidarPos,endPos) :
                self.getLidarSample(angle)
        
            self.lidarPos = endPos
        lidarRun = ""
    
    
    def drawLidarGrid(self) :
        global lidarMap
        for point in lidarMap :
            pygame.draw.circle(self.views["portermap"]["surface"], self.black, point, 1)
    
    
    def placePorter(self, e) : # UI for placing porter when starting sim
        global realPorterLocation
        global porterLocation
        global realPorterOrientation
        global porterOrientation
        # Configure variables if first loop
        if self.firstRun :
            self.firstRun = False
            self.porter = {}
            self.porterReal = {}
            porterOrientation = 0
            porterImuOrientation = porterOrientation
            realPorterOrientation = 0
            porterLocation = (0,0)
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
                        porterLocation              = (e.pos[0]+self.views["realmap"]["absPos"][0]-(self.pixelPorterSize[0]/2), e.pos[1]- self.views["realmap"]["absPos"][1]-(self.pixelPorterSize[1]/2))
                        realPorterLocation          = (e.pos[0]+ self.views["realmap"]["absPos"][0]-(self.pixelPorterSize[0]/2),e.pos[1]- self.views["realmap"]["absPos"][1]-(self.pixelPorterSize[1]/2))
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
                    if self.porterAdded  :
                        self.simRunning = True
                
                if e.button == 3 :
                    # Rotate porter 90
                    if porterOrientation > 90 :
                        porterOrientation -= 270
                    else :
                        porterOrientation += 90
                    porterImuOrientation = porterOrientation    
                    realPorterOrientation = porterOrientation
                    

    def realMovePorter(self) :
        # May not need all of these
        global realPorterLocation
        global realPorterOrientation
        global porterImuOrientation
        global speedVector
        global wheelSpeeds
        
        if not realCollision :
            if speedVector != [0,0] :
                realWheelSpeeds = (speedVector[0]*(1 +(self.wheelSpeedError*(0.5-random.random()))), speedVector[1]*(1 + (self.wheelSpeedError*(0.5-random.random()))))
                
                leftDelta  = self.simFrameTime * realWheelSpeeds[0] / self.scale
                rightDelta = self.simFrameTime * realWheelSpeeds[1] / self.scale
                orientation = math.radians(realPorterOrientation - 90)
                x,y = realPorterLocation
                o   = realPorterOrientation

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


                realPorterLocation = (new_x,new_y)
                realPorterOrientation = math.degrees(new_heading) + 90
                
                wheelSpeeds = (realWheelSpeeds[0]*(1 +(self.encoderError*(0.5-random.random()))), realWheelSpeeds[1]*(1 + (self.encoderError*(0.5-random.random()))))

                realPorterOrientation = self.constrainAngle(realPorterOrientation, 180, -180)
                # if realPorterOrientation > 180 :
                    # realPorterOrientation -= 360
                # elif realPorterOrientation < -180 :
                    # realPorterOrientation += 360
                    
                #collision detection
                if self.checkPorterObstruction() :
                    if self.checkPorterCollision() :
                        realPorterLocation = (x,y)
                        realPorterOrientation = o
                        wheelSpeeds = [0,0]
                        self.stopping = True
                        self.nextRunMode = ""
            else :
                wheelsSpeeds = [0,0]



                    
    def runSim(self, e) :
        global porterLocation #vector holding local location [x,y] in cm
        global realPorterLocation
        global lidarReady
        global lidarMap
        global lidarAngles

        if not self.stopping :
            self.createViews()
            self.placePorter(e)
            
            if self.simRunning : 
                self.realMovePorter()
                self.checkLidar()
                self.drawLidarGrid()
                if self.runMode == "runMapSim" :
                    self.mapping()      ####### USER FUNCTION
                elif self.runMode == "runNavSim" : 
                    self.navigation()
                
            if self.porterAdded :
                self.drawPorter(self.views["realmap"]["surface"], self.views["realmap"]["rect"], realPorterLocation[0],realPorterLocation[1], realPorterOrientation, self.porterReal["surface"])
                self.drawPorter(self.views["portermap"]["surface"], self.views["portermap"]["rect"], porterLocation[0],porterLocation[1], porterOrientation, self.porter["surface"])
            
            if self.simRunning : 
                self.simClock.tick(self.simFrameRate)
        else :
            # Else shutdown clear variables
            self.porterAdded = False
            self.simRunning = False
            self.runMode = self.nextRunMode
            self.stopping = False
            self.firstRun = True
            lidarReady = True
            lidarMap = set()
            lidarAngles = {}
                    
        
    def stop(self) :
        # Maybe deal with any errors?
        self.firstRun = False
        self.runMode=self.nextRunMode
        pass


    def main(self) :
        while True:
            e = pygame.event.poll()
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
                            if self.runMode != "" and self.runMode != None: # REVISIT : How is runMode EVER set to None??
                                self.stopping = True
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
    
    def calculatePorterPosition(self) :
        global porterLocation
        global porterOrientation
        global porterImuOrientation
        global wheelSpeeds
        
        if not realCollision :
            if wheelSpeeds != [0,0] :                
                leftDelta  = self.simFrameTime * wheelSpeeds[0] / self.scale
                rightDelta = self.simFrameTime * wheelSpeeds[1] / self.scale
                orientation = math.radians(porterOrientation - 90)
                x,y = porterLocation

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
                
                porterLocation = (new_x,new_y)
                porterOrientation = (math.degrees(new_heading) + 90)*0.5 + porterImuOrientation*0.5
                porterOrientation = self.constrainAngle(porterOrientation, 180, -180)

    
    def mapping(self) :
        # May not need all of these
        global USAvgDistances
        global wheelSpeeds
        global obstruction
        global maxSpeeds
        global porterLocation #vector holding local location [x,y] in cm
        global porterOrientation #angle from north (between -180,180) in degrees
        global lastCommand
        global speedVector
        global dataReady
        global threadLock
        global USThreashholds
        global stoppingDistance
        global realObstruction
        global realPorterLocation
        global realPorterOrientation
        global lidarReady
        global lidarMap
        global lidarAngles
        global lidarRun

        # Calculates porter position based on wheel speeds and IMU - IMU is overpowered (accurate)...
        self.calculatePorterPosition()

        # Things you may want to do
        # speedVector = [50,100]
        # if lidarReady :
        #     lidarRun = "a"    

        
    def navigation(self) :
        # May not need all of these
        global USAvgDistances
        global wheelSpeeds
        global obstruction
        global maxSpeeds
        global porterLocation #vector holding local location [x,y] in cm
        global porterOrientation #angle from north (between -180,180) in degrees
        global lastCommand
        global speedVector
        global dataReady
        global threadLock
        global USThreashholds
        global stoppingDistance
        global realObstruction
        global realPorterLocation
        global realPorterOrientation
        global lidarReady
        global lidarMap
        global lidarAngles
        global lidarRun

        # Calculates porter position based on wheel speeds and IMU - IMU is overpowered (accurate)...
        self.calculatePorterPosition()

        # Things you may want to do
        # speedVector = [50,100]
        # if lidarReady :
        #     lidarRun = "a"
        
        
if __name__ == "__main__" :
    i_porterSim = porterSim()
    i_porterSim.main()