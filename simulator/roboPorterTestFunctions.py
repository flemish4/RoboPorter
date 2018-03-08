
from breezyslam.vehicles import WheeledVehicle
from breezyslam.sensors import Laser

import math


# Method to load all from file ------------------------------------------------
# Each line in the file has the format:
#
#  TIMESTAMP  ... Q1  Q1 ... Distances
#  (usec)                    (mm)
#  0          ... 2   3  ... 24 ... 
#  
#where Q1, Q2 are odometry values

def load_data(datadir, dataset):
    
    filename = '%s/%s.dat' % (datadir, dataset)
    print('Loading data from %s...' % filename)
    
    fd = open(filename, 'rt')
    
    scans = []
    velocities = []
    
    while True:  
        
        s = fd.readline()
        
        if len(s) == 0:
            break       
            
        toks = s.split()[3:-1] # ignore ''
        velocityT = s.split()[3:-1] # ignore ''
        
        lidar = [int(tok) for tok in toks]
        velocity = [float(tok) for tok in velocityT]

        scans.append(lidar)
        velocities.append(velocity)
        
    fd.close()
        
    return  scans, velocities

class roboPorterLaser(Laser):
    
    def __init__(self):
                        #scan_size, scan_rate_hz, detection_angle_degrees, distance_no_detection_mm, detection_margin=0, offset_mm=0
        Laser.__init__(self, 360, 2, 360, 10000, 0, 0)
        
# Class for MinesRover custom robot ------------------------------------------

class RoboPorter(WheeledVehicle):
    
    def __init__(self):
        
        WheeledVehicle.__init__(self, 100, 500)
        
        self.ticks_per_cycle = 2000
                        
    def __str__(self):
        
        return '<%s ticks_per_cycle=%d>' % (WheeledVehicle.__str__(self), self.ticks_per_cycle)
        
    def computeVelocities(self, odometry):
        
        return WheeledVehicle.computeVelocities(self, odometry[0], odometry[1], odometry[2])

    def extractOdometry(self, timestamp, leftWheel, rightWheel):
                
        # Convert microseconds to seconds, ticks to angles        
        return timestamp / 1e6, \
               self._ticks_to_degrees(leftWheel), \
               self._ticks_to_degrees(rightWheel)
               
    def odometryStr(self, odometry):
        
        return '<timestamp=%d usec leftWheelTicks=%d rightWheelTicks=%d>' % \
               (odometry[0], odometry[1], odometry[2])
               
    def _ticks_to_degrees(self, ticks):
        
        return ticks * (180. / self.ticks_per_cycle)
        

        
        
        
