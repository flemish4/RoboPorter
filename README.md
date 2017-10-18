Git for all RobotPorter 17/18 files

Sim :
  - Desc      : A simulator tool writen in python using pygame. Designed to allow testing of navigation and mapping algorithms.
  - Guide  :
    - Ensure python 2.7 is installed
    - Ensure libraries are present using 'python -m pip <module_name> in cmd: pygame, numpy, pickle, bresenham
    - Double click on porterSim.py or execute from commandline
    - Maps must be stored in the same directory as porterSim.py
  - Features  :
    - mapBuilder :
      - Desc  : Allows creation of grid based maps
      - Guide : 
        - Left click the mapBuilder button in the menu bar
        - Left click and drag (in left view) to add wall (black) squares
        - Left click and drag on wall segments to remove them
        - Middle click to save the current map - this will be saved with the current time and date - it will also exit mapBuilder
    - selectMap :
       - Desc : Allows selection of current (no change), blank or saved maps
       - Guide :
         - Left click the selectMap button in the menu bar
         - Left click the desired map to load
    - runMapSim/runNavSim :
      - Desc : Allows simulation of any algorithm implemented
      - Guide :
        - Left click runMapSim/runNavSim
        - Left click (in left view) to place the roboPorter - it will appear in both views ([^real]porter knows starting location)
        - Right click to rotate the porter 90 degrees
        - Middle click to start the simulation
        - To simulate mapping or navigation algorithms, add code to the navigation() or mapping() functions and use global variables
        - DO NOT USE realPorter variables as these will not be present in real life
      - Features :
        - Draws view of real porter and [^real]porter to show accuracy of positioning
        - Simulates lidar and shows scan lines - to use, set global variable lidarRun to a or a100 to do a 360 scan or scan one    location.
          Data will appear in lidarAngles and lidarMap when lidarReady is True again
        - Movement is calculated using the speedVector variables, some error is introduced. wheelSpeeds is set, porterImuOrientation is           set, this is the realPorterOrientation with some error added - use this bearing in mind we do not know how accurate the imu is
        - calculatePorterPosition() is a good start for converting wheelSpeeds and porterImuOrientation into porterLocation
      - Settings :
        - simSpeed        : Used to vary the simulation speed (untested)
        - simFrameTime    : Length of frame, may be useful for slower machines
        - scale           : The number of real world cm per pixel
        - realPorterSize  : [x,y] the size of the porter in cm
        - (wheel|orientation|encoder)Error : percentage error included when calculating values
        - font            : Font used in menu
        - winX, winY      : Size of the program window in pixels
        - views           : Relative size, position and colour of and 'views' in the program
        - tileRealW       : Real world width/height of one tile - i.e. the smallest unit you can draw on the map
        - lidarRpm        : RPM of the lidar

Details and use will be added here as functionality is added...

