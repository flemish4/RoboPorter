LIDAR code to run on an arduino uno.

Automatically homes to a fixed location.

Currently only accepts one serial command :

"2<0-3>"

e.g. "22"

2 refers to the state/function that you want, this allows other functions to be added.

<0-3> refers to the resolution

0 - 385 samples  - 0.7s  - slightly slower per sample than the others

1 - 770 samples  - 1.17s - good for navigation?

2 - 1540 samples - 2.33s -

3 - 3080 samples - 4.65s -

Returns serial binary data using Serial.write. Not human readable but faster than Serial.print

Format:

step,time,dist


step - the number of steps - measured clockwise and is out of 3081.48 due to the gear ratio and microstepping
time - the time taken to get the sample in microseconds - useful for compensation due to motion
dist - the distance measured in cm
