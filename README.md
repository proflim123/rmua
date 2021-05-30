# rmua
Submission for RMUA technical assessment
# Instructions
Download the rmua.zip folder, it has everything inside\
Extract the files\
Open up a terminal, type the following commands:\
*cd rmua*\
*catkin_make*\
*source devel/setup.bash*
## Optional part
*cd src/map_server/src*\
*./drawer*\
This program allows a user to draw a map of their own by generating a blank canvas. Click the mouse at any point and lift it up at another point to draw a white line between these two points. \
Press Esc to save as “userdrawn.jpg”

## Main part
*roscore*\
In a separate terminal, \
*cd rmua*\
The map_server node is meant to run from rmua folder

**Map_server node**\
*rosrun map_server map_server _file_name:="file_name"*\
where the string file_name can be map2.pgm or userdrawn.jpg or some valid file in the src folder\
The image chosen will be displayed with OpenCV for verification. Press any key to proceed\
Known bugs: map_server may stop responding when a key is pressed to close the OpenCV window. I suspect it is a problem with the OpenCV installation on my computer\
Message is published sometimes, sometimes it isn't. *Use rostopic echo map* or *rostopic echo map_server/map* to see\
The message, when echoed has -1s and 0s instead of 255s and 0s. This is because 255 is stored in the OpenCV Mat as 11111111 in unsigned binary. However, the integer array in the message reads it as signed, and 11111111 in 2's complement is -1. As long as the empty dots can be distinguished from the occupied dots it should not be a problem as long as the subscriber knows the difference.

**Planner node**\
*rosrun planner planner*\
Known bugs: regretfully, I was not able to get the subscriber to correctly receive and interpret the occupany grid message, so I cheated and opened up the image file to work on it directly instead. In the cpp file I have commented out the blocks of code that do this. Simply comment out line 135 and uncomment lines 136 and 140 to load the maps in the map_server folder as Mats. However, that is not enough; the checks for start_ready, end_ready and map_ready need to be disabled, and the start and endpoints need to be hardcoded manually\
Moreover, the occupancy detection does not work properly; routes end up passing right through walls, as shown in RRT_Attempt.png. However, the main idea is there\
Anytime the system pauses at a OpenCV window, press any key to continue
