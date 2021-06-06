# rmua
Submission for RMUA technical assessment
# Instructions
Download the rmua.zip folder, it has everything inside

Extract the files into a folder called rmua

Open up a terminal, type the following commands:

*cd rmua*

*catkin build*

*source devel/setup.bash*
## Optional part
(You might need to run ```++ draw.cpp -o drawer -std=c++11 `pkg-config --cflags --libs opencv` ``` beforehand, but shouldn't be necessary)

*./drawer*

This program allows a user to draw a map of their own. Click the mouse at a point and lift it up at another point to draw a line between these two points. The line will be invisible until you lift up the mouse 

Press Esc to save as “userdrawn.jpg” inside rmua/src/map_server/src 

If you messed up just Esc and then run ./drawer again to overwrite the same file

## Main part
*roscore*

In a separate terminal, 

*cd rmua*

The map_server node is meant to run from rmua folder

**Map_server node**

*rosrun map_server map_server _file_name:="file_name"*, where file_name can be map2.pgm or userdrawn.jpg or some valid file in the src folder

eg rosrun map_server map_server _file_name:="map2.pgm"

If you choose not to declare the file name in the rosrun command it will default to the previous given file_name. If you don't declare file_name on the very first time you run it, the node won't load and publish any image

The message, when echoed has -1s and 0s instead of 255s and 0s. This is because 255 is stored in the OpenCV Mat as 11111111 in unsigned binary. However, the integer array in the message reads it as signed, and 11111111 in 2's complement is -1. However, as long as the empty dots can be distinguished from the occupied dots, and the subscriber knows the difference, it should not be a problem.

**Planner node**

*rosrun planner planner _maxbranchlength:=50*

where 50 can be replaced by the maximum length the RRT branches should be. As with the file_name, please declare it

It will notify in terminal when inputs (start, end and map) are received, and will proceed to run only when all 3 are received

The red tree comprises all nodes in the tree, whether or not they are part of the route

The blue tree is a path from start to end

The green tree is an optimized version of the blue tree, skipping any unnecessary nodes

After publishing the path message the OpenCV window will wait for you to press any key before closing

Click the window before pressing the key


**Assumptions made**

The planner node is hard coded to take a map of dimensions 500 by 500. Any attempt to upload a map of some other dimension WILL crash the program

Map must be sent first before start and endpoints, which are to be sent to the topics start_point and end_point, both of which are to be PointStamped messages. I used rqt to do this rather than write new nodes, as shown in the screenshot RQT

The origin is assumed to be 0,0 which is the top left of the map. Any coordinates must have their x and y coordinate between 0 and 499 inclusive, and hopefully not in an occupied square. I personally recommend (and tested using these points in the screenshots):

start = (50,50) and end = (450,450) for maps 1 and 2 and userdrawn
Map1Tree.png

start = (10,10) and end = (490,490) for map 3

start = (10,10) and end = (250,250) for map 4

The robot is assumed to be only one pixel wide. In general that is not the case, but for simplicity reasons I have assumed so
