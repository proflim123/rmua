# rmua
Submission for RMUA technical assessment
# Instructions
Download the rmua.zip folder only, it has everything inside\
Extract the files\
Open up a terminal, typee the following commands:\
*cd rmua*\
*catkin_make*\
*source devel/setup.bash*\
## Optional
*cd src/map_server/src*\
You might need to run (*g++ draw.cpp -o drawer -std=c++11 `pkg-config --cflags --libs opencv`*) beforehand\
*./drawer*\
This program allows a user to draw a map of their own. Click the mouse at a point and lift it up at another point to draw a line between these two points. \
Press Esc to save as “userdrawn.jpg”\
## Optional part over


**Main part**
roscore
cd rmua //The map_server node is meant to run from rmua folder	

**Map_server node**
rosrun map_server map_server _file_name:="file_name", where file_name can be map2.pgm or userdrawn.jpg or some valid file in the src folder
If you choose not the declare the file name in the rosrun command it will default to the previous given file_name

**Planner node**
rosrun planner planner 
