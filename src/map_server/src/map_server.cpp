#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <sstream>
#include <string>
#include <stdint.h>
using namespace cv;
using namespace std;
using namespace geometry_msgs;
using namespace nav_msgs;

//string file_name = ;

int main(int argc, char **argv)
{	
	string file_name;
 	ros::init(argc, argv, "map_node");
	ros::NodeHandle m("~");
	m.getParam("file_name", file_name);
	ros::Publisher map_pub = m.advertise<nav_msgs::OccupancyGrid>("map", 10);
	ros::Rate loop_rate(0.1);
	Mat image = imread("src/map_server/src/" + file_name, IMREAD_GRAYSCALE); 
	if(!image.data ) {
      		cout <<  "Image not found or unable to open" << endl ;
      		return -1;
    	}	
	Mat image_bw;
	threshold(image, image_bw, 100, 255.0, THRESH_BINARY); //Use threshold function to convert to black and white, 100 being the threshold

	float width = image_bw.cols; 
	float resolution = width/500;//Assumption: image is a square
	Pose origin;
	origin.position.x = 0;
	origin.position.y = 0;
	origin.position.z = 0;//Assumption: origin is at (0,0), everything is in positive x y plane
	origin.orientation.w = 1;
	//image_bw.create(500,500, CV_8UC3);//image is now a single channel array
	OccupancyGrid msg;
	msg.data.assign(image_bw.data, image_bw.data + image_bw.total());	
	msg.header.seq = 1;
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = "World";
	msg.info.map_load_time = ros::Time::now();
	msg.info.resolution = resolution;
	msg.info.width = image_bw.cols;
	msg.info.height = image_bw.rows;
	msg.info.origin = origin;
	while(ros::ok()){
		map_pub.publish(msg);
//If you were to take a look at the published message in rostopic, you would see a bunch of -1s among the 0s. This is not shocking; the white points are stored as 255, which in unsigned 8bit binary would be 11111111. However, ros reads it as 2's complement, and 11111111 is -1. However, as long as we humans know the true meaning, it shouldnt make a difference. "Shouldnt"
		loop_rate.sleep();
		ros::spinOnce();
	}
	return 0;
}
