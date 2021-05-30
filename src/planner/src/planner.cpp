#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "ctime"
#include <iostream>
#include <cstdlib>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <tgmath.h> 

//using namespace geometry_msgs;
using namespace nav_msgs;
using namespace std;

struct Waypoint{
	int x, y, index, parent;//In the array knownpoints, this fetches the parent point
};

Waypoint startpoint;
Waypoint endpoint;
bool start_ready;
bool end_ready;
bool map_ready;
//vector<vector<int>> occupancydata;
int occupancydata [500][500];
int width,height;
Waypoint origin;
Path path;
vector<Waypoint> knownpoints;
vector<Waypoint> route;
vector<cv::Point> CVpoints;
cv::Mat grid;

bool isEqual(Waypoint a, Waypoint b){
	return (a.x == b.x and a.y == b.y); 
}

float distance(Waypoint a, Waypoint b){
	return sqrt(pow((b.x-a.x), 2) + pow((b.y-a.y), 2));
}

bool occupied(int x, int y){
	bool ans = occupancydata[y][x];
	if(ans == -1){return true;}
	else {return false;}
	//cv::Scalar intensity = grid.at<uchar>(cv::Point(y, x));
	//return(int(intensity.val[0]) != 0 );
}

bool occupied(Waypoint p){
	bool ans = occupancydata[p.y][p.x];
	if(ans == -1){return true;}
	else {return false;}
	//return(int(grid.at<uchar>(cv::Point(p.x,p.y))) != 0);
}

bool pathClear(Waypoint a, Waypoint b){ //Imagine a line joining the points a and b. This algorithm finds the approximate coordinate of each pixel on the line and checks if it is occupied. If so much as one pixel is occupied, the entire line is blocked since dy/dx = gradient, an increment of xcoord by 1 pixel increases ycoord by (gradient) pixels
	Waypoint start;
	Waypoint end;

	if(a.x < b.x){		  
		start = a;  
		end = b;
	}
	else if(a.x > b.x){
		start = b;
		end = a;
	}
	else{//a.x = b.x, cannot divide by zero
		if(a.y < b.y){
			for(int i = a.y; i < b.y; i++){
				if(occupied(a.x, i)){return false;}
			}
			return true;
		}
		else if(a.y > b.y){
			for(int i = b.y; i < a.y; i++){
				if(occupied(a.x, i)){return false;}
			}
			return true;
		}
		else{return false;}//Although they are the same point and the path is obviously clear, I don't want the program to waste time
	}
	int xcoord = start.x;
	int ycoord = start.y;	
	float gradient = (end.y - start.y)/(end.x - start.x);
	for(int i = 1; i < (end.x - start.x); i++){
		xcoord++;
		float temp = start.y + i*gradient;
		ycoord = round(temp);
		if(occupied(xcoord,ycoord)){return false;}
	}
	return true;//If the program made it this far, all the points are not occupied, so the path is clear
}

void startpointCallback(const boost::shared_ptr<geometry_msgs::PointStamped const>& msg){
	startpoint.x = msg->point.x;
	startpoint.y = msg->point.y;
	startpoint.index = 0;
	start_ready = true;
}

void endpointCallback(const boost::shared_ptr<geometry_msgs::PointStamped const>& msg){
	endpoint.x = msg->point.x;
	endpoint.y = msg->point.y;
	end_ready = true;
}

void mapCallback(const boost::shared_ptr<OccupancyGrid const>& msg){//Given the array of points, write into a matrix
	width = msg->info.width;
	height = msg->info.height;
	for(int row = 0; row < height; row++){//Row major order
		for(int column = 0; column < width; column++){
			occupancydata [row][column] = msg->data[width*row + column];
		}
		
	}
	origin.x = msg->info.origin.position.x;
	origin.y = msg->info.origin.position.y;
	map_ready = true;
}

void calculatepath(){//Assumes startpoint, endpoint and occupancy_data are all there
	knownpoints.clear();
	cv::Mat grid(500, 500, CV_8UC3, occupancydata);//comment this line if you want to load the image files instead
	/*cv::Mat image = cv::imread("src/map_server/src/map1.png", cv::IMREAD_UNCHANGED); 
	if(!image.data ) {
      		cout <<  "Image not found or unable to open" << endl ;
    	}
	threshold(image, grid, 100, 255.0, cv::THRESH_BINARY);*/
  	cv::namedWindow( "Map", cv::WINDOW_AUTOSIZE );
	cv::imshow("Map", grid);
	cv::Point s = cv::Point(startpoint.x, startpoint.y);
	cv::drawMarker(grid, s, cv::Scalar(0, 255, 0), cv::MARKER_TRIANGLE_DOWN, 10);//green
	cv::Point e = cv::Point(endpoint.x, endpoint.y);
	cv::drawMarker(grid, e, cv::Scalar(255, 0, 0), cv::MARKER_TRIANGLE_UP, 10);//blue
	cv::imshow("Map", grid); 
	cv::waitKey(00); //press any key to continue;

	if(pathClear(startpoint, endpoint)){ //Robot can travel from start to goal without any obstacles in the way
		route.push_back(startpoint);
		route.push_back(endpoint);	
  		cv::line(grid, s, e, cv::Scalar( 0, 0, 255 ), 2);//red line
		cv::imshow("Map", grid); 
		cv::waitKey(00);//press any key to continue;
	}
	else{	
		knownpoints.push_back(startpoint);
  		while(true){	
			Waypoint newpoint;
			newpoint.x = rand() % 500 + 1; 
			newpoint.y = rand() % 500 + 1;
			if(occupied(newpoint)){continue;} //Generate a different point right away
			Waypoint nearestpoint;	
			float nearestdist = 10000; //Guaranteed larger than any possible distance in the 500 by 500 square
			bool pointfound = false;
			for(auto& pt : knownpoints){ 
				if(pathClear(pt, newpoint)){
					if(distance(pt, newpoint)< nearestdist){
						nearestdist = distance(pt, newpoint); //Find the nearest point to the newly generated point
						nearestpoint = pt;
						if(!pointfound){pointfound = true;}
					}
				}
			}
			if(!pointfound){continue;} //The generated point was blocked off from every other known point. Try again
			cv::drawMarker(grid, cv::Point(newpoint.x, newpoint.y), cv::Scalar(0, 255, 0), cv::MARKER_TILTED_CROSS, 10);
			cv::line(grid, cv::Point(nearestpoint.x, nearestpoint.y), cv::Point(newpoint.x, newpoint.y), cv::Scalar( 0, 0, 255 ), 2);
			cv::imshow("Map", grid); 
			cv::waitKey(100);
			newpoint.parent = nearestpoint.index; //These assignments are for tracing purposes
			newpoint.index = knownpoints.size();
			knownpoints.push_back(newpoint);
			if(pathClear(endpoint, newpoint)){  
//If the point makes it this far, that means it is a valid point that is linked to both the goal, as well as the list of knownpoints, which is ultimately linked to start, meaning that a path from start to endexists.
				cv::line(grid, cv::Point(newpoint.x, newpoint.y), cv::Point(endpoint.x, endpoint.y), cv::Scalar( 0, 0, 255 ), 2);
				cv::imshow("Map", grid); 
				cv::waitKey(00);
				route.clear();
				route.push_back(endpoint);
				int currentindex;
				Waypoint currentpoint;
				currentpoint = newpoint;
				currentindex = knownpoints.size();
				while(currentindex != 0){//Iterate through the linked list, only start has index zero
					route.push_back(knownpoints[currentpoint.index]);
					currentpoint = knownpoints[currentpoint.parent];
					currentindex = currentpoint.index;
				}
				route.push_back(startpoint);
				reverse(route.begin(), route.end());
				break;//The path is complete
			}//If the path is not clear, return to the start of the loop and try again
		}
	}
//We have a list of waypoints, but we want a path
			
	vector<geometry_msgs::PoseStamped> poses;	
	geometry_msgs::PoseStamped posestamped;
	posestamped.pose.orientation.w = 1;
	posestamped.header.frame_id = "World";
	int count = 1;
	for(auto& pt : route){
		posestamped.header.seq = count;
		posestamped.header.stamp = ros::Time::now();
		posestamped.pose.position.x = float(pt.x);
		posestamped.pose.position.y = float(pt.y);
		posestamped.pose.position.z = 0.0;
		poses.push_back(posestamped);
		count++;
	}
	path.poses = poses;
}


int main(int argc, char **argv){
	srand((unsigned) time(0));//For generating random points
        ros::init(argc, argv, "planner_node");
        ros::NodeHandle p;
	ros::Subscriber start_sub = p.subscribe("start_point",1, startpointCallback);
	ros::Subscriber end_sub = p.subscribe("end_point",1, endpointCallback);
	ros::Subscriber map_sub = p.subscribe("map",1, mapCallback);
        ros::Publisher path_pub = p.advertise<Path>("path", 1);
	int count = 1;
	path.header.frame_id = "world";
        while (ros::ok()){
                ros::spinOnce();
		if(start_ready and end_ready and map_ready){
			calculatepath();
			path.header.seq = count;
			path.header.stamp = ros::Time::now();
               		path_pub.publish(path);
			path.poses.clear();
			start_ready = false;
			end_ready = false;
			map_ready = false;
			count++;
		}
        }
        return 0;
}

