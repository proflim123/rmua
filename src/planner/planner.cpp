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

using namespace nav_msgs;
using namespace std;

struct Waypoint{
	double x, y;//I used doubles because ints cause gradient to be zero
	int index, parent;//In the array knownpoints, this fetches the parent point
};

Waypoint startpoint;
Waypoint endpoint;
bool start_ready;
bool end_ready;
bool map_ready;
vector<signed char> occupancydata;
int width,height;
Waypoint origin;
Path path;
float resolution;
vector<Waypoint> knownpoints;
vector<Waypoint> route, finalroute;
cv::Mat grid(500, 500, CV_8UC3);
double maxbranchlength;
bool optimize;

bool isEqual(Waypoint a, Waypoint b){
	return (a.x == b.x and a.y == b.y); 
}

float distance(Waypoint a, Waypoint b){
	return sqrt(pow((b.x-a.x), 2) + pow((b.y-a.y), 2));
}

bool occupied(int x, int y){
	return(occupancydata[y*width + x] == -1);
}

bool occupied(Waypoint p){
	return(occupancydata[p.y*width + p.x] == -1);
}

bool pathClear(Waypoint a, Waypoint b){ //Imagine a line joining the points a and b. This algorithm finds the approximate coordinate of each pixel on the line and checks if it is occupied. If so much as one pixel is occupied, the entire line is blocked since dy/dx = gradient, an increment of xcoord by 1 pixel increases ycoord by (gradient) pixels. Assumption: the robot is one pixel wide. Obviously this is not realistic in most cases
	Waypoint start, end;
	if (abs(a.x - b.x) > abs(a.y - b.y)){//dx > dy
		if(a.x > b.x){
			start = b;
			end = a;
		}
		else{
			start = a;
			end = b;
		}
		int xcoord = start.x; 
		int ycoord = start.y; 
		double gradient = (end.y - start.y)/(end.x - start.x); 
		double temp = start.y;
		double temp2;
		for(int i = 1; i < (end.x - start.x); i++){
			xcoord++;
			temp += gradient;
			temp2 = temp;
			ycoord = temp2;
			if(occupied(xcoord,ycoord)){
				return false;
			}
		}
		return true;
	}
	else {//dx <= dy
		if(a.x == b.x and a.y == b.y){return false;}//Same point, don't waste time. And nearestdist should not be zero
		if(a.y > b.y){
			start = b;
			end = a;
		}
		else{
			start = a;
			end = b;
		}
		int xcoord = start.x; 
		int ycoord = start.y; 
		double gradient = (end.x - start.x)/(end.y - start.y); 
		double temp = start.x;
		double temp2;
		for(int i = 1; i < (end.y - start.y); i++){
			ycoord++;
			temp += gradient;
			temp2 = temp;
			xcoord = temp2;
			if(occupied(xcoord,ycoord)){		
				return false;
			}
		}
		return true;
	}
}

void startpointCallback(const boost::shared_ptr<geometry_msgs::PointStamped const>& msg){
	startpoint.x = (msg->point.x - origin.x)/resolution;
	startpoint.y = msg->point.y;
	startpoint.index = 0;//Unique indices for start and end point, everyone else has an index > 0 
	start_ready = true;
	cout<<"Start point received: " << startpoint.x << " , " << startpoint.y << endl;
}

void endpointCallback(const boost::shared_ptr<geometry_msgs::PointStamped const>& msg){
	endpoint.x = msg->point.x;
	endpoint.y = msg->point.y;
	endpoint.index = -1;
	end_ready = true;
	cout<<"End point received: " << endpoint.x << " , " << endpoint.y << endl;
}

void mapCallback(const boost::shared_ptr<OccupancyGrid const>& msg){
	width = msg->info.width;
	height = msg->info.height;
	occupancydata = msg->data;
	origin.x = msg->info.origin.position.x;
	origin.y = msg->info.origin.position.y;
	resolution = msg->info.resolution;
	map_ready = true;
	cout<<"Map received"<<endl;
}

void calculatepath(){//Assumes startpoint, endpoint and occupancy_data are all there
	knownpoints.clear();
	std::vector<signed char>::const_iterator mapDataIter = occupancydata.begin();
	for(unsigned int j = 0; j < height; ++j){
  		for(unsigned int i = 0; i < width; ++i){
      			if (*mapDataIter == -1){
				grid.at<uchar>(j,3*i) = (uchar)255;
				grid.at<uchar>(j,3*i+1) = (uchar)255;
				grid.at<uchar>(j,3*i+2) = (uchar)255;
      			}
			else{
        			grid.at<uchar>(j,3*i) = (uchar)0;
				grid.at<uchar>(j,3*i+1) = (uchar)0;
				grid.at<uchar>(j,3*i+2) = (uchar)0;
     			}
     			++mapDataIter;
  		}
  	}
	cv::namedWindow( "Map", cv::WINDOW_NORMAL );
	cv::Point s = cv::Point(startpoint.x, startpoint.y);
	cv::drawMarker(grid, s, cv::Scalar(255, 255, 255), cv::MARKER_TRIANGLE_DOWN, 10);
	cv::Point e = cv::Point(endpoint.x, endpoint.y);
	cv::drawMarker(grid, e, cv::Scalar(255, 255, 255), cv::MARKER_TRIANGLE_UP, 10);
	cv::imshow("Map", grid); 
	cv::resizeWindow("Map", 650, 1000);
	cv::waitKey(500); 

	if(pathClear(startpoint, endpoint)){ //Robot can travel from start to goal without any obstacles in the way
		route.push_back(endpoint);
		route.push_back(startpoint);	
  		cv::line(grid, s, e, cv::Scalar( 0, 0, 255 ), 2);
	}
	else{	
		knownpoints.push_back(startpoint);
  		while(true){	
			Waypoint newpoint;
			newpoint.x = rand() % 500 + 1; 
			newpoint.y = rand() % 500 + 1;
			if(occupied(newpoint)){continue;} //Generate a different point right away
			Waypoint nearestpoint;	
			float nearestdist = 1000; //Guaranteed larger than any possible distance in the 500 by 500 square
			bool pointfound = false;
			for(auto& pt : knownpoints){ 
				if(distance(pt, newpoint) < nearestdist){//Potential candidate. However, if this distance > maxbranchlength, truncate it first. then check if obstructed
					if(distance(pt, newpoint) > maxbranchlength){
						Waypoint Test;
						double dx, dy;
						dx = (newpoint.x - pt.x)*(maxbranchlength/distance(pt, newpoint));
						dy = (newpoint.y - pt.y)*(maxbranchlength/distance(pt, newpoint));
						Test.x = pt.x + dx;
						Test.y = pt.y + dy;
						if(pathClear(pt, Test)){
							nearestdist = distance(pt, newpoint); //Find the nearest point to the newly generated point
							nearestpoint = pt;
							if(!pointfound){pointfound = true;}		
						}
					}
					else{
						if(pathClear(pt, newpoint)){
							nearestdist = distance(pt, newpoint); 
							nearestpoint = pt;
							if(!pointfound){pointfound = true;}
						}
					}
				}//If distance from newpoint to pt in tree exceeds nearest dist, we dont consider it at all
			}
			if(!pointfound){continue;} //The generated point was blocked off from every other known point. Try again
			if(nearestdist > maxbranchlength){//Too long a branch, truncate
				double dx, dy;
				dx = (newpoint.x - nearestpoint.x)*(maxbranchlength/nearestdist);
				dy = (newpoint.y - nearestpoint.y)*(maxbranchlength/nearestdist);
				newpoint.x = nearestpoint.x + dx;
				newpoint.y = nearestpoint.y + dy;
			}
			cv::drawMarker(grid, cv::Point(newpoint.x, newpoint.y), cv::Scalar(0, 255, 0), cv::MARKER_TILTED_CROSS, 10);
			cv::line(grid, cv::Point(nearestpoint.x, nearestpoint.y), cv::Point(newpoint.x, newpoint.y), cv::Scalar( 0, 0, 255 ), 2);
			cv::imshow("Map", grid); 
			cv::waitKey(500);
			newpoint.parent = nearestpoint.index; //These assignments are for tracing purposes
			newpoint.index = knownpoints.size();
			knownpoints.push_back(newpoint);
			if(pathClear(endpoint, newpoint)){  
				if(distance(endpoint, newpoint) < maxbranchlength){
			cv::line(grid, cv::Point(newpoint.x, newpoint.y), cv::Point(endpoint.x, endpoint.y), cv::Scalar( 0, 0, 255 ), 2);
					cv::imshow("Map", grid); 
					cv::waitKey(500);
					route.push_back(endpoint);
					int currentindex;
					Waypoint currentpoint, prevpoint;
					currentpoint = newpoint;
					currentindex = knownpoints.size();
		cv::line(grid, cv::Point(endpoint.x, endpoint.y), cv::Point(currentpoint.x, currentpoint.y), cv::Scalar( 255, 0, 0 ), 2);
					while(currentindex != 0){//Iterate through the linked list, only start has index zero
						route.push_back(knownpoints[currentpoint.index]);
						prevpoint = knownpoints[currentpoint.parent];
		cv::line(grid, cv::Point(currentpoint.x, currentpoint.y), cv::Point(prevpoint.x, prevpoint.y), cv::Scalar( 255, 0, 0 ), 2);
						currentpoint = prevpoint;
						currentindex = currentpoint.index;
					}
					route.push_back(startpoint);//This route is in reverse, for a reason you'll see later
					break;//The path is complete
				}
				else{
					Waypoint trunc;//This point is between newpoint and endpoint
					double dx, dy;
					dx = (endpoint.x - newpoint.x)*(maxbranchlength/distance(endpoint, newpoint));
					dy = (endpoint.y - newpoint.y)*(maxbranchlength/distance(endpoint, newpoint));
					trunc.x = newpoint.x + dx;
					trunc.y = newpoint.y + dy;
			cv::drawMarker(grid, cv::Point(trunc.x, trunc.y), cv::Scalar(0, 255, 0), cv::MARKER_TILTED_CROSS, 10);
			cv::line(grid, cv::Point(trunc.x, trunc.y), cv::Point(newpoint.x, newpoint.y), cv::Scalar( 0, 0, 255 ), 2);
					trunc.parent = newpoint.index; //These assignments are for tracing purposes
					trunc.index = knownpoints.size();
					knownpoints.push_back(trunc);
					cv::imshow("Map", grid); 
					cv::waitKey(500);
				}
			}
		}
	}
	cv::imshow("Map", grid);
	cv::waitKey(500);
	cout<<"Points in route, starting from endpoint: "<< endl;
	for(int i = 0; i < route.size(); i++){cout << i+1 << ": " << route[i].x<<","<<route[i].y<<endl;}
//At this point, we have a list of waypoints, but we want a path. Sometimes, unnecessary loops and meanders exist in the path, and they need to be purged. The solution is to check, for each point starting from the end, if the point before it is absolutely necessary. I.e. what is the point with the smallest index that point n can connect directly to? If point 7 can connect directly to point 2 with no issue points 3 to 6 are "pointless". However this optimized tree has the possibility of generating very long paths since certain points are skipped, possibly exceeding the max branch length of 20 established earlier.
	Waypoint currentpoint, prevpoint;
	if(route.size() > 3 and optimize){
		cout<<"Optimizing path"<<endl;
		bool loop = true;
		int i = route.size()-1;
		while(loop){
			currentpoint = route[i];
			for(int j = 0; j < i; j++){
				prevpoint = route[j];
				if(pathClear(currentpoint, prevpoint)){
	cv::line(grid, cv::Point(currentpoint.x, currentpoint.y), cv::Point(prevpoint.x, prevpoint.y), cv::Scalar( 0, 255, 0 ), 2);
	cv::imshow("Map", grid);
					finalroute.push_back(currentpoint);
					i = j;
					if(i == 0){
						loop = false;
						break;
					}
				}
			}
		}
		finalroute.push_back(endpoint);
		cout<<"Points in optimized route"<< endl;
		for(auto& pt : finalroute){cout<<pt.x<<","<<pt.y<<endl;}
	}
	else{
		finalroute = route;
		reverse(finalroute.begin(), finalroute.end());
	}
	
	cv::imshow("Map", grid); 
	cv::waitKey(500); 
	vector<geometry_msgs::PoseStamped> poses;	
	geometry_msgs::PoseStamped posestamped;
	posestamped.pose.orientation.w = 1;
	posestamped.header.frame_id = "World";
	int count = 1;
	for(auto& pt : finalroute){
		posestamped.header.seq = count;
		posestamped.header.stamp = ros::Time::now();
		posestamped.pose.position.x = float(pt.x);
		posestamped.pose.position.y = float(pt.y);
		posestamped.pose.position.z = 0.0;
		poses.push_back(posestamped);
		count++;
	}
	path.poses = poses;
	cout<<"Path ready"<<endl;
	route.clear();
	finalroute.clear();
}


int main(int argc, char **argv){
	srand((unsigned) time(0));//For generating random points
        ros::init(argc, argv, "planner_node");
        ros::NodeHandle p("~");
	p.getParam("maxbranchlength", maxbranchlength);
	p.getParam("optimize", optimize);
	ros::Subscriber start_sub = p.subscribe("/start_point",1, startpointCallback);
	ros::Subscriber end_sub = p.subscribe("/end_point",1, endpointCallback);
	ros::Subscriber map_sub = p.subscribe("/map_node/map",1, mapCallback);
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
			cout<<"Path published"<<endl;
			path.poses.clear();
			start_ready = false;
			end_ready = false;
			map_ready = false;
			count++;
			cout<<"Press any key to continue"<<endl;
			cv::waitKey(0);
			cv::destroyAllWindows();
			ros::spinOnce();
		}
        }
        return 0;
}

