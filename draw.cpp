#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>
using namespace cv;
Point start, end;
Mat canvas;
static void onMouse( int event, int x, int y, int, void*)
{
    	if( event == EVENT_LBUTTONDOWN ){
		start = Point(x,y);
		return;
      	}
	else if(event == EVENT_LBUTTONUP){
		end = Point(x,y);
		line(canvas, start, end, Scalar( 255, 255, 255 ), 2);//red line
		imshow("Draw map", canvas); 
		return;
	}
	else{return;}
}



int main( int argc, char** argv ) {
  
  canvas = Mat::zeros(500, 500, CV_8UC3);
  namedWindow( "Draw map", WINDOW_AUTOSIZE );
  setMouseCallback( "Draw map", onMouse, 0 );
  imshow( "Draw map", canvas );
  while(true){
 	char key = (char) waitKey(0);
	if(key == 27){
  		imwrite("src/map_server/src/userdrawn.jpg", canvas);
		break;
	}
  }
  return 0;
}
