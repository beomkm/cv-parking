#include "opencv2/opencv.hpp"  
#include <iostream>  
  
using namespace cv;  
using namespace std;  
  


int main(int, char**)  
{  

	VideoCapture capture(0);  
	if (!capture.isOpened()) {  
		cout << "Cannot open devices" << endl;
		return 1;
	}  


	capture.set(CAP_PROP_FRAME_WIDTH, 640);  
	capture.set(CAP_PROP_FRAME_HEIGHT, 480);  

	Mat frame;  
	namedWindow("input", 1);  
	resizeWindow("input", 640, 480);

	Size size = Size((int)capture.get(CAP_PROP_FRAME_WIDTH),
				(int)capture.get(CAP_PROP_FRAME_HEIGHT));
	int fps = capture.get(CAP_PROP_FPS);
	VideoWriter outputVideo;
	outputVideo.open("output.avi", VideoWriter::fourcc('X','V','I','D'),
					fps, size, true);
	if(!outputVideo.isOpened()) {
		cout << "Failed to write videofile." << endl;
		return 1;
	}


	while(1) {  

		capture >> frame;  

		imshow("input", frame);  
		outputVideo << frame;


		if (waitKey(1) == 27 ) break;
	}  


	return 0;  
}  

