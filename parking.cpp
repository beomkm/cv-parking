#include "opencv2/opencv.hpp"  
#include <iostream>  
#include <unistd.h>
  
using namespace cv;  
using namespace std;  
  


int main(int, char**)  
{  

	VideoCapture capture("new_firstobst.avi");  
	if (!capture.isOpened()) {  
		cout << "Cannot open devices" << endl;
		return 1;
	}  


	capture.set(CAP_PROP_FRAME_WIDTH, 640);  
	capture.set(CAP_PROP_FRAME_HEIGHT, 480);  

	Mat frame;  

	namedWindow("input", WINDOW_NORMAL);  
	namedWindow("output1", WINDOW_NORMAL);  
	namedWindow("output2", WINDOW_NORMAL);  
	resizeWindow("input", 640, 480);
	resizeWindow("output1", 640, 480);
	resizeWindow("output2", 640, 480);

	Mat gray;
	Mat bin1;
	Mat bin2;
	Mat canny;

	Mat labels, stats, centroids;
	int numLabels;

	vector<Vec4i> lines;

	while(1) {  

		capture >> frame;  
		if(frame.empty()) {
			break;
		}

		cvtColor(frame, gray, CV_RGB2GRAY);
		//adaptiveThreshold(gray, bin, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 37, 5);
		threshold(gray, bin1, 96, 255, THRESH_BINARY_INV);
		threshold(gray, bin2, 96, 255, THRESH_BINARY_INV);

		//threshold(gray, bin, 84, 255, THRESH_BINARY);

		
		Mat element5(5, 5, CV_8U, Scalar(1));
		morphologyEx(bin1, bin1,  MORPH_OPEN, element5);
		morphologyEx(bin2, bin2,  MORPH_CLOSE, element5);

		Canny(bin2, canny, 125, 250);

	
		numLabels = connectedComponentsWithStats(bin1, labels, stats, centroids, 8, CV_32S);


		HoughLinesP(canny, lines, 1, CV_PI/180, 40, 20, 20);

		for(int i=0; i<numLabels; i++) {
			int left = stats.at<int>(i, CC_STAT_LEFT);
			int top = stats.at<int>(i, CC_STAT_TOP);
			int width = stats.at<int>(i, CC_STAT_WIDTH);
			int height = stats.at<int>(i, CC_STAT_HEIGHT);

			rectangle(frame, Point(left, top), Point(left+width, top+height), Scalar(0, 255, 0), 1);
		}

		for(int i=0; i<lines.size(); i++) {
			Vec4i lv = lines[i];
			line(frame, Point(lv[0], lv[1]), Point(lv[2], lv[3]), Scalar(0, 255, 255), 1, CV_AA);
		}
	
		imshow("input", frame);  
		imshow("output1", bin1);  
		imshow("output2", canny);  

		if (waitKey(60000) == 'n') continue;

	}  



	return 0;  
}  

