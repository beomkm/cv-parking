#include "opencv2/opencv.hpp"
#include <iostream>
#include <cmath>
#include <unistd.h>


using namespace cv;
using namespace std;

const int WIDTH = 640;
const int HEIGHT = 480;
const int ROI_X = 320;
const int ROI_Y = 100;
const int ROI_WIDTH = 320;
const int ROI_HEIGHT = 380;



int main(int, char**)
{

	VideoCapture capture("new_secondobst.avi");
	if (!capture.isOpened()) {
		cout << "Cannot open devices" << endl;
		return 1;
	}


	capture.set(CAP_PROP_FRAME_WIDTH, WIDTH);
	capture.set(CAP_PROP_FRAME_HEIGHT, HEIGHT);

	Mat frame;

	namedWindow("input", WINDOW_NORMAL);
	namedWindow("output1", WINDOW_NORMAL);
	namedWindow("output2", WINDOW_NORMAL);
	resizeWindow("input", ROI_WIDTH, ROI_HEIGHT);
	resizeWindow("output1", ROI_WIDTH, ROI_HEIGHT);
	resizeWindow("output2", ROI_WIDTH, ROI_HEIGHT);

	Mat gray;
	Mat bin1;
	Mat bin2;
	Mat canny;

	Mat labels, stats, centroids;
	int numLabels;

	int state = 0;
	int stateCnt = 0;

	vector<Vec4i> lines;

	while(1) {

		capture >> frame;
		if(frame.empty()) {
			break;
		}


		Rect roi_rect = Rect(320, 100, 320, 380);
		Mat roi = frame(roi_rect);

		cvtColor(roi, gray, CV_RGB2GRAY);
		//adaptiveThreshold(gray, bin, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 37, 5);
		threshold(gray, bin1, 96, 255, THRESH_BINARY_INV);
		threshold(gray, bin2, 96, 255, THRESH_BINARY_INV);

		//threshold(gray, bin, 84, 255, THRESH_BINARY);


		Mat element5(5, 5, CV_8U, Scalar(1));
		morphologyEx(bin1, bin1,  MORPH_OPEN, element5);
		morphologyEx(bin2, bin2,  MORPH_CLOSE, element5);

		Canny(bin2, canny, 125, 250);


		numLabels = connectedComponentsWithStats(bin1, labels, stats, centroids, 8, CV_32S);


		//HoughLinesP(canny, lines, 1, CV_PI/180, 40, 20, 20);
		HoughLinesP(canny, lines, 1, CV_PI/180, 30, 20, 20);

		//draw grid
		for(int i=0; i<ROI_WIDTH; i+=50) {
			line(roi, Point(i, 0), Point(i, ROI_HEIGHT), Scalar(255, 255, 0), 1);
		}

		for(int i=0; i<ROI_HEIGHT; i+=50) {
			line(roi, Point(0, i), Point(ROI_WIDTH, i), Scalar(255, 255, 0), 1);
		}


		int numRect = 0;
		for(int i=1; i<numLabels; i++) {
			int area = stats.at<int>(i, CC_STAT_AREA);
			int left = stats.at<int>(i, CC_STAT_LEFT);
			int top = stats.at<int>(i, CC_STAT_TOP);
			int width = stats.at<int>(i, CC_STAT_WIDTH);
			int height = stats.at<int>(i, CC_STAT_HEIGHT);

			if(area > 30 && area < 4900) { //70*70
				rectangle(roi, Point(left, top), Point(left+width, top+height), Scalar(0, 255, 0), 1);
			}

			if(height<50 && top>40) {
				++numRect;
			}
		}
		printf("%d\n", numRect);




		for(int i=0; i<lines.size(); i++) {
			Vec4i lv = lines[i];
			float dist = norm(Mat(Point(lv[0], lv[1])), Mat(Point(lv[2], lv[3])));
			int near = 0;
			for(int j=1; j<numLabels; j++) {
				int left = stats.at<int>(j, CC_STAT_LEFT);
				int top = stats.at<int>(j, CC_STAT_TOP);
				int width = stats.at<int>(j, CC_STAT_WIDTH);
				int height = stats.at<int>(j, CC_STAT_HEIGHT);
				int px = left + width/2;
				int py = top + height/2;
				float rho = (float)((px-lv[0])*(lv[3]-lv[1]) - (py-lv[1])*(lv[4]-lv[2]))/dist;
				if(rho<400 && rho>-400) {
					near += 1;
				}

			}
			if(near > 5)
				line(roi, Point(lv[0], lv[1]), Point(lv[2], lv[3]), Scalar(0, 0, 255), 1, CV_AA);
			else
				line(roi, Point(lv[0], lv[1]), Point(lv[2], lv[3]), Scalar(0, 255, 255), 1, CV_AA);





		}



		//imshow("input", roi);
		//imshow("output1", bin1);
		//imshow("output2", canny);  

		if (waitKey(60000) == 'n') continue;

	}



	return 0;
}
