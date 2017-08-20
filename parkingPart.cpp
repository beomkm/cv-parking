#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <sherlotics/variable.hpp>
#include <cstdio>

#define ROI1_X 320
#define ROI1_Y 100
#define ROI1_WIDTH 320
#define ROI1_HEIGHT 380

#define ROI2_X 0
#define ROI2_Y 100
#define ROI2_WIDTH 640
#define ROI2_HEIGHT 380


using namespace cv;
using namespace std;

ros::Publisher pub;

Mat frame;

Mat gray1;
Mat binOpen1;
Mat binClose1;
Mat canny;

Mat gray2;
Mat binOpen2;

Mat labels1, stats1, centroids1;
int numLabels1;

Mat labels2, stats2, centroids2;
int numLabels2;

int state = 0;
int stateCnt = 0;
int actNo = 0;


std::vector<Vec4i> lines;




void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    frame = cv_bridge::toCvShare(msg, "bgr8")->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }

  resize(frame, frame, Size(640, 480), 0, 0, CV_INTER_CUBIC);
  /**************subscribe******************/

  if(0){
    robotMode = PARKING_MODE;
  }
  //PARKING_MODE
  //NORMAL_TO_PARKING
  //NORMAL



  /****************process******************/
  Rect roi_rect1 = Rect(ROI1_X, ROI1_Y, ROI1_WIDTH, ROI1_HEIGHT);
  Mat roi1 = frame(roi_rect1);
  cvtColor(roi1, gray1, CV_BGR2GRAY);


  Rect roi_rect2 = Rect(ROI2_X, ROI2_Y, ROI2_WIDTH, ROI2_HEIGHT);
  Mat roi2 = frame(roi_rect2);
  cvtColor(roi2, gray2, CV_BGR2GRAY);



  //adaptiveThreshold(gray, bin, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 37, 5);
  threshold(gray1, binOpen1, 96, 255, THRESH_BINARY_INV);
  threshold(gray1, binClose1, 96, 255, THRESH_BINARY_INV);

  threshold(gray2, binOpen2, 96, 255, THRESH_BINARY_INV);
  //threshold(gray, bin, 84, 255, THRESH_BINARY);


  Mat element5(5, 5, CV_8U, Scalar(1));
  morphologyEx(binOpen1, binOpen1,  MORPH_OPEN, element5);
  morphologyEx(binClose1, binClose1,  MORPH_CLOSE, element5);

  morphologyEx(binOpen2, binOpen2,  MORPH_OPEN, element5);

  Canny(binClose1, canny, 125, 250);



  numLabels1 = connectedComponentsWithStats(binOpen1, labels1, stats1, centroids1, 8, CV_32S);
  numLabels2 = connectedComponentsWithStats(binOpen2, labels2, stats2, centroids2, 8, CV_32S);
  //HoughLinesP(canny, lines, 1, CV_PI/180, 40, 20, 20);
  HoughLinesP(canny, lines, 1, CV_PI/180, 30, 20, 20);

  int numRect = 0;
  for(int i=1; i<numLabels1; i++) {
    int area = stats1.at<int>(i, CC_STAT_AREA);
    int left = stats1.at<int>(i, CC_STAT_LEFT);
    int top = stats1.at<int>(i, CC_STAT_TOP);
    int width = stats1.at<int>(i, CC_STAT_WIDTH);
    int height = stats1.at<int>(i, CC_STAT_HEIGHT);

    if(width<190 && height<60 && top>40) {
      ++numRect;
    }
  }

  //imshow("park", binOpen2);
  int numStop = 0;
  int numStopY = 0;
  for(int i=1; i<numLabels2; i++) {
    int area = stats2.at<int>(i, CC_STAT_AREA);
    int left = stats2.at<int>(i, CC_STAT_LEFT);
    int top = stats2.at<int>(i, CC_STAT_TOP);
    int width = stats2.at<int>(i, CC_STAT_WIDTH);
    int height = stats2.at<int>(i, CC_STAT_HEIGHT);


    if(width>600 && height<200 && (top+height/2)>120) {
      ++numStop;
      numStopY = top+height/2;
    }
  }


  switch(state) {
    case 0:
      actNo = 1;
      if(numRect == 3) {
        ++stateCnt;
      }
      else if(numRect != 4 && numRect != 5) {
        stateCnt = 0;
      }
      if(stateCnt >= 8) {
        stateCnt = 0;
        state = 1;
      }

      break;
    case 1:
      actNo = 4;
      ++stateCnt;
      if(stateCnt > 76) {
        stateCnt = 0;
        state = 2;
      }
      break;
    case 2:
      actNo = 0;
      if(numStop > 0) {
        ++stateCnt;
      }
      else {
        --stateCnt;
      }
      if(stateCnt >= 8) {
        stateCnt = 0;
        state = 3;
      }
      else if(stateCnt <= -8) {
        stateCnt = 0;
        state = 8;
      }
      break;
    case 3:
      actNo = 1;
      if(numStop == 0) {
        ++stateCnt;
      }
      else {
        stateCnt = 0;
      }
      if(stateCnt >= 4) {
        stateCnt = 0;
        state = 4;
      }
      break;
    case 4:
      actNo = 0;
      ++stateCnt;
      if(stateCnt >= 20) {
        stateCnt = 0;
        state = 5;
      }
      break;
    case 5:
      actNo = 2;
      if(numStop == 1 && numStopY>180 && numStopY<250) {
        ++stateCnt;
      }
      else {
        stateCnt = 0;
      }
      if(stateCnt >= 6) {
        stateCnt = 0;
        state = 6;
      }
      break;
    case 6:
      actNo = 3;
      ++stateCnt;
      if(stateCnt > 76) {
        stateCnt = 0;
        state = 7;
      }
      break;
    case 7:
      actNo = 1;
      robotMode = NORMAL;
      break;
    case 8:
      actNo = 3;
      ++stateCnt;
      if(stateCnt > 76) {
        stateCnt = 0;
        state = 9;
      }
      break;
    case 9:
      actNo = 1;
      if(numRect == 0) {
        ++stateCnt;
      }
      else {
        stateCnt = 0;
      }
      if(stateCnt >= 28) {
        stateCnt = 0;
        state = 10;
      }
      break;
    case 10:
      actNo = 4;
      ++stateCnt;
      if(stateCnt > 76) {
        stateCnt = 0;
        state = 3;
      }
      break;
    default:
      actNo = 0;
  }


  //printf("////////// [ %d ]\n", state);

  for(int i=0; i<lines.size(); i++) {
    Vec4i lv = lines[i];
    float dist = norm(Mat(Point(lv[0], lv[1])), Mat(Point(lv[2], lv[3])));
    int near = 0;
    for(int j=1; j<numLabels1; j++) {
      int left = stats1.at<int>(j, CC_STAT_LEFT);
      int top = stats1.at<int>(j, CC_STAT_TOP);
      int width = stats1.at<int>(j, CC_STAT_WIDTH);
      int height = stats1.at<int>(j, CC_STAT_HEIGHT);
      int px = left + width/2;
      int py = top + height/2;
      float rho = (float)((px-lv[0])*(lv[3]-lv[1]) - (py-lv[1])*(lv[4]-lv[2]))/dist;
      if(rho<400 && rho>-400) {
        near += 1;
      }
    }
  }





  /****************publish******************/

  if(robotMode == PARKING_MODE){
    std_msgs::Float32MultiArray PARKINGtoTELEOP_msg;
    PARKINGtoTELEOP_msg.data.clear();
    //float linear_velocity = 0.0;
    //float angular_velocity = 0.0;
    //float sendData[2] = {linear_velocity,angular_velocity};
    float sendData[2];
    actToOutput(actNo, sendData);

    for (int i = 0; i < 2; i++)
      {
          PARKINGtoTELEOP_msg.data.push_back(sendData[i]);
      }
    pub.publish(PARKINGtoTELEOP_msg);
  }

  waitKey(WAITKEYSIZE);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "parkingPart_node");

  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("parkingMode_msg", QUEUESIZE, imageCallback);

  pub = nh.advertise<std_msgs::Float32MultiArray>("TELEOP_msg", QUEUESIZE);

  ros::spin();

  return 0;
}


void actToOutput(int act, float *output)
{
  switch(act) {
    case 0:
      robotMode = PARKING_MODE;
      output[0] = 0.0f;
      output[1] = 0.0f;
      break;
    case 1:
      robotMode = NORMAL_TO_PARKING;
      output[0] = 0.03f;
      output[1] = 0.0f;
      break;
    case 2:
      robotMode = PARKING_MODE;
      output[0] = 0.03f;
      output[1] = 0.0f;
      break;
    case 3:
      robotMode = PARKING_MODE;
      output[0] = 0.0f;
      output[1] = 0.1f;
      break;
    case 4:
      robotMode = PARKING_MODE;
      output[0] = 0.0f;
      output[1] = -0.1f;
      break;
  }
}
