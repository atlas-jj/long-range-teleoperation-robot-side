#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <map>
#include <ctime>
#include <sstream>
#include <algorithm>
#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "math_helper.h"
#include "string_convertor.h"
#include "transformation2D.h"
//#include "signature_visualization/pathData.h"
#include "wam_msgs/MatrixMN.h"
#include "wam_srvs/JointMove.h"
#include "sensor_msgs/JointState.h"

using namespace cv;
using namespace std;

cv::Mat recvImg;

wam_srvs::JointMove mv_srv;
ros::ServiceClient Joint_move_client ;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
   try
    {
      recvImg=cv_bridge::toCvShare(msg, "bgr8")->image;//using for simulation display
      namedWindow("Alignment",CV_WINDOW_NORMAL);

      imshow("Alignment", recvImg);
      // cv::waitKey(30);
    }//end of try
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
 }

//inStr "0 0 0 0"
void moveRobot(string inStr)
{
  std::vector<double> pos = string_convertor::fromString2Array(inStr);
  mv_srv.request.joints = string_convertor::convert2Float(pos);
  cout<<"send to robot to the  position ";
  string_convertor::printOutStdVector(pos);
  cout << "Press any key to continue..." << endl;
  getchar();
  Joint_move_client.call(mv_srv);///////////////////////////////////////////////////
}

//inStr "0 0 0 0"
void moveRobotByVect(std::vector<double> pos)
{
  mv_srv.request.joints = string_convertor::convert2Float(pos);
  cout<<"send to robot to the  position ";
  string_convertor::printOutStdVector(pos);
  // cout << "Press any key to continue..." << endl;
  // getchar();
  Joint_move_client.call(mv_srv);///////////////////////////////////////////////////
  boost::this_thread::sleep( boost::posix_time::milliseconds(1000) );
}

void moveRobotByJointRange(string initialStr, int jointIndex, double minVal, double maxVal, double incremental)
{
  std::vector<double> pos = string_convertor::fromString2Array(initialStr);
  double initial = pos[jointIndex];
  double posTempt = initial;
  //from initial to minVal,
  while(posTempt>=minVal)
  {
     posTempt = posTempt - incremental;
     pos[jointIndex]=posTempt;
     moveRobotByVect(pos);
  }
  //from minVal to maxVal,
  while(posTempt<=maxVal)
  {
     posTempt = posTempt + incremental;
     pos[jointIndex]=posTempt;
     moveRobotByVect(pos);
  }
  //from maxVal to initial
  while(posTempt>=initial)
  {
     posTempt = posTempt - incremental;
     pos[jointIndex]=posTempt;
     moveRobotByVect(pos);
  }
}

void moveRobotByJointChange(string initialStr, int jointIndex, double destVal, double incremental)
{
  std::vector<double> pos = string_convertor::fromString2Array(initialStr);
  double initial = pos[jointIndex];
  //check incremental
  if((destVal - initial) * incremental<0)
     incremental = 0 - incremental;

  double posTempt = initial;
  while((destVal - posTempt)* incremental>0)
  {
     posTempt = posTempt + incremental;
     pos[jointIndex]=posTempt;
     moveRobotByVect(pos);
  }
}

void pressKey2Continue(int seqI)
{
  cout<<"move chris sequence  "<<seqI<<endl;
  cout << "Press any key to continue..." << endl;
  getchar();
}
//===========================MAIN FUNCTION START===========================

int main(int argc, char* argv[]){
  double incremental = 0.05;
  ros::init(argc, argv, "controller");
  ros::NodeHandle nh;
  //ros::Publisher pubTask = nh.advertise<signature_visualization::pathData>("/path_data", 1, true);//task will be only published once
  //ros::Publisher pubTask2 = nh.advertise<std_msgs::String>("/chris/strokes", 1, true);//task will be only published once
  //image_transport::ImageTransport it(nh);
  //image_transport::Subscriber subImage = it.subscribe("/camera/rgb/image_rect_color", 1, imageCallback);///camera/rgb/image_rect_color  /usb_cam/image_raw
  //ros::Subscriber sub2 = nh.subscribe("/chris/targetPoints_2D", 1000, signature_data_callback);
  Joint_move_client = nh.serviceClient<wam_srvs::JointMove>("/zeus/wam/joint_move");
  moveRobot("-1.4 -0.43 0 2.24 0 0 0");
  cout<<"close to faraway"<<endl;
  cout << "Press any key to continue..." << endl;
  getchar();
  for(int i=0;i>-20;i--) //close to faraway
  {
    std::vector<double> pos;
    double dp = (double)i/100;
    pos.push_back(-1.4 );
    pos.push_back(-0.43 + dp);
    pos.push_back(0);
    pos.push_back(2.24 - dp);
    pos.push_back(0);
    pos.push_back(0);
    pos.push_back(0);
    moveRobotByVect(pos);
  }
  moveRobotByJointChange("-1.4 -0.35 0 2.24 0 0 0", 0, -1.6, incremental);
  //now from middle to left and right
  //pressKey2Continue(1);
  moveRobotByJointChange("-1.6 -0.35 0 2.15 0 0 0", 0, 0.3 , incremental);
  //pressKey2Continue(2);
  moveRobotByJointChange("0.3 -0.35 0 2.20 0 0 0", 0, -0.2 , incremental);
  moveRobotByJointChange("-0.2 -0.35 0 2.30 0 0 0", 0, -1.4, incremental);

  moveRobotByJointChange("-1.4 -0.35 0 2.40 0 0 0", 0, 0.3, incremental);
  moveRobotByJointChange("0.3 -0.35 0 2.22 0 0 0", 0, -1.4 , incremental);
  // pressKey2Continue(3);
  // moveRobotByJointRange("-1.4 -0.43 0 2.45 0 0 0", 0, -1.6,  -0.4, incremental);
  //pressKey2Continue(6);
  moveRobotByJointChange("-1.4 -0.35 0 2.15 0 0 0", 0, 0, incremental);
  moveRobotByJointChange("0 -0.35 0 2.15 0 0 0", 1, 0.2, incremental);
  moveRobotByJointChange("0 0.2 0 2.15 0 0 0", 0, 0.4, incremental);
  moveRobotByJointChange("0.4 0.2 0 2.0 0 0 0", 0, 0, incremental);
  moveRobotByJointRange("0.0 0.2 0 1.9 0 0 0", 2, -0.1,0.5, incremental);
  moveRobotByJointChange("0.0 0.2 0 1.9 0 0 0", 0, 0.3, incremental);
  moveRobotByJointChange("0.3 0.2 0 1.9 0 0 0", 3, 2.1, incremental);
  moveRobotByJointChange("0.3 0.2 0 2.1 0 0 0", 0, 0.5, incremental);
  moveRobotByJointChange("0.5 0.2 0 2.1 0 0 0", 1, 0, incremental);
  ros::spinOnce();


    //cv::namedWindow("view");
    //cv::startWindowThread();

      //declaration of function
  	//cv::destroyWindow("view");
    return 0;
}
