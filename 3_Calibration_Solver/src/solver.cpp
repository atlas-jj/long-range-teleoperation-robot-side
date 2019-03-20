
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <opencv2/opencv.hpp>


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

#include "colormod.h" // namespace Color

#ifndef Included_MATH_HELPER_H
#define Included_MATH_HELPER_H
#include "math_helper.h"
#endif
#ifndef Included_STRING_CONVERTOR_H
#define Included_STRING_CONVERTOR_H
#include "string_convertor.h"
#endif
#include "transformation2D.h"
#include "keyFrame.h"
#include "pair_solver.h"
#include "chris_ICP.h"


using namespace std;

Color::Modifier c_red(Color::FG_RED);
Color::Modifier c_yellow(Color::FG_YELLOW);
Color::Modifier c_green(Color::FG_GREEN);
Color::Modifier c_default(Color::FG_DEFAULT);
int numThres =100;
pair_solver mySolver(numThres);

keyFrame previousKF;
keyFrame currentRobotPose;
string currentRobotPoseStr="";
ofstream f1;


void twc_callback(const std_msgs::String::ConstPtr& msg)
 {
   //cout<<"twc_callback"<<endl;
    std::string str= msg->data.c_str();
    vector<string> sta4=string_convertor::split(str,',');
    //cout<<c_yellow<<str<<endl<<c_default;
    if(sta4.size()==18)
    {
       int mnid = std::atoi(sta4[0].c_str());
       double tsecs = atof(sta4[1].c_str());
       cv::Mat currentTWC(4,4,cv::DataType<double>::type);
       for(int i=2;i<sta4.size();i++)
       {
         double tempt=atof(sta4[i].c_str());
         int row=(i-2)/4;
         int col=i-2-4*row;
      	 currentTWC.at<double>(row,col)=tempt;
      }
      //cout<<"current TWC"<<currentTWC<<endl;
      keyFrame kf(mnid, tsecs,currentTWC);
      if(previousKF.mnFrameId ==-1)
        previousKF = kf;
      else if(keyFrame::getBaseLine(previousKF.translation,kf.translation)>0.02)
      {
        //get robot pose, add to pair
        cout<<c_red<<"add to pair, baseline: "<<keyFrame::getBaseLine(previousKF.translation,kf.translation)<<endl<<c_default;
        if(currentRobotPose.mnFrameId!=-1)
        {
          mySolver.AddNewPair(currentRobotPose, kf);
          f1<<str<<endl;
          f1<<currentRobotPoseStr<<endl;
          cout << endl << "paired samples saved!" << endl;
        }
        cout<<c_green<<"pair Count: "<<mySolver.GetPairCount()<<endl<<c_default;
        cout<<"numthres"<<numThres<<endl;
        previousKF = kf;
      }

      if(mySolver.GetPairCount()>=numThres && !(mySolver.isReady()))
      {
            //solve the eqations
            mySolver.solveTranslationICP();
            mySolver.solveOrientationSDP();
            mySolver.Save2File();

      }
      //else //wait untill the base line is Good
    }

}

void solveByLoadingFile()
{
    std::ifstream file("/home/chris/catkin_ws/src/long_range_teleoperation/robot-side/3_Calibration_Solver/dataset/data_backup.txt");
    std::string str;
    int count=0;
    keyFrame cameraPose;
    while (std::getline(file, str))
    {
      cout<<str<<endl;
      count ++;
      if(count%2==0)  //robot pose
      {
        keyFrame kf(str, 1);
        mySolver.AddNewPair(kf, cameraPose);
        cout <<"paired samples saved!" << endl;
      }
      else
      {
        keyFrame kf(str, 0);
        cameraPose = kf;
      }
    }

    //if(mySolver.GetPairCount()>=numThres && !(mySolver.isReady()))
    {
      cout<<"start solving"<<endl;
          //solve the eqations
          mySolver.solveTranslationICP();
          mySolver.solveOrientationSDP();
          mySolver.Save2File();

    }

}
string addDouble(double d)
{
  std::ostringstream ssss;
  ssss << d;
  return ssss.str();
}

string recordRobotPoseString(int mnid, double timesecs,double x, double y, double z, double qx, double qy, double qz, double qw)
{
  std::stringstream ss;
  ss<<mnid<<",";
  ss<<std::setprecision(15)<<timesecs<<",";
  std::ostringstream ssss;
  ss<<addDouble(x)<<",";
  ss<<addDouble(y)<<",";
  ss<<addDouble(z)<<",";
  ss<<addDouble(qx)<<",";
  ss<<addDouble(qy)<<",";
  ss<<addDouble(qz)<<",";
  ss<<addDouble(qw)<<",";
  return ss.str();
}
void wamPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  //cout<<"wam msgs"<<endl;
   geometry_msgs::Pose thisPose = msg->pose;
   //cout<<c_yellow<<"current pose: "<<thisPose<<endl<<c_default;
   //cout<<"current header: "<<msg->header.stamp.toSec()<<endl;
   //note that: the robot pose, is actually, translation: from robot to endeffector. rotation: from robot to endeffector.
   //but what we use in keyframe class is, translation: from robot to end effector. Rotation: from endeffector to robot
   //so we need to inverse the robot's rotation.
   std::vector<double> v = math_helper::getInverseQuaternion(thisPose.orientation.x, thisPose.orientation.y, thisPose.orientation.z, thisPose.orientation.w);

   keyFrame fr(mySolver.GetPairCount(),msg->header.stamp.toSec(), thisPose.position.x, thisPose.position.y, thisPose.position.z, v[0], v[1], v[2], v[3]);
   currentRobotPose=fr;
   currentRobotPoseStr = recordRobotPoseString(mySolver.GetPairCount(),msg->header.stamp.toSec(), thisPose.position.x, thisPose.position.y, thisPose.position.z, v[0], v[1], v[2], v[3]);
}





//===========================MAIN FUNCTION START===========================

int main(int argc, char* argv[]){
  // Initialize the ROS system and become a node.
  bool loadFromtxtFile=false;
   if(argc==2){
        numThres = atoi(argv[1]);
        loadFromtxtFile = true;
      }

   ros::init(argc, argv, "solver");
   ros::start();
   ros::NodeHandle nh;

   if(!loadFromtxtFile)
   {
     f1.open("/home/chris/catkin_ws/src/long_range_teleoperation/robot-side/3_Calibration_Solver/dataset/data.txt");
     f1 << fixed;

    }
    else
    {
      solveByLoadingFile();
    }
    ros::Subscriber sub2 = nh.subscribe("/chris/twc", 1000, twc_callback);
    ros::Subscriber subP = nh.subscribe("/zeus/wam/pose", 1000, wamPoseCallback);
   //waitKey();
   //chris_ICP::test("p.txt");
   ros::spin();
     //declaration of function
   ros::shutdown();

   f1.close();

   return 0;
}
