
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
//#include <opencv2/opencv.hpp>

#include <iostream>
#include <cstdlib>
#include <fstream>
#include <exception>
#include <vector>

#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <stdio.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <stdlib.h>
#include <arpa/inet.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
#include <boost/asio.hpp>
#include <ctime>
#include <sys/time.h>
//#include <boost/asio/use_future.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "math_helper.h"
#include "string_convertor.h"
#include "colormod.h"

#define BUFLEN 272000  //Max length of buffer
string UDP_SERVER_IP="103.125.217.74";//"172.31.1.147"//define the udp server ip address  //"127.0.0.1" "172.31.1.147"/
string PORT="30002"; //The port on which to listen for incoming data
int maxPackageSize=1000;
using boost::asio::ip::udp;
enum { max_length = 1024 };

//using namespace cv;
using namespace std;

string logFileName = "log.txt";
std::ofstream logFileStream;

Color::Modifier c_red(Color::FG_RED);
Color::Modifier c_yellow(Color::FG_YELLOW);
Color::Modifier c_green(Color::FG_GREEN);
Color::Modifier c_default(Color::FG_DEFAULT);

ros::Publisher pubCommand;

boost::asio::io_service io_service;
udp::socket s(io_service, udp::endpoint(udp::v4(), 0));

int data_size=0;
string endTag="clc";
string fake_coarse_target_data_string = "";
int cmd_index = 0;
string image_base64Str = "null";
udp::resolver resolver(io_service);
udp::resolver::iterator iterator4 ;

bool connected = false;
time_t lastConnected =  time(0);
double reConnDetectCycle = 20; //detect connection status and decide reConnection threshold.

void Connect();
void setConnected();
udp::resolver::query query(udp::v4(), UDP_SERVER_IP , PORT);

string to_string(int i)
{
  std::stringstream ss;
  ss<<i;
  return ss.str();
}

bool packageValid(string inputStr)
{
   if(inputStr.find(endTag) != std::string::npos)
     return true;
   else
     return false;
}


void write2log(string logStr)
{
  try{
      //std::ofstream fileOut(logFileName.c_str(), std::ios::out);
      struct timeval tp;
      gettimeofday(&tp,NULL);
      long int ms = tp.tv_sec * 1000 + tp.tv_usec / 1000;

      logFileStream<< logStr <<","<<setprecision(15)<< ms*0.001 << "\n";
  }
  catch(exception &e) {
   std::cout << "Catch an exception: " << e.what() << std::endl;
  }
}

//return total data size
int sendStr(string sendback_cmd)
{
  sendback_cmd +=endTag;
  try {
    //get robot position x,y,z
    int myArrayLength=sendback_cmd.size();
    char myArray[myArrayLength];//as 1 char space for null is also required
    strcpy(myArray, sendback_cmd.c_str());

    boost::asio::socket_base::send_buffer_size option(BUFLEN);//BUFLEN
    if(myArrayLength*8*8>=212992) //3328
           cout<<"warning: data length beyond UDP datagram."<<endl;
    //if data length is larger the UDP datagram size, split it into several packages.
    int packageNum=myArrayLength/maxPackageSize;
    if(packageNum==0)
      s.send_to(boost::asio::buffer(myArray, myArrayLength), *iterator4);
    else
      {
        for(int i=0;i<packageNum+1;i++)
        {
            //get different package
          int thisPackageSize=maxPackageSize;
          if(i==packageNum)
              thisPackageSize=myArrayLength-maxPackageSize*packageNum;
          char toSendArray[thisPackageSize];
          int startIndex=0+i*maxPackageSize;
          for(int m=0;m<thisPackageSize;m++)
              toSendArray[m]=myArray[startIndex+m];
          s.send_to(boost::asio::buffer(toSendArray, thisPackageSize), *iterator4);
          //cout<<toSendArray<<'\0'<<endl;
          boost::this_thread::sleep(boost::posix_time::milliseconds(100));
        }
      }
    std::cout << "sent txt:  "<<sendback_cmd<<endl;
    return myArrayLength*8;
  }
  catch(exception &e) {
   std::cout << "Catch an exception: " << e.what() << std::endl;
   return 0;
 }
}

void carvScriptCallback(const std_msgs::String::ConstPtr& msg)
{
  //send the command to remote robot
  cout<<"recved: "<<endl;//<<msg->data<<endl;
  if(msg->data!="")
  {
    cmd_index++;
    string toSendstr = to_string(cmd_index) + "#CARV#" + msg->data;
    int sendByteSize = sendStr(toSendstr);
    //write to log script
    write2log("SEND,"+to_string(cmd_index)+",CARV," + to_string(sendByteSize));
  }
}

void coarseTaskStatusCallback(const std_msgs::String::ConstPtr& msg)
{
  //send the command to remote robot
  //cout<<"recved: "<<endl;//<<msg->data<<endl;
  if(msg->data!="")
  {
    cmd_index++;
    string toSendstr = to_string(cmd_index) + "#COARSE_DONE#" + msg->data;
    int sendByteSize = sendStr(toSendstr);
    write2log("SEND,"+to_string(cmd_index)+",COARSE_DONE," + to_string(sendByteSize));
  }
}

void faceCoarseTargetCallback(const std_msgs::String::ConstPtr& msg)
{
  //send the command to remote robot
  //cout<<"recved: "<<endl;//<<msg->data<<endl;
  if(msg->data!="" && msg->data !=fake_coarse_target_data_string)
  {
    fake_coarse_target_data_string = msg->data;
    // string strTemp = "coarse_target_move:1:"+fake_coarse_target_data_string + ",1"; //combine to a new command
    // std_msgs::String msg22;
    // msg22.data = strTemp;
    // pubCommand.publish(msg22);
  }
}
void imgStrCallback(const std_msgs::String::ConstPtr& msg)
{
  //send the command to remote robot
  //cout<<"recved: "<<endl;//<<msg->data<<endl;
  if(msg->data!="")
  {
    image_base64Str = msg->data;
  }
}

// void imageCallback(const sensor_msgs::ImageConstPtr& msg)
//  {
//    cv_bridge::CvImageConstPtr cv_ptr;
//    try
//    {
//        cv_ptr = cv_bridge::toCvShare(msg);
//        cv::Mat = cv_ptr->image,cv_ptr->header.stamp.toSec(); //make this to base64
//    }
//    catch (cv_bridge::Exception& e)
//    {
//        ROS_ERROR("cv_bridge exception: %s", e.what());
//        return;
//    }
//    //cv::Mat = cv_ptr->image,cv_ptr->header.stamp.toSec()
//  }



string recvr_txt()
{
  boost::this_thread::sleep(boost::posix_time::milliseconds(10));
  char in_reply[max_length];
  udp::endpoint sender_endpoint;
  // udp::endpoint listen_endpoint(
  //       boost::asio::ip::address::from_string(UDP_SERVER_IP),
  //       std::atoi(PORT));
  s.non_blocking(true);
  size_t reply_length = 0;
  try{
    reply_length = s.receive_from(boost::asio::buffer(in_reply, max_length), sender_endpoint);
  }
  catch(exception &e) {
    //std::cout << "Catch an exception: " << e.what() << std::endl;
  }

  if(reply_length>0)
  {
     char readfrom[reply_length];
     for(int i=0;i<reply_length;i++)
        readfrom[i]=in_reply[i];
     string inStr(readfrom);
     cout<<"received:"<<inStr<<endl;
     return inStr;
  }
  else
    return "";
}

void recvr_response(ros::NodeHandle nh)
{
  //publish coarse target and fine target to controller
  //if recvr request image, get the new image and send back to human side.
  string inStr = recvr_txt();
  //std::cout << "msg received:  "+inStr<<endl;
  //check connection status
  if(inStr.find("echo_test") != std::string::npos) //confirm connection established.
    setConnected();
  if (packageValid(inStr))//
  {
    try{
      int in_size = (inStr.size()) * 8;
      int endIndex=inStr.find(endTag);
      string strTemp=inStr.substr(0,endIndex); //split the end tag
      cout<<"strTemp:"<<strTemp<<endl;
      if(strTemp.find("RESP:") != std::string::npos) //that the response, log it
      {
        //cmdIndex:RESP:CARV:datasize
        std::vector<string> straa = string_convertor::split(strTemp, ':');
        if(straa.size() == 4)
            write2log("RESP," + straa[0] + "," + straa[2] + "," + straa[3]);
      }

      if(strTemp.find("require_image") != std::string::npos)
      {
        //get the latest image and send back to server.
        //return the newest image, resize it and then encode it to base64 str.
        // cmdIndexID#RQ_IMAGE#image data
        //,"+to_string(cmd_index)+",
        cout<<"command request image"<<endl;
        write2log("RECRV,RQ_IMAGE," + to_string(in_size));
        cmd_index ++;
        string toSendstr = to_string(cmd_index) + "#RQ_IMAGE#" + image_base64Str;
        int sendByteSize = sendStr(toSendstr);
        write2log("SEND,"+to_string(cmd_index)+",RQ_IMAGE," + to_string(sendByteSize));
        //send back RESP
      }

      if(strTemp.find("coarse_target") != std::string::npos)
      {
        //publish coarse target to coarse controller.
        cout<<"command goto coarse_target"<<endl;
        std::vector<string> straa = string_convertor::split(strTemp, ':');
        string datastr = straa[2];
        strTemp = "coarse_target_move:1:"+fake_coarse_target_data_string + "," + datastr; //combine to a new command
        write2log("RECRV,COARSE_TARGET," + to_string(strTemp.size()*8));
      }
      if(strTemp.find("fine_target") != std::string::npos)
      {
        //publish fine target to coarse controller
        //return the newest image, resize it and then encode it to base64 str.
        //coarse_target_move
        cout<<"command request fine_target"<<endl;
        write2log("RECRV,FINE_TARGET," + to_string(in_size));
      }
      std_msgs::String msg;
      msg.data = strTemp;
      pubCommand.publish(msg); //publish the new command. when other node processed, publish the same topic again and make it empty
    }
    catch(exception &e) {
      std::cout << "Catch an exception: " << e.what() << std::endl;
    }
  }

}

void checkConnection()
{
  double diffSeconds = difftime(time(0), lastConnected);
  if(diffSeconds>=reConnDetectCycle)
    Connect();
}

void setConnected()
{
  connected = true;
  lastConnected = time(0);
}

void Connect()
{
  connected = false;
  iterator4 = resolver.resolve(query);
  cout << "UDP client is trying to connect to ...."<< UDP_SERVER_IP<<":"<<PORT<< endl;
  //waiting for remote connection

  while(!connected)
  {
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    sendStr("test_conn:1:0");
    if(recvr_txt().find("echo_test") != std::string::npos) //confirm connection established.
      setConnected();
  }
  cout << "Connection established! ...."<< UDP_SERVER_IP<<":"<<PORT<< endl;
}

//===========================MAIN FUNCTION START===========================

int main(int argc, char* argv[]){

  if (argc >2) {
        UDP_SERVER_IP=argv[1];
        PORT=argv[2];
  }
  //rename a new log file

time_t theTime = time(NULL);
struct tm *aTime = localtime(&theTime);
int day = aTime->tm_mday;
int month = aTime->tm_mon + 1; // Month is 0 – 11, add 1 to get a jan-dec 1-12 concept
int year = aTime->tm_year + 1900; // Year is # years since 1900
int hour=aTime->tm_hour;
int min=aTime->tm_min;
int secs = aTime->tm_sec;
std::stringstream ss;
ss<<year<<month<<day<<"-"<<hour<<"-"<<min<<"-"<<secs;
logFileName = "log_"+ss.str()+".txt";
logFileStream.open (logFileName.c_str(), std::ofstream::out | std::ofstream::app);

  write2log("UDP Robot Client Side Started......");
  Connect();

  ros::init(argc, argv, "client");
  ros::NodeHandle nh;

  ros::Subscriber subP = nh.subscribe("/carv/script", 1, carvScriptCallback);
  ros::Subscriber subP2 = nh.subscribe("/coarse/task_status", 1, coarseTaskStatusCallback);
  ros::Subscriber subImg = nh.subscribe("img_str", 1, imgStrCallback);
  ros::Subscriber subFakeCoarseTarget = nh.subscribe("/coarse/target", 1, faceCoarseTargetCallback);
  //pubP = nh.advertise<std_msgs::String>("/rr/response", 1, true);//task will be only published once
  pubCommand = nh.advertise<std_msgs::String>("/udp/command", 1, true);
  ros::Rate loop_rate(10);//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //bool done=false;
  while(ros::ok())
  {
    ros::spinOnce();
    checkConnection();
    recvr_response(nh);
   }
  logFileStream.flush();
  logFileStream.close();
  ros::shutdown();
  return 0;
}
