
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
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "colormod.h" // namespace Color
#include "keyboard.h"
#ifndef Included_MATH_HELPER_H
#define Included_MATH_HELPER_H
#include "math_helper.h"
#endif
#ifndef Included_STRING_CONVERTOR_H
#define Included_STRING_CONVERTOR_H
#include "string_convertor.h"
#endif

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

// Robot state publishing
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayRobotState.h>
// Kinematics
#include <moveit_msgs/GetPositionIK.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
//wam
#include "wam_msgs/MatrixMN.h"
#include "wam_srvs/JointMove.h"
#include "sensor_msgs/JointState.h"

#include "pose_helper.h"
using namespace std;
bool commandLock = false;
Color::Modifier c_red(Color::FG_RED);
Color::Modifier c_yellow(Color::FG_YELLOW);
Color::Modifier c_green(Color::FG_GREEN);
Color::Modifier c_default(Color::FG_DEFAULT);
int dofNum = 7;
bool lock = false;
bool ready_signal1 = false;
bool ready_signal2 = false;
bool ready_signal3 = false;
double step_size = 0.02;//m
double current_pos[]={9999, 9999, 9999, 9999, 9999, 9999, 9999};//initialize as an invalid  value set, stores the current pose
std::vector<double> joint_values; //IK solutions
moveit::core::RobotModelPtr kinematic_model;
wam_srvs::JointMove mv_srv;
ros::ServiceClient Joint_move_client ;
geometry_msgs::Pose thisPose;
ros::Publisher pubCommand;

cv::Mat readTransformation()
{
  std::ifstream transform_file;
  std::string transform_file_path = "/home/chris/catkin_ws/src/long_range_teleoperation/robot-side/3_Calibration_Solver/result.txt";
  transform_file.open(transform_file_path.c_str(), std::ios_base::in | std::ios_base::binary);

  if(!transform_file) {
      std::cerr << "Can't open transform file" << std::endl;
      std::exit(-1);
  }
  cv::Mat transformation(4,4,cv::DataType<double>::type);

  std::string line;
  int i = 0;
  while(getline(transform_file, line) && i < 4) {
      std::istringstream in(line);
      double c1, c2, c3, c4;
      in >> c1 >> c2 >> c3 >> c4;
      transformation.at<double>(i,0) = c1;
      transformation.at<double>(i,1) = c2;
      transformation.at<double>(i,2) = c3;
      transformation.at<double>(i,3) = c4;
      ++i;
  }

  std::cout <<"calibration solver result: "<<endl<< transformation <<std::endl;
  return transformation;
}

std::vector<double> targetTransformation(double x, double y, double z)
{
  cv::Mat transformationMatrix = readTransformation();
  cv::Mat t(4,1,cv::DataType<double>::type);
  t.at<double>(0,0) = x;
  t.at<double>(1,0) = y;
  t.at<double>(2,0) = z;
  t.at<double>(3,0) = 1;
  cv::Mat result = transformationMatrix * t;
  std::vector<double> rv;
  rv.push_back(result.at<double>(0,0)/result.at<double>(3,0));
  rv.push_back(result.at<double>(1,0)/result.at<double>(3,0));
  rv.push_back(result.at<double>(2,0)/result.at<double>(3,0));
  return rv;
}

void wamPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
 {
   //cout<<"wam msgs"<<endl;
    thisPose = msg->pose;
    ready_signal1 = true;
 }

void wamJointsCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
   //cout<<"joint pose obtained:"<<endl;//<<initial_Joint_pose[1]<<endl;
    //cout<<initial_Joint_pose[0]<<"  "<<initial_Joint_pose[1]<<"  "<<initial_Joint_pose[2]<<" "<<initial_Joint_pose[3]<<" "<< initial_Joint_pose[4]
   // <<" "<<initial_Joint_pose[5]<<" "<<initial_Joint_pose[6]<<endl;
   current_pos[0]=msg->position[0];
   current_pos[1]=msg->position[1];
   current_pos[2]=msg->position[2];
   current_pos[3]=msg->position[3];
   current_pos[4]=msg->position[4];
   current_pos[5]=msg->position[5];
   current_pos[6]=msg->position[6];
   ready_signal2 = true;
}

void moveRobotJointValue(std::vector<double> joint_values_t)
{
  lock=true;
  std::vector<float> jnts;
     for(size_t i=0;i<dofNum;i++)
     {
       float newJ = (float)(joint_values_t[i]);
       jnts.push_back(newJ);
     }
  mv_srv.request.joints = jnts;
  cout<<"send to robot to the  position "<<endl;
  string_convertor::printOutStdVector_f(jnts);
  cout << "Press any key to move..." << endl;
  getchar();
  Joint_move_client.call(mv_srv);///////////////////////////////////////////////////
  //boost::this_thread::sleep( boost::posix_time::milliseconds(1000) );
  lock=false;
}

geometry_msgs::Pose comput_fk(double currentJoints[]) //get current end  effector  pose
{
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  //fill in the seed
  kinematic_state->setVariablePositions(currentJoints);
  cout<<"compute fk current joints "<<currentJoints[0]<<", "<<currentJoints[1]<<", "<<currentJoints[2]<<endl;
  kinematic_state->update();
  const Eigen::Affine3d end_effector_state = kinematic_state->getGlobalLinkTransform("wam/wrist_palm_link");//table_base
  ROS_INFO_STREAM("FK Translation: " << end_effector_state.translation());
  Eigen::Quaterniond rot_q(end_effector_state.rotation());
  ROS_INFO_STREAM("FK Rotation q x: " << rot_q.x());
  ROS_INFO_STREAM("FK Rotation q y: " << rot_q.y());
  ROS_INFO_STREAM("FK Rotation q z: " << rot_q.z());
  ROS_INFO_STREAM("FK Rotation q w: " << rot_q.w());
  geometry_msgs::Pose p;
  p.position.x = end_effector_state.translation()(0);
  p.position.y = end_effector_state.translation()(1);
  p.position.z = end_effector_state.translation()(2);
  p.orientation.x = rot_q.x();
  p.orientation.y = rot_q.y();
  p.orientation.z = rot_q.z();
  p.orientation.w = rot_q.w();
  return p;
}

bool comput_IK(geometry_msgs::Pose p, double referenceJoints[])
{
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm");//All, it's in the planning group name
  // const std::vector<std::string> &joint_names2 = joint_model_group->getJointModelNames();
  // joint_names2=string_convertor::safeRemoveStringVec(joint_names2,0);//first  item of  joint name  is  caused by  mistake
  // cout<<"joint names count  "<<joint_names2.size()<<endl;//8
  //fill in the seed
  kinematic_state->setVariablePositions(referenceJoints);
  cout<<"compute IK current joints "<<referenceJoints[0]<<", "<<referenceJoints[1]<<", "<<referenceJoints[2]<<endl;
  kinematic_state->update();
  bool found_ik = kinematic_state->setFromIK(joint_model_group, p, 10, 0.1);
  // Now, we can print out the IK solution (if found):
  if (found_ik)
  {
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    cout<<c_green<<"IK solution found: ";
    for(std::size_t i=0; i < joint_values.size(); ++i)
    {
      ROS_INFO("IK solution Joint %s: %f", to_string(i+1).c_str(), joint_values[i]);
      cout<<joint_values[i]<<",";
    }
    cout<<c_default<<endl;
  }
  else
  {
    ROS_INFO("Did not find IK solution");
  }
  return found_ik;
}

void test_IK_FK()
{
  geometry_msgs::Pose pose = comput_fk(current_pos);
  cout<<"using pose to test "<<endl;
  cout<<"x "<<pose.position.x <<" y "<<pose.position.y<<" z "<<pose.position.z<<endl;
  bool ik_found = comput_IK(pose, current_pos);
  if(ik_found)
  {
    ROS_INFO("asdaf");
  }
}

void printPose(geometry_msgs::Pose pose)
{
  cout<<"x "<<pose.position.x <<" y "<<pose.position.y<<" z "<<pose.position.z<<endl;
  cout<<"qx: "<<pose.orientation.x <<" qy: "<<pose.orientation.y<<" qz: "<<pose.orientation.z<<" qw: "<< pose.orientation.w<<endl;
}

void execute2target(geometry_msgs::Pose targetRobotPose)
{
  cout<<c_red<<"New target: "<<endl;
  printPose(targetRobotPose);
  cout<<c_red<<"press key to continue"<<c_default<<endl;
  ready_signal1=false;ready_signal2=false;
  while(!ready_signal1&&!ready_signal2)
  {
    ros::spinOnce();
    boost::this_thread::sleep( boost::posix_time::milliseconds(100) );
    cout<<"wait for ready_signal1 and ready_signal2"<<endl;
  }
  cout<<"robot joints and poses values updated!"<<endl;
  geometry_msgs::Pose initial_robot_pose = thisPose;
  geometry_msgs::Pose initial_FK_pose = comput_fk(current_pos);

  cout<<c_green<<" corr FK target is: "<<endl;
  printPose(pose_helper::convert2FKtarget(initial_FK_pose, initial_robot_pose, targetRobotPose));
  cout<<c_default;

  cout<<"initial robot pose"<<endl;
  printPose(initial_robot_pose);
  cout<<"initial FK pose"<<endl;
  printPose(initial_FK_pose);
  int steps_num = pose_helper::get_steps_num(initial_robot_pose, targetRobotPose, step_size);
  cout<<"steps Number for this target: "<<steps_num<<endl;

  getchar();
  for(int i=0; i< steps_num; i++)
  {
    geometry_msgs::Pose step_pose = pose_helper::next_step_pose(initial_FK_pose, initial_robot_pose, targetRobotPose, i+1, steps_num);
    cout<<c_red;
    cout<<"step "<<(i+1)<<" target, out of "<<steps_num<<endl;
    printPose(step_pose);
    cout<<c_default;
    //this step_pose is represented in IK sovler's frame now.
    bool ik_found = comput_IK(step_pose, current_pos);
    if(ik_found)
    {
      moveRobotJointValue(joint_values);
      cout<<c_yellow;
      ROS_INFO("robot moved ");
      cout<<c_default;
      //update joint values
      current_pos[0]=joint_values[0];
      current_pos[1]=joint_values[1];
      current_pos[2]=joint_values[2];
      current_pos[3]=joint_values[3];
      current_pos[4]=joint_values[4];
      current_pos[5]=joint_values[5];
      current_pos[6]=joint_values[6];
      // ready_signal2 = false;
      // while(!ready_signal2)
      // {
      //   ros::spinOnce();
      //   boost::this_thread::sleep( boost::posix_time::milliseconds(100) );
      //   cout<<"wait for ready_signal2"<<endl;
      // }
    }
    else
      i = i -1;
    cout<<"loop index : "<<i<<endl;
  }
  cout<<"haha, task accomplished!"<<endl;
}


void udpCommandCallback(const std_msgs::String::ConstPtr& msg)
{
  if(commandLock)
  {
    cout<<c_red<<"command locked!"<<endl<<c_default;
    return;
  }
  //send the command to remote robot
  cout<<"recved: "<<msg->data<<endl;//<<msg->data<<endl;
  if(msg->data.find("coarse_target_move") != std::string::npos && !commandLock) //fine our command
  {
    //execute command
    commandLock = true;
    std::vector<string> straa = string_convertor::split(msg->data, ':');
    string datastr = straa[2];
    std::vector<double> dataValues = string_convertor::split2double(datastr, ',');
    std::vector<double> transformedResult = targetTransformation(dataValues[0], dataValues[1], dataValues[2]);
    geometry_msgs::Pose target_robot_pose = pose_helper::construct_geometry_pose(transformedResult[0], transformedResult[1], transformedResult[2], (int)(dataValues[3]));
    cout<<c_green<<"go to execution phase"<<endl<<c_default;
    execute2target(target_robot_pose);
    // std_msgs::String msg;
    // msg.data = "";
    // pubCommand.publish(msg); //clear the command buffer.
    commandLock = false;
  }
  if(msg->data.find("way_point_move") != std::string::npos && !commandLock) //fine our command
  {
    //execute command
    //way_point_move:1:x,y,z,qx,qy,qz,qw,.........
    commandLock = true;
    std::vector<string> straa = string_convertor::split(msg->data, ':');
    string datastr = straa[2];
    std::vector<double> dataValues = string_convertor::split2double(datastr, ',');
    geometry_msgs::Pose target_robot_pose;
    target_robot_pose.position.x = dataValues[0]
    target_robot_pose.position.y = dataValues[1]
    target_robot_pose.position.z = dataValues[2]
    target_robot_pose.orientation.x = dataValues[3]
    target_robot_pose.orientation.y = dataValues[4]
    target_robot_pose.orientation.z = dataValues[5]
    target_robot_pose.orientation.w = dataValues[6]

    cout<<c_green<<"go to execution phase"<<endl<<c_default;
    execute2target(target_robot_pose);
    // std_msgs::String msg;
    // msg.data = "";
    // pubCommand.publish(msg); //clear the command buffer.
    commandLock = false;
  }
}
//===========================MAIN FUNCTION START===========================

int main(int argc, char* argv[]){
  // Initialize the ROS system and become a node.
   // if(argc==2)
   //      numThres = atoi(argv[1]);
  ros::init(argc, argv, "tele_op");
  ros::NodeHandle n("~");
  ros::Subscriber subP = n.subscribe("/zeus/wam/pose", 1, wamPoseCallback);
  ros::Subscriber wam_joints_sub=n.subscribe("/zeus/wam/joint_states",1, wamJointsCallback);
  ros::Subscriber udp_tasks=n.subscribe("/udp/command",1, udpCommandCallback);
  //pubCommand = n.advertise<std_msgs::String>("/udp/command", 1, true);
  Joint_move_client = n.serviceClient<wam_srvs::JointMove>("/zeus/wam/joint_move");

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
  commandLock = true;
  test_IK_FK();
  //std::vector<double> testrestulst = targetTransformation(-0.0632346, 0.0754276, 0.728763);
  //cout<<"target transofrmation: "<<testrestulst[0]<<"  "<<testrestulst[1]<<"  "<<testrestulst[2]<<endl;
  cout<<"press key to continue"<<endl;
  getchar();

    while(!ready_signal1)
    {
      ros::spinOnce();
      boost::this_thread::sleep( boost::posix_time::milliseconds(100) );
      cout<<"wait for readySignal 1 in main"<<endl;
    }
    commandLock = false;
    cout<<"readySignal1 Received! goto main loop. CommandLock set to false."<<endl;
  // int presetMark = 3; //press button -0.08, -0.69, 0.17, 3./// handler  0.4, 0, 0.4, 1
  // geometry_msgs::Pose target_robot_pose = pose_helper::construct_geometry_pose(0.4, 0, 0.4, 1);
  // execute2target(target_robot_pose);
  ros::spin();
  ros::waitForShutdown();
}
