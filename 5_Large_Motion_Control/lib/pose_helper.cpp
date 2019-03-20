#include "pose_helper.h"

std::vector<double> pose_helper::get_preset_orientation(int preset_mark)
{
  std::vector<double> preset_orientations;
  switch(preset_mark){
    case 1: cout <<"get preset 1 orientation: From Top"<<endl;
            preset_orientations.push_back(0.0661); //qx
            preset_orientations.push_back(0.99533); //qy
            preset_orientations.push_back(-0.02945); //qz
            preset_orientations.push_back(0.06385); //qw
            break;
    case 2: cout <<"get preset 2 orientation: From Top"<<endl;
            preset_orientations.push_back(0.03193); //qx
            preset_orientations.push_back(-0.722004); //qy
            preset_orientations.push_back(0.010633); //qz
            preset_orientations.push_back(-0.69107); //qw
            break;
    case 3: cout <<"get preset 3 orientation: From Top"<<endl;
            preset_orientations.push_back(0.73485); //qx
            preset_orientations.push_back(0.538221); //qy
            preset_orientations.push_back(-0.319786); //qz
            preset_orientations.push_back(0.26086); //qw
            break;
    case 4: cout <<"get preset 4 orientation: From Top"<<endl;
            preset_orientations.push_back(0); //qx
            preset_orientations.push_back(0); //qy
            preset_orientations.push_back(0); //qz
            preset_orientations.push_back(0); //qw
            break;
    default:cout <<"get default orientation: From Top"<<endl;
            preset_orientations.push_back(0); //qx
            preset_orientations.push_back(0); //qy
            preset_orientations.push_back(0); //qz
            preset_orientations.push_back(0); //qw
            break;
  }
  return preset_orientations;
}


int pose_helper::get_steps_num(geometry_msgs::Pose currentPose, geometry_msgs::Pose targetPose, double step_size)
{
  double deltax = abs(targetPose.position.x - currentPose.position.x);
  double deltay = abs(targetPose.position.y - currentPose.position.y);
  double deltaz = abs(targetPose.position.z - currentPose.position.z);
  cout<<"dx "<<deltax<<" dy "<<deltay<<" dz "<<deltaz<<endl;
  double maxd = deltax;
  if(deltay>maxd)
    maxd = deltay;
  if(deltaz>maxd)
    maxd = deltaz;
  cout<<"maxd "<<maxd<<endl;
  cout<<"step size "<<step_size<<endl;
  return (int)(maxd/step_size) + 1;
}

geometry_msgs::Pose pose_helper::construct_geometry_pose(double x, double y, double z, int preset_mark)
{
  geometry_msgs::Pose rtPose;
  rtPose.position.x = x;
  rtPose.position.y = y;
  rtPose.position.z = z;
  std::vector<double> poseQ = get_preset_orientation(preset_mark);
  rtPose.orientation.x = poseQ[0];
  rtPose.orientation.y = poseQ[1];
  rtPose.orientation.z = poseQ[2];
  rtPose.orientation.w = poseQ[3];
  return rtPose;
}

geometry_msgs::Pose pose_helper::next_step_pose(geometry_msgs::Pose InitialFKpose,
                                    geometry_msgs::Pose InitialRobotPose,
                                    geometry_msgs::Pose targetRobotPose,
                                  int step_index, int step_num)
{
  //step_index starting from 1 to step_num
  double deltaX = targetRobotPose.position.x - InitialFKpose.position.x;
  double deltaY = targetRobotPose.position.y - InitialFKpose.position.y;
  double deltaZ = targetRobotPose.position.z - InitialRobotPose.position.z;
  tf::Quaternion q_current(InitialFKpose.orientation.x, InitialFKpose.orientation.y, InitialFKpose.orientation.z, InitialFKpose.orientation.w);
  tf::Quaternion q_target(targetRobotPose.orientation.x, targetRobotPose.orientation.y, targetRobotPose.orientation.z, targetRobotPose.orientation.w);
  double scalar_step = ((double)step_index / (double)step_num);
  cout<<"scalar step: " <<scalar_step<<endl;

  geometry_msgs::Pose next_pose;

    next_pose.position.x = InitialFKpose.position.x + deltaX * scalar_step;
    next_pose.position.y = InitialFKpose.position.y + deltaY * scalar_step;
    next_pose.position.z = InitialFKpose.position.z + deltaZ * scalar_step;

  tf::Quaternion q_nextStep = q_current.slerp(q_target, scalar_step);
  next_pose.orientation.x = q_nextStep.x();
  next_pose.orientation.y = q_nextStep.y();
  next_pose.orientation.z = q_nextStep.z();
  next_pose.orientation.w = q_nextStep.w();
  return next_pose;
}

geometry_msgs::Pose pose_helper::convert2FKtarget(geometry_msgs::Pose InitialFKpose,
                                    geometry_msgs::Pose InitialRobotPose,
                                    geometry_msgs::Pose targetRobotPose)
{
  //step_index starting from 1 to step_num
  double deltaX = targetRobotPose.position.x - InitialFKpose.position.x;
  double deltaY = targetRobotPose.position.y - InitialFKpose.position.y;
  double deltaZ = targetRobotPose.position.z - InitialRobotPose.position.z;
  geometry_msgs::Pose next_pose;

    next_pose.position.x = InitialFKpose.position.x + deltaX;
    next_pose.position.y = InitialFKpose.position.y + deltaY;
    next_pose.position.z = InitialFKpose.position.z + deltaZ;
  next_pose.orientation.x = targetRobotPose.orientation.x;
  next_pose.orientation.y = targetRobotPose.orientation.y;
  next_pose.orientation.z = targetRobotPose.orientation.z;
  next_pose.orientation.w = targetRobotPose.orientation.w;
  return next_pose;
}
