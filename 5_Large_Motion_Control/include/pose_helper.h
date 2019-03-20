#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>

#include <vector>
using namespace std;
class pose_helper
{
    public:
       pose_helper();
       ~pose_helper();

       static std::vector<double> get_preset_orientation(int preset_mark);
       static int get_steps_num(geometry_msgs::Pose currentPose, geometry_msgs::Pose targetPose, double step_size);
       static geometry_msgs::Pose construct_geometry_pose(double x, double y, double z, int prest_mark);
       static geometry_msgs::Pose next_step_pose(geometry_msgs::Pose InitialFKpose,
                                           geometry_msgs::Pose InitialRobotPose,
                                           geometry_msgs::Pose targetRobotPose,
                                         int step_index, int step_num);
       static geometry_msgs::Pose convert2FKtarget(geometry_msgs::Pose InitialFKpose,
                                           geometry_msgs::Pose InitialRobotPose,
                                           geometry_msgs::Pose targetRobotPose);
};
