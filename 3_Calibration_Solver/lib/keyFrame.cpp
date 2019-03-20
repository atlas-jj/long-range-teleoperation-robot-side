#include "keyFrame.h"

string keyFrame::pose2str()
{
  std::stringstream ss;
  ss<<mnFrameId<<",";
  ss<<std::setprecision(15)<<timeStamp<<",";
  for(int ti=0;ti<TWC.rows;ti++)
  {
    for(int tj=0;tj<TWC.cols;tj++)
    {
      std::ostringstream ssss;
      ssss << TWC.at<float>(ti,tj);
      ss<<ssss.str()<<",";
    }
  }
  return ss.str();
}

keyFrame::keyFrame(int mnid, double timesecs, vector<string> twcStr)
{
  cv::Mat currentTWC(4,4,cv::DataType<double>::type);
  int count=0;
  for(int ti=0;ti<TWC.rows;ti++)
  {
    for(int tj=0;tj<TWC.cols;tj++)
    {
      double tempt=atof(twcStr[count].c_str());
      currentTWC.at<double>(ti,tj)=tempt;
      count ++;
    }
  }
  keyFrame(mnid, timesecs, currentTWC);
}

keyFrame::keyFrame(int mnid, double timesecs, cv::Mat _TWC)
{
  mnFrameId = mnid;
  timeStamp = timesecs;
  TWC=_TWC;
  //convert TWC to position and rotation matrix
  rotation = TWC.rowRange(0,3).colRange(0,3);
  rotation = rotation.t();
  translation = TWC.rowRange(0,3).col(3);
  // cout<<"rotation : "<<endl<<rotation<<endl;
  // cout<<"translation:"<<endl<<translation<<endl;

  setQuaternion();
  // cout<<"Quaternion: x:"<<kq[0]<<" Y:"<<kq[1]<<" Z:"<<kq[2]<<" W:"<<kq[3]<<endl;
}

keyFrame::keyFrame()
{
  mnFrameId=-1;//initialize value
}

keyFrame::keyFrame(string str, int cam_robot_mark)
{
  vector<string> sta4=string_convertor::split(str,',');
  if(cam_robot_mark==0)//camera frame
  {
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
      keyFrame(mnid, tsecs,currentTWC);
    }
    else
      mnFrameId = -1;
  }
  else  //robot frame
  {
    if(sta4.size()==9)
    {
      int mnid = std::atoi(sta4[0].c_str());
      double tsecs = atof(sta4[1].c_str());
      keyFrame(mnid, tsecs,atof(sta4[2].c_str()), atof(sta4[3].c_str()), atof(sta4[4].c_str()), atof(sta4[5].c_str()),
                atof(sta4[6].c_str()), atof(sta4[7].c_str()), atof(sta4[8].c_str()));
    }
    else
      mnFrameId = -1;
  }
}

keyFrame::keyFrame(int mnid, double timesecs,double x, double y, double z, double qx, double qy, double qz, double qw)
{
  mnFrameId = mnid;
  timeStamp = timesecs;
  cv::Mat twc(4,4,cv::DataType<double>::type);
  twc.at<double>(3,0) = 0;twc.at<double>(3,1) = 0;twc.at<double>(3,2) = 0;twc.at<double>(3,3) = 1;
  cv::Mat t(3,1,cv::DataType<double>::type);
  cv::Mat r(3,3,cv::DataType<double>::type);

  t.at<double>(0,0) = x; t.at<double>(1,0) = y; t.at<double>(2,0) = z;
  t.copyTo(twc.rowRange(0,3).col(3));

  tf::Quaternion q(qx, qy , qz, qw);
  tf::Matrix3x3 rMatrix(q);

  double l = rMatrix[0][0];
  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++)
      {
        twc.at<double>(i,j)= rMatrix[i][j];
        r.at<double>(i,j)= rMatrix[i][j];
      }

  //cout<<"rotation: "<<endl<<r<<endl;

  translation = t;
  rotation =r.t();

  setQuaternion();

  TWC = twc;

  //cout<<"transformed twc: "<<endl<<TWC<<endl;

}

keyFrame::~keyFrame()
{

}


double keyFrame::getBaseLine(cv::Mat t1, cv::Mat t2)
{
  cv::Mat b(3,1,cv::DataType<double>::type);
  for(int i=0;i<3;i++)
    b.at<double>(i,0) = t1.at<double>(i,0) - t2.at<double>(i,0);
  return cv::norm(b);
}

void keyFrame::setQuaternion()
{
  tf::Matrix3x3 M(rotation.at<double>(0,0),rotation.at<double>(0,1),rotation.at<double>(0,2),
  rotation.at<double>(1,0),rotation.at<double>(1,1),rotation.at<double>(1,2),
  rotation.at<double>(2,0),rotation.at<double>(2,1),rotation.at<double>(2,2));
  M.getRotation(kq);
}
