#pragma once

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver/CustomMsg.h>
#include <vector>

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> myCloud;

enum Feature{Nor, Poss_Plane, Real_Plane, Edge_Jump, Edge_Plane, Wire, ZeroPoint};
enum Surround{Prev, Next};
enum E_jump{Nr_nor, Nr_zero, Nr_180, Nr_inf, Nr_blind};
struct orgtype
{
  double range;
  double dista;
  double angle[2];
  double intersect;
  E_jump edj[2];
  Feature ftype;
  orgtype()
  {
    range = 0;
    edj[Prev] = Nr_nor;
    edj[Next] = Nr_nor;
    ftype = Nor;
    intersect = 2;
  }
};

class FeatureExtraction
{
public:
  FeatureExtraction();
  void horizonHandler(const livox_ros_driver::CustomMsg::ConstPtr &msg);
  void giveFeature(myCloud &pl, std::vector<orgtype> &types, myCloud &pl_corn, myCloud &pl_surf);
  void pubCloud(myCloud &pl, ros::Publisher pub, const ros::Time &ct);
  int checkPlane(const myCloud &pl, std::vector<orgtype> &types, uint i, uint &i_nex, Eigen::Vector3d &curr_direct);
  bool checkCorner(const myCloud &pl, std::vector<orgtype> &types, uint i, Surround nor_dir);

private:
  ros::NodeHandle nh;
  ros::Subscriber sub_points;
  ros::Publisher pub_full, pub_surf, pub_corn;
};
