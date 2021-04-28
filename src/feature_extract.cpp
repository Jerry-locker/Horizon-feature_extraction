#include <cmath>
#include "feature_extract.h"

using namespace std;

#define M_PI 3.14159265358979
#define IS_VALID(a)  ((abs(a)>1e8) ? true : false)

const int N_SCANS = 6; //假设当前用的是mid
const double blind = 0.5, inf_bound = 10;
const int group_size = 8;
const double disA = 0.01, disB = 0.1;
const double limit_maxmid = 9, limit_midmin = 16, limit_maxmin = 3.24;
const double p2l_ratio = 400;
double jump_up_limit = cos(175.0/180*M_PI), jump_down_limit = cos(5.0/180*M_PI);
double cos160 = cos(160.0/180*M_PI);
const double edgea = 3, edgeb = 0.05;
double smallp_intersect = cos(170.0/180*M_PI), smallp_ratio = 1.2;
const int point_filter_num = 4;
double vx,vy,vz;

FeatureExtraction::FeatureExtraction():nh("~")
{
  sub_points = nh.subscribe("/livox/lidar", 1000, &FeatureExtraction::horizonHandler, this);
  pub_full = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud", 100);         //发布整个点云
  pub_surf = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 100);    //发布面点
  pub_corn = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100);   //发布角点
}

void FeatureExtraction::horizonHandler(const livox_ros_driver::CustomMsg::ConstPtr &msg)
{
  std::vector<myCloud> pl_buff(N_SCANS);
  std::vector<std::vector<orgtype>> typess(N_SCANS);
  myCloud pl_full, pl_corn, pl_surf;

  uint plsize = msg->point_num;

  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);
  pl_full.resize(plsize);

  for(int i=0; i<N_SCANS; i++)
  {
    pl_buff[i].reserve(plsize);
  }

  for(uint i=1; i<plsize; i++)
  {
    if((msg->points[i].line < N_SCANS) && ((msg->points[i].tag & 0x30) == 0x10)
        && (!IS_VALID(msg->points[i].x)) && (!IS_VALID(msg->points[i].y)) && (!IS_VALID(msg->points[i].z)))
    {
      pl_full[i].x = msg->points[i].x;
      pl_full[i].y = msg->points[i].y;
      pl_full[i].z = msg->points[i].z;
      pl_full[i].intensity = msg->points[i].reflectivity;
      pl_full[i].curvature = msg->points[i].offset_time / float(1000000); //use curvature as time of each laser points ？？？

      if((std::abs(pl_full[i].x - pl_full[i-1].x) > 1e-7)
          || (std::abs(pl_full[i].y - pl_full[i-1].y) > 1e-7)
          || (std::abs(pl_full[i].z - pl_full[i-1].z) > 1e-7))
      {
        pl_buff[msg->points[i].line].push_back(pl_full[i]);
      }
    }
  }

  if(pl_buff[0].size() <= 7) {return;}

  for(int j=0; j<N_SCANS; j++)
  {
    myCloud &pl = pl_buff[j];
    vector<orgtype> &types = typess[j];
    plsize = pl.size();
    types.resize(plsize);
    plsize--;
    for(uint i=0; i<plsize; i++)
    {
      types[i].range = sqrt(pl[i].x*pl[i].x + pl[i].y*pl[i].y);
      vx = pl[i].x - pl[i+1].x;
      vy = pl[i].y - pl[i+1].y;
      vz = pl[i].z - pl[i+1].z;
      types[i].dista = vx*vx + vy*vy + vz*vz;
      // std::cout<<vx<<" "<<vx<<" "<<vz<<" "<<std::endl;
    }
    // plsize++;
    types[plsize].range = sqrt(pl[plsize].x*pl[plsize].x + pl[plsize].y*pl[plsize].y);

    giveFeature(pl, types, pl_corn, pl_surf);
  }

  ros::Time ct;
  ct.fromNSec(msg->timebase);
  pubCloud(pl_full, pub_full, msg->header.stamp);
  pubCloud(pl_surf, pub_surf, msg->header.stamp);
  pubCloud(pl_corn, pub_corn, msg->header.stamp);
}

void FeatureExtraction::giveFeature(myCloud &pl, vector<orgtype> &types, myCloud &pl_corn, myCloud &pl_surf)
{
  uint plsize = pl.size();
  uint plsize2;
  if(plsize == 0)
  {
    printf("something wrong\n");
    return;
  }
  uint head = 0;

  while(types[head].range < blind)
  {
    head++;
  }

  // Surf
  plsize2 = (plsize > group_size) ? (plsize - group_size) : 0;

  Eigen::Vector3d curr_direct(Eigen::Vector3d::Zero());
  Eigen::Vector3d last_direct(Eigen::Vector3d::Zero());

  uint i_nex = 0, i2;
  uint last_i = 0; uint last_i_nex = 0;
  int last_state = 0;
  int plane_type;

  for(uint i=head; i<plsize2; i++)
  {
    if(types[i].range < blind)
    {
      continue;
    }
    // i_nex = i;
    i2 = i;
    // std::cout<<" i: "<<i<<" i_nex "<<i_nex<<"group_size: "<<group_size<<" plsize "<<plsize<<" plsize2 "<<plsize2<<std::endl;
    plane_type = checkPlane(pl, types, i, i_nex, curr_direct);

    if(plane_type == 1)
    {
      for(uint j=i; j<=i_nex; j++)
      {
        if(j!=i && j!=i_nex)
        {
          types[j].ftype = Real_Plane;
        }
        else
        {
          types[j].ftype = Poss_Plane;
        }
      }

      // if(last_state==1 && fabs(last_direct.sum())>0.5)
      if(last_state==1 && last_direct.norm()>0.1)
      {
        double mod = last_direct.transpose() * curr_direct;
        if(mod>-0.707 && mod<0.707)
        {
          types[i].ftype = Edge_Plane;
        }
        else
        {
          types[i].ftype = Real_Plane;
        }
      }

      i = i_nex - 1;
      last_state = 1;
    }
    else if(plane_type == 2)
    {
      i = i_nex;
      last_state = 0;
    }
    else if(plane_type == 0)
    {
      if(last_state == 1)
      {
        uint i_nex_tem;
        uint j;
        for(j=last_i+1; j<=last_i_nex; j++)
        {
          uint i_nex_tem2 = i_nex_tem;
          Eigen::Vector3d curr_direct2;

          uint ttem = checkPlane(pl, types, j, i_nex_tem, curr_direct2);

          if(ttem != 1)
          {
            i_nex_tem = i_nex_tem2;
            break;
          }
          curr_direct = curr_direct2;
        }

        if(j == last_i+1)
        {
          last_state = 0;
        }
        else
        {
          for(uint k=last_i_nex; k<=i_nex_tem; k++)
          {
            if(k != i_nex_tem)
            {
              types[k].ftype = Real_Plane;
            }
            else
            {
              types[k].ftype = Poss_Plane;
            }
          }
          i = i_nex_tem-1;
          i_nex = i_nex_tem;
          i2 = j-1;
          last_state = 1;
        }

      }
    }

    last_i = i2;
    last_i_nex = i_nex;
    last_direct = curr_direct;

  }

  plsize2 = plsize > 3 ? plsize - 3 : 0;
  for(uint i=head+3; i<plsize2; i++)
  {
    if(types[i].range<blind || types[i].ftype>=Real_Plane)
    {
      continue;
    }

    if(types[i-1].dista<1e-16 || types[i].dista<1e-16)
    {
      continue;
    }

    Eigen::Vector3d vec_a(pl[i].x, pl[i].y, pl[i].z);
    Eigen::Vector3d vecs[2];

    for(int j=0; j<2; j++)
    {
      int m = -1;
      if(j == 1)
      {
        m = 1;
      }

      if(types[i+m].range < blind)
      {
        if(types[i].range > inf_bound)
        {
          types[i].edj[j] = Nr_inf;
        }
        else
        {
          types[i].edj[j] = Nr_blind;
        }
        continue;
      }

      vecs[j] = Eigen::Vector3d(pl[i+m].x, pl[i+m].y, pl[i+m].z);
      vecs[j] = vecs[j] - vec_a;

      types[i].angle[j] = vec_a.dot(vecs[j]) / vec_a.norm() / vecs[j].norm();
      if(types[i].angle[j] < jump_up_limit)
      {
        types[i].edj[j] = Nr_180;
      }
      else if(types[i].angle[j] > jump_down_limit)
      {
        types[i].edj[j] = Nr_zero;
      }
    }

    types[i].intersect = vecs[Prev].dot(vecs[Next]) / vecs[Prev].norm() / vecs[Next].norm();
    if(types[i].edj[Prev]==Nr_nor && types[i].edj[Next]==Nr_zero && types[i].dista>0.0225 && types[i].dista>4*types[i-1].dista)
    {
      if(types[i].intersect > cos160)
      {
        if(checkCorner(pl, types, i, Prev))
        {
          types[i].ftype = Edge_Jump;
        }
      }
    }
    else if(types[i].edj[Prev]==Nr_zero && types[i].edj[Next]== Nr_nor && types[i-1].dista>0.0225 && types[i-1].dista>4*types[i].dista)
    {
      if(types[i].intersect > cos160)
      {
        if(checkCorner(pl, types, i, Next))
        {
          types[i].ftype = Edge_Jump;
        }
      }
    }
    else if(types[i].edj[Prev]==Nr_nor && types[i].edj[Next]==Nr_inf)
    {
      if(checkCorner(pl, types, i, Prev))
      {
        types[i].ftype = Edge_Jump;
      }
    }
    else if(types[i].edj[Prev]==Nr_inf && types[i].edj[Next]==Nr_nor)
    {
      if(checkCorner(pl, types, i, Next))
      {
        types[i].ftype = Edge_Jump;
      }

    }
    else if(types[i].edj[Prev]>Nr_nor && types[i].edj[Next]>Nr_nor)
    {
      if(types[i].ftype == Nor)
      {
        types[i].ftype = Wire;
      }
    }
  }

  plsize2 = plsize-1;
  double ratio;
  for(uint i=head+1; i<plsize2; i++)
  {
    if(types[i].range<blind || types[i-1].range<blind || types[i+1].range<blind)
    {
      continue;
    }

    if(types[i-1].dista<1e-8 || types[i].dista<1e-8)
    {
      continue;
    }

    if(types[i].ftype == Nor)
    {
      if(types[i-1].dista > types[i].dista)
      {
        ratio = types[i-1].dista / types[i].dista;
      }
      else
      {
        ratio = types[i].dista / types[i-1].dista;
      }

      if(types[i].intersect<smallp_intersect && ratio < smallp_ratio)
      {
        if(types[i-1].ftype == Nor)
        {
          types[i-1].ftype = Real_Plane;
        }
        if(types[i+1].ftype == Nor)
        {
          types[i+1].ftype = Real_Plane;
        }
        types[i].ftype = Real_Plane;
      }
    }
  }

  int last_surface = -1;
  for(uint j=head; j<plsize; j++)
  {
    if(types[j].ftype==Poss_Plane || types[j].ftype==Real_Plane)
    {
      if(last_surface == -1)
      {
        last_surface = j;
      }

      if(j == uint(last_surface+point_filter_num-1))
      {
        PointType ap;
        ap.x = pl[j].x;
        ap.y = pl[j].y;
        ap.z = pl[j].z;
        ap.curvature = pl[j].curvature;
        pl_surf.push_back(ap);

        last_surface = -1;
      }
    }
    else
    {
      if(types[j].ftype==Edge_Jump || types[j].ftype==Edge_Plane)
      {
        pl_corn.push_back(pl[j]);
      }
      if(last_surface != -1)
      {
        PointType ap;
        for(uint k=last_surface; k<j; k++)
        {
          ap.x += pl[k].x;
          ap.y += pl[k].y;
          ap.z += pl[k].z;
          ap.curvature += pl[k].curvature;
        }
        ap.x /= (j-last_surface);
        ap.y /= (j-last_surface);
        ap.z /= (j-last_surface);
        ap.curvature /= (j-last_surface);
        pl_surf.push_back(ap);
      }
      last_surface = -1;
    }
  }
}

void FeatureExtraction::pubCloud(myCloud &pl, ros::Publisher pub, const ros::Time &ct)
{
  pl.height = 1; pl.width = pl.size();
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(pl, output);
  output.header.frame_id = "livox";
  output.header.stamp = ct;
  pub.publish(output);
}

int FeatureExtraction::checkPlane(const myCloud &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct)
{
  double group_dis = disA*types[i_cur].range + disB;
  group_dis = group_dis * group_dis;
  // i_nex = i_cur;

  double two_dis;
  vector<double> disarr;
  disarr.reserve(20);

  for(i_nex=i_cur; i_nex<i_cur+group_size; i_nex++)
  {
    if(types[i_nex].range < blind)
    {
      curr_direct.setZero();
      return 2;
    }
    disarr.push_back(types[i_nex].dista);
  }

  for(;;)
  {
    if((i_cur >= pl.size()) || (i_nex >= pl.size())) break;

    if(types[i_nex].range < blind)
    {
      curr_direct.setZero();
      return 2;
    }
    vx = pl[i_nex].x - pl[i_cur].x;
    vy = pl[i_nex].y - pl[i_cur].y;
    vz = pl[i_nex].z - pl[i_cur].z;
    two_dis = vx*vx + vy*vy + vz*vz;
    if(two_dis >= group_dis)
    {
      break;
    }
    disarr.push_back(types[i_nex].dista);
    i_nex++;
  }

  double leng_wid = 0;
  double v1[3], v2[3];
  for(uint j=i_cur+1; j<i_nex; j++)
  {
    if((j >= pl.size()) || (i_cur >= pl.size())) break;
    v1[0] = pl[j].x - pl[i_cur].x;
    v1[1] = pl[j].y - pl[i_cur].y;
    v1[2] = pl[j].z - pl[i_cur].z;

    v2[0] = v1[1]*vz - vy*v1[2];
    v2[1] = v1[2]*vx - v1[0]*vz;
    v2[2] = v1[0]*vy - vx*v1[1];

    double lw = v2[0]*v2[0] + v2[1]*v2[1] + v2[2]*v2[2];
    if(lw > leng_wid)
    {
      leng_wid = lw;
    }
  }


  if((two_dis*two_dis/leng_wid) < p2l_ratio)
  {
    curr_direct.setZero();
    return 0;
  }

  uint disarrsize = disarr.size();
  for(uint j=0; j<disarrsize-1; j++)
  {
    for(uint k=j+1; k<disarrsize; k++)
    {
      if(disarr[j] < disarr[k])
      {
        leng_wid = disarr[j];
        disarr[j] = disarr[k];
        disarr[k] = leng_wid;
      }
    }
  }

  if(disarr[disarr.size()-2] < 1e-16)
  {
    curr_direct.setZero();
    return 0;
  }

//  if(lidar_type==MID || lidar_type==HORIZON)
//  {
    double dismax_mid = disarr[0]/disarr[disarrsize/2];
    double dismid_min = disarr[disarrsize/2]/disarr[disarrsize-2];

    if(dismax_mid>=limit_maxmid || dismid_min>=limit_midmin)
    {
      curr_direct.setZero();
      return 0;
    }
//  }
//  else
//  {
//    double dismax_min = disarr[0] / disarr[disarrsize-2];
//    if(dismax_min >= limit_maxmin)
//    {
//      curr_direct.setZero();
//      return 0;
//    }
//  }

  curr_direct << vx, vy, vz;
  curr_direct.normalize();
  return 1;
}

bool FeatureExtraction::checkCorner(const myCloud &pl, vector<orgtype> &types, uint i, Surround nor_dir)
{
  if(nor_dir == 0)
  {
    if(types[i-1].range<blind || types[i-2].range<blind)
    {
      return false;
    }
  }
  else if(nor_dir == 1)
  {
    if(types[i+1].range<blind || types[i+2].range<blind)
    {
      return false;
    }
  }
  double d1 = types[i+nor_dir-1].dista;
  double d2 = types[i+3*nor_dir-2].dista;
  double d;

  if(d1<d2)
  {
    d = d1;
    d1 = d2;
    d2 = d;
  }

  d1 = sqrt(d1);
  d2 = sqrt(d2);


  if(d1>edgea*d2 || (d1-d2)>edgeb)
  {
    return false;
  }

  return true;
}



