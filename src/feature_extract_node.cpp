#include "feature_extract.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "feature_extract");
  FeatureExtraction f_extract;
  ros::spin();
  return 0;
}
