#include "PCLFilter.h"

void PCLFilter::passThroughFilter(const CloudConstPtr &cloud, Cloud::Ptr &cloud_filtered, std::string filter_target, float lower_bound, float upper_bound){
  pcl::PassThrough<pcl::PointXYZI> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName(filter_target);
  pass.setFilterLimits(lower_bound, upper_bound);
  pass.filter(*cloud_filtered);
}
