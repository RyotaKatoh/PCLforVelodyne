#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

class PCLFilter {
public:
  typedef pcl::PointCloud<pcl::PointXYZI> Cloud;
  typedef typename Cloud::ConstPtr CloudConstPtr;

  void passThroughFilter(const CloudConstPtr &cloud, Cloud::Ptr &cloud_filtered, std::string filter_target, float lower_bound, float upper_bound);

};
