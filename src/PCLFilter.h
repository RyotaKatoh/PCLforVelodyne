#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

class PCLFilter {
public:
    typedef pcl::PointCloud<pcl::PointXYZI> Cloud;
    typedef typename Cloud::ConstPtr CloudConstPtr;
    
    void passThroughFilter(const CloudConstPtr &cloud, Cloud::Ptr &cloud_filtered, std::string filter_target, float lower_bound, float upper_bound);
    
    void noiseRemovalFilter(const CloudConstPtr &cloud, Cloud::Ptr &cloud_filtered);
};
