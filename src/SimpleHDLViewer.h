#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/hdl_grabber.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>

#include "PCLFilter.h"
#include "PCLClustering.h"

using namespace std;
using namespace pcl;
using namespace pcl::console;
using namespace pcl::visualization;

class SimpleHDLViewer
{
  public:
    typedef pcl::PointCloud<pcl::PointXYZI> Cloud;
    typedef typename Cloud::ConstPtr CloudConstPtr;

    SimpleHDLViewer (Grabber& grabber,
        pcl::visualization::PointCloudColorHandler<pcl::PointXYZI> &handler) :
        cloud_viewer_ (new pcl::visualization::PCLVisualizer ("PCL HDL Cloud")),
        grabber_ (grabber),
        handler_ (handler)
    {
    }
    void cloud_callback(const CloudConstPtr &cloud);
    void run();

    void passThroughFilter(const CloudConstPtr &cloud, Cloud::Ptr &cloud_filtered, string target, float lower_bound, float upper_bound);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer_;

    pcl::Grabber& grabber_;
    boost::mutex cloud_mutex_;

    CloudConstPtr cloud_;
    pcl::visualization::PointCloudColorHandler<pcl::PointXYZI> &handler_;
        
};
