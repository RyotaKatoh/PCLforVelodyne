#include "SimpleHDLViewer.h"

using namespace std;
using namespace pcl;
using namespace pcl::console;
using namespace pcl::visualization;

void SimpleHDLViewer::cloud_callback(const CloudConstPtr &cloud){
  boost::mutex::scoped_lock lock(cloud_mutex_);
  cloud_ = cloud;
}

void SimpleHDLViewer::run(){
  cloud_viewer_->addCoordinateSystem (3.0);
  cloud_viewer_->setBackgroundColor (0, 0, 0);
  cloud_viewer_->initCameraParameters ();
  cloud_viewer_->setCameraPosition (0.0, 10.0, 0.0, 0.0, 1.0, 0.0, 0);
  cloud_viewer_->setCameraClipDistances (0.0, 50.0);

  boost::function<void (const CloudConstPtr&)> cloud_cb = boost::bind (
      &SimpleHDLViewer::cloud_callback, this, _1);
  boost::signals2::connection cloud_connection = grabber_.registerCallback (
      cloud_cb);

  grabber_.start ();

  while (!cloud_viewer_->wasStopped ())
  {
    CloudConstPtr cloud;

    // See if we can get a cloud
    if (cloud_mutex_.try_lock ())
    {
      cloud_.swap (cloud);
      cloud_mutex_.unlock ();
    }

    if (cloud)
    {
      handler_.setInputCloud (cloud);
      if (!cloud_viewer_->updatePointCloud (cloud, handler_, "HDL"))
        cloud_viewer_->addPointCloud (cloud, handler_, "HDL");

      cloud_viewer_->spinOnce ();
    }

    if (!grabber_.isRunning ())
      cloud_viewer_->spin ();

    boost::this_thread::sleep (boost::posix_time::microseconds (100));
  }

  grabber_.stop ();

  cloud_connection.disconnect ();
}
