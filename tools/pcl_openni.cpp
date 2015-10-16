//#include <pcl/io/openni_grabber.h>
#include <pcl/io/openni_camera/openni_driver.h>

//#include <pcl/visualization/cloud_viewer.h>

// class SimpleOpenNIViewer {
// public:
//  SimpleOpenNIViewer() : viewer("PCL OpenNI Viewer") {}
//
//  void cloud_cb_(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud) {
//    if (!viewer.wasStopped())
//      viewer.showCloud(cloud);
//  }
//
//  void run() {
//    pcl::Grabber *interface = new pcl::OpenNIGrabber();
//
//    boost::function<void(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &)> f
//    =
//        boost::bind(&SimpleOpenNIViewer::cloud_cb_, this, _1);
//
//    interface->registerCallback(f);
//
//    interface->start();
//
//    while (!viewer.wasStopped()) {
//      boost::this_thread::sleep(boost::posix_time::seconds(1));
//    }
//
//    interface->stop();
//  }
//
//  pcl::visualization::CloudViewer viewer;
//};

int main() {
  using openni_wrapper::OpenNIDriver;
  OpenNIDriver &driver = OpenNIDriver::getInstance();
  driver.updateDeviceList();
  std::cout << "Num devices: " << driver.getNumberDevices() << '\n';
}
