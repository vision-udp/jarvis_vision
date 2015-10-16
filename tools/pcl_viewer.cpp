#include <iostream>
#include <memory>
#include <thread>
#include <chrono>

#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>

class simple_visualizer {
public:
  simple_visualizer() : viewer("3D Viewer") {}

  void run() {
    viewer.setBackgroundColor(1, 1, 1);
    viewer.addCoordinateSystem(1.0f, "global");
    viewer.initCameraParameters();
    viewer.setCameraPosition(0.0, -0.3, -0.2, 0.0, -0.3, 1.0, 0.0, -1.0, 0.0);

    auto interface = std::make_unique<pcl::OpenNIGrabber>();

    boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &)>
        cb = [&](const auto &cloud) { f_claude = cloud; };
    interface->registerCallback(cb);

    interface->start();
    while (!viewer.wasStopped()) {
      update_viewer();
      viewer.spinOnce();
      std::this_thread::sleep_for(std::chrono::milliseconds(40));
    }
    interface->stop();
  }

private:
  void update_viewer() {
    auto claude = f_claude;
    if (!claude)
      return;

    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
    // color_handler_cloud(claude, 0, 0, 0);
    if (!viewer.updatePointCloud(claude, "claude"))
      viewer.addPointCloud(claude, "claude");
    viewer.setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "claude");
  }

private:
  pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr f_claude;
  pcl::visualization::PCLVisualizer viewer;
};

int main() {
  simple_visualizer viz;
  viz.run();
}
