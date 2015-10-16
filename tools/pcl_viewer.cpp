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
    using pcl::PointCloud;
    using pcl::PointXYZRGBA;
    using pcl::OpenNIGrabber;
    using namespace std::chrono_literals;

    viewer.setBackgroundColor(1, 1, 1);
    viewer.addCoordinateSystem(1.0f, "global");
    viewer.initCameraParameters();
    viewer.setCameraPosition(0.0, -0.3, -0.2, 0.0, -0.3, 1.0, 0.0, -1.0, 0.0);

    auto interface = std::make_unique<OpenNIGrabber>();

    boost::function<void(const PointCloud<PointXYZRGBA>::ConstPtr &)> cb = [&](
        const auto &cloud) { f_claude = cloud; };
    interface->registerCallback(cb);

    interface->start();
    while (!viewer.wasStopped()) {
      update_viewer();
      viewer.spinOnce();
      std::this_thread::sleep_for(40ms);
    }
    interface->stop();
  }

private:
  void update_viewer() {
    using pcl::visualization::PCL_VISUALIZER_POINT_SIZE;

    auto claude = f_claude;
    if (!claude)
      return;

    if (!viewer.updatePointCloud(claude, "claude"))
      viewer.addPointCloud(claude, "claude");
    viewer.setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, 2,
                                            "claude");
  }

private:
  pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr f_claude;
  pcl::visualization::PCLVisualizer viewer;
};

int main() {
  simple_visualizer viz;
  viz.run();
}
