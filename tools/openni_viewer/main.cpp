
#include <iostream>
#include <memory>
#include <thread>
#include <chrono>

#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

class simple_visualizer {
  using PointXYZRGBA = pcl::PointXYZRGBA;
  using PointXYZ = pcl::PointXYZ;

public:
  simple_visualizer() : viewer("3D Viewer", false) {}

  void run() {
    using pcl::PointCloud;
    using pcl::OpenNIGrabber;
    using namespace std::chrono_literals;

    viewer.setFullScreen(true);
    viewer.setWindowBorders(true);
    viewer.setBackgroundColor(0, 0, 0);
    viewer.initCameraParameters();
    viewer.setCameraPosition(0.0, -0.3, -0.2, 0.0, -0.3, 1.0, 0.0, -1.0, 0.0);
    viewer.createInteractor();

    auto interface = std::make_unique<OpenNIGrabber>();
    auto device = interface->getDevice();
    device->setDepthRegistration(true);

    boost::function<void(const PointCloud<PointXYZRGBA>::ConstPtr &)> cb;
    cb = [&](const auto &cloud) { f_claude = cloud; };

    interface->registerCallback(cb);
    interface->start();

    while (!viewer.wasStopped()) {
      update_viewer();
      viewer.spinOnce();
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
  }

private:
  pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr f_claude;
  pcl::visualization::PCLVisualizer viewer;
};

int main() {
  simple_visualizer viz;
  viz.run();
}
