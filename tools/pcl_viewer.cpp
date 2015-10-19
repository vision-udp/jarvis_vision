#include <iostream>
#include <memory>
#include <thread>
#include <chrono>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/visualization/pcl_visualizer.h>

template <typename Point>
void statistical_outliner_removal(
    const typename pcl::PointCloud<Point>::ConstPtr &cloud, const int k,
    const double stddev_mult, pcl::PointCloud<Point> &output_cloud) {
  pcl::StatisticalOutlierRemoval<Point> sor;
  sor.setInputCloud(cloud);
  sor.setMeanK(k);
  sor.setStddevMulThresh(stddev_mult);
  sor.setNegative(true);
  sor.filter(output_cloud);
}

template <typename Point>
void downsample(const typename pcl::PointCloud<Point>::ConstPtr &cloud,
                pcl::PointCloud<Point> &output_cloud) {
  pcl::VoxelGrid<Point> filter;
  filter.setInputCloud(cloud);
  filter.setLeafSize(0.007f, 0.007f, 0.007f);
  filter.filter(output_cloud);
}

template <typename Point>
void find_cylinders(const typename pcl::PointCloud<Point>::ConstPtr &cloud,
                    pcl::PointCloud<Point> &output_cloud) {
  using model_t = pcl::SampleConsensusModelCylinder<Point, pcl::PointXYZ>;
  auto model = boost::make_shared<model_t>(cloud);
  pcl::RandomSampleConsensus<Point> ransac(model);
  ransac.setDistanceThreshold(0.01);
  ransac.computeModel();

  std::vector<int> inliers;
  ransac.getInliers(inliers);
  pcl::copyPointCloud(*cloud, inliers, output_cloud);
}

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
    // register_keyboard_manager();

    auto interface = std::make_unique<OpenNIGrabber>();

    boost::function<void(const PointCloud<PointXYZ>::ConstPtr &)> cb;
    cb = [&](const auto &cloud) { f_claude = cloud; };
    interface->registerCallback(cb);
    interface->getDevice()->setDepthRegistration(true);

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

    using cloud_t = pcl::PointCloud<pcl::PointXYZ>;
    auto filtered_cloud = boost::make_shared<cloud_t>();

    // downsample(claude, *filtered_cloud);
    // statistical_outliner_removal(claude, 40, 2.00, *filtered_cloud);
    find_cylinders(claude, *filtered_cloud);

    //    using handler_t =
    //        pcl::visualization::PointCloudColorHandlerCustom<PointXYZRGBA>;
    //    handler_t handler(filtered_cloud, 255, 0, 0);
    //
    //    if (!viewer.updatePointCloud(claude, "francisco"))
    //      viewer.addPointCloud(claude, "francisco");

    if (!viewer.updatePointCloud(filtered_cloud, "claude"))
      viewer.addPointCloud(filtered_cloud, "claude");
    //    viewer.setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, 1,
    //                                            "claude");
  }

  //  void register_keyboard_manager() {
  //    using pcl::visualization::KeyboardEvent;
  //    boost::function<void(const KeyboardEvent &)> callback;
  //    callback = [](const KeyboardEvent &ev) {
  //      std::cout << "KeyEv" << std::endl;
  //      if (ev.keyUp())
  //        return;
  //      std::cout << "HOLA" << std::endl;
  //      if (ev.getKeyCode() == 0)
  //        std::cout << ev.getKeySym() << std::endl;
  //    };
  //    viewer.registerKeyboardCallback(callback);
  //  }

private:
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr f_claude;
  pcl::visualization::PCLVisualizer viewer;
};

int main() {
  simple_visualizer viz;
  viz.run();
}
