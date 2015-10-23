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

template <typename Point>
void statistical_outliner_removal(
    const typename pcl::PointCloud<Point>::ConstPtr &cloud,
    pcl::PointCloud<Point> &output_cloud) {
  pcl::StatisticalOutlierRemoval<Point> sor;
  sor.setInputCloud(cloud);
  sor.setMeanK(50);
  sor.setStddevMulThresh(0.5);
  sor.setNegative(false);
  sor.filter(output_cloud);
}

template <typename Point>
auto statistical_outliner_removal(
    const typename pcl::PointCloud<Point>::ConstPtr &cloud) {
  auto output = boost::make_shared<pcl::PointCloud<Point>>();
  statistical_outliner_removal(cloud, *output);
  return output;
}

template <typename Point>
void downsample(const typename pcl::PointCloud<Point>::ConstPtr &cloud,
                pcl::PointCloud<Point> &output_cloud) {
  pcl::VoxelGrid<Point> filter;
  filter.setInputCloud(cloud);
  filter.setLeafSize(0.005f, 0.005f, 0.005f);
  filter.filter(output_cloud);
}

template <typename Point>
auto downsample(const typename pcl::PointCloud<Point>::ConstPtr &cloud) {
  auto output = boost::make_shared<pcl::PointCloud<Point>>();
  downsample(cloud, *output);
  return output;
}

template <typename Point>
void passthrough(const typename pcl::PointCloud<Point>::ConstPtr &cloud,
                 pcl::PointCloud<Point> &output_cloud) {
  pcl::PassThrough<Point> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.1f, 2.5f);
  pass.filter(output_cloud);
}

template <typename Point>
auto passthrough(const typename pcl::PointCloud<Point>::ConstPtr &cloud) {
  auto output = boost::make_shared<pcl::PointCloud<Point>>();
  passthrough(cloud, *output);
  return output;
}

// Returns remaining PointCloud
template <typename Point>
auto find_planes(const typename pcl::PointCloud<Point>::ConstPtr &cloud,
                 pcl::PointCloud<Point> &output_cloud) {

  pcl::NormalEstimation<Point, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<Point, pcl::Normal> seg;
  pcl::ExtractIndices<Point> extract;

  auto inliers = boost::make_shared<pcl::PointIndices>();
  auto cloud_filtered = boost::make_shared<pcl::PointCloud<Point>>(*cloud);

  size_t count = 0;
  pcl::PointCloud<Point> partial_output;
  auto normals = boost::make_shared<pcl::PointCloud<pcl::Normal>>();
  pcl::ModelCoefficients coefficients;

  while (true) {
    auto tree = boost::make_shared<pcl::search::KdTree<Point>>();

    // Estimate normals
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud_filtered);
    ne.setKSearch(50);
    ne.compute(*normals);

    // Segmentation
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_MSAC);
    // seg.setNormalDistanceWeight(0.1);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.003);
    // seg.setRadiusLimits(0.02, 0.15);
    seg.setInputCloud(cloud_filtered);
    seg.setInputNormals(normals);
    seg.segment(*inliers, coefficients);

    if (inliers->indices.size() < 1000)
      break;
    ++count;

    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(partial_output);

    output_cloud += partial_output;

    auto new_filtered = boost::make_shared<pcl::PointCloud<Point>>();
    extract.setNegative(true);
    extract.filter(*new_filtered);
    cloud_filtered = statistical_outliner_removal<Point>(new_filtered);
  }
  std::clog << "Found planes: " << count << std::endl;
  return cloud_filtered;
}

// Returns remaining PointCloud
template <typename Point>
auto find_cylinders(const typename pcl::PointCloud<Point>::ConstPtr &cloud,
                    pcl::PointCloud<Point> &output_cloud) {

  pcl::NormalEstimation<Point, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<Point, pcl::Normal> seg;
  pcl::ExtractIndices<Point> extract;

  auto inliers = boost::make_shared<pcl::PointIndices>();
  auto cloud_filtered = boost::make_shared<pcl::PointCloud<Point>>(*cloud);

  size_t count = 0;
  pcl::PointCloud<Point> partial_output;
  auto normals = boost::make_shared<pcl::PointCloud<pcl::Normal>>();
  pcl::ModelCoefficients coefficients;

  while (true) {
    auto tree = boost::make_shared<pcl::search::KdTree<Point>>();

    // Estimate normals
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud_filtered);
    ne.setKSearch(50);
    ne.compute(*normals);

    // Segmentation
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_MSAC);
    // seg.setNormalDistanceWeight(0.1);
    seg.setMaxIterations(5000);
    seg.setDistanceThreshold(0.025);
    seg.setRadiusLimits(0.01, 1);
    seg.setInputCloud(cloud_filtered);
    seg.setInputNormals(normals);
    seg.segment(*inliers, coefficients);

    if (inliers->indices.size() < 50)
      break;
    ++count;

    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(partial_output);

    output_cloud += partial_output;

    auto new_filtered = boost::make_shared<pcl::PointCloud<Point>>();
    extract.setNegative(true);
    extract.filter(*new_filtered);
    cloud_filtered = statistical_outliner_removal<Point>(new_filtered);
  }
  std::clog << "Found cylinders: " << count << std::endl;
  return cloud_filtered;
}

template <typename Point>
auto find_spheres(const typename pcl::PointCloud<Point>::ConstPtr &cloud,
                  pcl::PointCloud<Point> &output_cloud) {

  pcl::NormalEstimation<Point, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<Point, pcl::Normal> seg;
  pcl::ExtractIndices<Point> extract;

  auto inliers = boost::make_shared<pcl::PointIndices>();
  auto cloud_filtered = boost::make_shared<pcl::PointCloud<Point>>(*cloud);

  size_t count = 0;
  pcl::PointCloud<Point> partial_output;
  auto normals = boost::make_shared<pcl::PointCloud<pcl::Normal>>();
  pcl::ModelCoefficients coefficients;

  while (true) {
    auto tree = boost::make_shared<pcl::search::KdTree<Point>>();

    // Estimate normals
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud_filtered);
    ne.setKSearch(50);
    ne.compute(*normals);

    // Segmentation
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_SPHERE);
    seg.setMethodType(pcl::SAC_MSAC);
    // seg.setNormalDistanceWeight(0.1);
    seg.setMaxIterations(5000);
    seg.setDistanceThreshold(0.025);
    seg.setRadiusLimits(0.01, 1);
    seg.setInputCloud(cloud_filtered);
    seg.setInputNormals(normals);
    seg.segment(*inliers, coefficients);

    if (inliers->indices.size() < 50)
      break;
    ++count;

    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(partial_output);

    output_cloud += partial_output;

    auto new_filtered = boost::make_shared<pcl::PointCloud<Point>>();
    extract.setNegative(true);
    extract.filter(*new_filtered);
    cloud_filtered = statistical_outliner_removal<Point>(new_filtered);
  }
  std::clog << "Found spheres: " << count << std::endl;
  return cloud_filtered;
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
    // interface->getDevice()->setDepthRegistration(true);

    interface->start();
    while (!viewer.wasStopped()) {
      update_viewer();
      std::clog << "Sleep" << std::endl;
      viewer.spinOnce();
      std::clog << "Awake" << std::endl;
    }
    interface->stop();
  }

private:
  void update_viewer() {
    using pcl::visualization::PCL_VISUALIZER_POINT_SIZE;
    using handler_t =
        pcl::visualization::PointCloudColorHandlerCustom<PointXYZ>;

    auto claude = f_claude;
    if (!claude)
      return;

    using cloud_t = pcl::PointCloud<pcl::PointXYZ>;
    auto planes_cloud = boost::make_shared<cloud_t>();
    auto cylinders_cloud = boost::make_shared<cloud_t>();
    auto spheres_cloud = boost::make_shared<cloud_t>();

    handler_t claude_handler(claude, 255, 128, 255);

    claude = passthrough<PointXYZ>(claude);
    claude = downsample<PointXYZ>(claude);

    if (!viewer.updatePointCloud(claude, claude_handler, "claude"))
      viewer.addPointCloud(claude, claude_handler, "claude");

    claude = find_planes(claude, *planes_cloud);
    claude = find_cylinders(claude, *cylinders_cloud);
    claude = find_spheres(claude, *spheres_cloud);

    handler_t plane_handler(planes_cloud, 255, 0, 0);
    handler_t cylinder_handler(cylinders_cloud, 0, 255, 0);
    handler_t sphere_handler(claude, 0, 0, 255);

    if (!viewer.updatePointCloud(planes_cloud, plane_handler, "planes"))
      viewer.addPointCloud(planes_cloud, plane_handler, "planes");

    if (!viewer.updatePointCloud(cylinders_cloud, cylinder_handler,
                                 "cylinders"))
      viewer.addPointCloud(cylinders_cloud, cylinder_handler, "cylinders");

    if (!viewer.updatePointCloud(spheres_cloud, sphere_handler, "spheres"))
      viewer.addPointCloud(spheres_cloud, sphere_handler, "spheres");
  }

private:
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr f_claude;
  pcl::visualization::PCLVisualizer viewer;
};

int main() {
  simple_visualizer viz;
  viz.run();
}
