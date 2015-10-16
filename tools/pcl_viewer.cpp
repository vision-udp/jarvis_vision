#include <iostream>
#include <memory>
#include <thread>
#include <chrono>

#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>

class simple_visualizer {
public:
  simple_visualizer() : viewer("3D Viewer", false) {}

  void run() {
    using pcl::PointCloud;
    using pcl::PointXYZRGBA;
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

    boost::function<void(const PointCloud<PointXYZRGBA>::ConstPtr &)> cb;
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
    using pcl::visualization::PCL_VISUALIZER_LINE_WIDTH;

    auto claude = f_claude;
    if (!claude)
      return;

    if (!viewer.updatePointCloud(claude, "claude"))
      viewer.addPointCloud(claude, "claude");
    viewer.setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, 2,
                                            "claude");
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
  pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr f_claude;
  pcl::visualization::PCLVisualizer viewer;
};

int main() {
  simple_visualizer viz;
  viz.run();
}
