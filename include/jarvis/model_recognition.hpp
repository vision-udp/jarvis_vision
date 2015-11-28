//          Copyright Diego Ramírez November 2015
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#ifndef JARVIS_MODEL_RECOGNITION_HPP
#define JARVIS_MODEL_RECOGNITION_HPP

#include <jarvis/pcl_fwd.hpp>
#include <boost/shared_ptr.hpp>

namespace jarvis {

template <typename PointT, typename PointNT>
class model_recognition {
public:
  using cloud_t = pcl::PointCloud<PointT>;
  using normals_t = pcl::PointCloud<PointNT>;
  using cloud_const_ptr = boost::shared_ptr<const cloud_t>;
  using normals_const_ptr = boost::shared_ptr<const normals_t>;

public:
  /// \brief Sets the cloud to use.
  ///
  void set_input_cloud(const cloud_const_ptr &input_cloud) {
    cloud = input_cloud;
  }

  /// \brief Sets the input normals.
  ///
  void set_input_normals(const normals_const_ptr &input_normals) {
    normals = input_normals;
  }

  /// \brief Tests if the input cloud could be a cylinder.
  ///
  /// \param coeffs The coefficients of the found model.
  ///
  /// \returns The probability that the test is true.
  ///
  double test_cylinder(pcl::ModelCoefficients &coeffs);

  /// \brief Tests if the input cloud could be a sphere.
  ///
  /// \param coeffs The coefficients of the found model.
  ///
  /// \returns The probability that the test is true.
  ///
  double test_sphere(pcl::ModelCoefficients &coeffs);

  /// \brief Tests if the input cloud could be a cube.
  ///
  /// \returns The probability that the test is true.
  ///
  double test_cube();

private:
  cloud_const_ptr cloud;
  normals_const_ptr normals;
};

} // end namespace djp

#endif // Header guard
