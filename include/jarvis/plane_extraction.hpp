//          Copyright Diego Ramírez November 2015
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#ifndef JARVIS_PLANE_EXTRACTION_HPP
#define JARVIS_PLANE_EXTRACTION_HPP

#include <jarvis/pcl_fwd.hpp>
#include <vector>                  // for std::vector
#include <cassert>                 // for assert
#include <cstddef>                 // for std::size_t
#include <boost/shared_ptr.hpp>    // for boost::shared_ptr
#include <pcl/ModelCoefficients.h> // for ModelCoefficients
#include <pcl/PointIndices.h>      // for PointIndices

namespace jarvis {

template <typename PointT>
class plane_extractor {
  using cloud_t = pcl::PointCloud<PointT>;
  using normals_t = pcl::PointCloud<pcl::Normal>;
  using cloud_const_ptr = boost::shared_ptr<const cloud_t>;
  using normals_const_ptr = boost::shared_ptr<const normals_t>;

public:
  /// \brief Sets the input cloud.
  ///
  /// \param input_cloud The input cloud to use.
  ///
  void set_input_cloud(const cloud_const_ptr &input_cloud) {
    cloud = input_cloud;
  }

  /// \brief Sets the indices of the points to use.
  ///
  /// \param input_indices The input cloud to use.
  ///
  void
  set_indices(const boost::shared_ptr<const pcl::PointIndices> &input_indices) {
    indices = input_indices;
  }

  /// \brief Sets the normals of the input cloud.
  ///
  /// \param input_normals The normals of the input cloud.
  ///
  void set_input_normals(const normals_const_ptr &input_normals) {
    normals = input_normals;
  }

  /// \brief Extract the planes from the input cloud.
  ///
  /// \returns The number of extracted planes.
  ///
  std::size_t extract_planes();

  /// \brief Extract the planes from the input cloud by using its normals.
  ///
  /// \returns The number of extracted planes.
  ///
  std::size_t extract_planes_from_normals();

  /// \brief Returns the number of planes extracted in the last execution.
  ///
  std::size_t get_num_planes() const { return inliers_of.size(); }

  /// \brief Returns the indices which do not belong to any plane.
  ///
  /// \pre \c extract_planes() must have been executed.
  ///
  boost::shared_ptr<pcl::PointIndices> get_remaining_indices() const {
    return rem;
  }

  /// \brief Returns a reference to the inlier indices of the \p nth extracted
  /// plane.
  ///
  const pcl::PointIndices &get_inliers(const std::size_t nth) const {
    assert(nth < get_num_planes());
    return inliers_of[nth];
  }

  /// \brief Returns a reference to the model coefficients of the \p nth
  /// extracted plane.
  ///
  const pcl::ModelCoefficients &get_coefficients(const std::size_t nth) const {
    assert(nth < get_num_planes());
    return coeffs_of[nth];
  }

private:
  cloud_const_ptr cloud;
  normals_const_ptr normals;
  boost::shared_ptr<const pcl::PointIndices> indices;
  boost::shared_ptr<pcl::PointIndices> rem;
  std::vector<pcl::ModelCoefficients> coeffs_of;
  std::vector<pcl::PointIndices> inliers_of;
};

} // end namespace jarvis

#endif // Header guard
