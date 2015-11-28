//          Copyright Diego Ramírez November 2015
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#ifndef JARVIS_PLANE_EXTRACTION_HPP
#define JARVIS_PLANE_EXTRACTION_HPP

#include <jarvis/pcl_fwd.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>

#include <boost/shared_ptr.hpp>
#include <vector>
#include <cassert>
#include <cstddef> // for std::size_t

namespace jarvis {

template <typename PointT>
class plane_extractor {
  using cloud_t = pcl::PointCloud<PointT>;
  using cloud_const_ptr = boost::shared_ptr<const cloud_t>;

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

  /// \brief Sets the minimum of points allowed to a plane.
  ///
  /// \param value The value to set.
  ///
  /// \note The default value is 10000.
  ///
  void set_min_points(std::size_t value) { min_points = value; }

  /// \brief Extract the planes from the input cloud.
  ///
  /// \returns The number of extracted planes.
  ///
  std::size_t extract_planes();

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
  std::size_t min_points{10000};
  boost::shared_ptr<const pcl::PointIndices> indices;
  boost::shared_ptr<pcl::PointIndices> rem;
  std::vector<pcl::ModelCoefficients> coeffs_of;
  std::vector<pcl::PointIndices> inliers_of;
};

} // end namespace jarvis

#endif // Header guard
