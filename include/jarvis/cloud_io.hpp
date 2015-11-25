//          Copyright Diego Ramírez November 2015
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#ifndef JARVIS_CLOUD_IO_HPP
#define JARVIS_CLOUD_IO_HPP

#include <jarvis/pcl_fwd.hpp>
#include <boost/shared_ptr.hpp>
#include <string>

namespace jarvis {

/// \brief Loads a point cloud from the given pcd filename.
///
/// \param filename Location of the point clout to load.
///
/// \returns The loaded point cloud.
///
/// \throws std::runtime_error on error.
///
template <typename PointT>
boost::shared_ptr<pcl::PointCloud<PointT>>
load_cloud(const std::string &filename);

} // end namespace jarvis

#endif // Header guard
