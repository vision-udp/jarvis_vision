/// \file lu_transform.hpp
/// \brief Implements the Lu-Trnasform a fast texture descriptor.
/// For detailed reference:
///  http://www.mobvis.org/publications/eccv2006_tavakolibjoerkmanhaymaneklundh.pdf
///
/// \author Jorge Aguirre
/// \version 1.0
/// \date 2015-11-29

#ifndef JARVIS_LU_TRANSFORM_HPP
#define JARVIS_LU_TRANSFORM_HPP

#include <numeric>
#include <iterator>
#include <cassert>

#include <pcl/common/common_headers.h>
#include <opencv2/core/eigen.hpp>

namespace jarvis {

template <typename T, size_t W = 8>
inline std::vector<T> lu_transform(const cv::Mat &source, const ptrdiff_t l,
                                   const size_t spacing) {
  using matrix = Eigen::Matrix<T, W, W>;
  const auto window_size = W;
  assert(l > 0 && static_cast<size_t>(l) < window_size);
  assert(spacing > 0);

  steady_timer timer;

  const int anchor = static_cast<int>(window_size) / 2;
  cv::Mat expanded;
  cv::copyMakeBorder(source, expanded, anchor, anchor, anchor, anchor,
                     cv::BORDER_CONSTANT | cv::BORDER_ISOLATED);

  const auto rows = static_cast<size_t>(source.rows) / spacing;
  const auto cols = static_cast<size_t>(source.cols) / spacing;

  std::vector<T> result;
  result.reserve(rows * cols);

  auto roi = [&expanded, &spacing](size_t i, size_t j) {
    int x = static_cast<int>(j * spacing);
    int y = static_cast<int>(i * spacing);
    return expanded(cv::Rect(x, y, window_size, window_size));
  };

  matrix window;
  Eigen::FullPivLU<matrix> lu;
  std::vector<T> coeff(window_size);
  for (size_t i = 0; i < rows; ++i)
    for (size_t j = 0; j < cols; ++j) {
      cv::cv2eigen(roi(i, j), window);
      auto diag_u = lu.compute(window).matrixLU().diagonal().cwiseAbs();
      Eigen::Map<Eigen::Matrix<T, W, 1>>(coeff.data(), diag_u.size()) = diag_u;

      std::sort(coeff.begin(), coeff.end(), std::greater<T>());
      auto first = std::next(coeff.begin(), l);
      auto last = coeff.end();
      result.push_back(std::accumulate(first, last, T{0}));
    }

  assert(result.size() == rows * cols);
  return result;
}
}
#endif
