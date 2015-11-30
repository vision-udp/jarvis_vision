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

namespace {
template <typename Matrix>
Matrix lu_triangular_upper(Matrix &window, const cv::Mat &roi) {
  cv::cv2eigen(roi, window);
  Eigen::FullPivLU<Matrix> lu(window);
  return lu.matrixLU().template triangularView<Eigen::Upper>();
}
}

namespace jarvis {

template <typename T, size_t W = 8>
inline std::vector<T> lu_transform(const cv::Mat &source, const ptrdiff_t l,
                                   const size_t spacing) {
  using matrix = Eigen::Matrix<T, W, W>;
  const auto window_size = W;
  assert(l > 0 && static_cast<size_t>(l) < window_size);

  matrix window;
  auto apply_transform = [&](const cv::Mat &roi) {
    assert(static_cast<size_t>(roi.cols) == window_size &&
           static_cast<size_t>(roi.rows) == window_size);
    Eigen::Matrix<T, W, 1> diag_u =
        lu_triangular_upper(window, roi).diagonal().cwiseAbs();
    std::vector<T> coeff(window_size);
    Eigen::Map<Eigen::Matrix<T, W, 1>>(coeff.data(), diag_u.size()) = diag_u;
    std::sort(coeff.begin(), coeff.end(), std::greater<T>());
    auto begin = std::next(coeff.begin(), l);
    auto end = coeff.end();
    return std::accumulate(begin, end, 0);
  };

  cv::Mat expanded;
  const int anchor = static_cast<int>(window_size) / 2;
  cv::copyMakeBorder(source, expanded, anchor, anchor, anchor, anchor,
                     cv::BORDER_DEFAULT);

  auto rows = static_cast<size_t>(source.rows);
  auto cols = static_cast<size_t>(source.cols);

  if (spacing) {
    rows /= spacing;
    cols /= spacing;
  }

  std::vector<T> result;
  result.reserve(rows * cols);

  auto max_height = expanded.rows - anchor;
  auto max_width = expanded.cols - anchor;
  for (int i = anchor; i < max_height; i += spacing)
    for (int j = anchor; j < max_width; j += spacing) {
      auto roi = cv::Rect(j - anchor, i - anchor, window_size, window_size);
      result.push_back(apply_transform(expanded(roi)));
    }

  assert(result.size == rows * cols);
  return result;
}
}
#endif
