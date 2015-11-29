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

template <typename T, size_t W = 4>
inline std::vector<T> lu_transform(const cv::Mat &mat, ptrdiff_t l) {
  using matrix = Eigen::Matrix<T, W, W>;
  matrix window;

  const auto window_size = W;
  const auto height = static_cast<size_t>(mat.rows) / window_size;
  const auto width = static_cast<size_t>(mat.cols) / window_size;

  std::vector<T> result;
  result.reserve(width * height);

  auto roi = [&mat](size_t i, size_t j) {
    int x = static_cast<int>(j * window_size);
    int y = static_cast<int>(i * window_size);
    return mat(cv::Rect(x, y, window_size, window_size));
  };

  std::vector<T> coeff(window_size);
  for (size_t i = 0; i < height; ++i)
    for (size_t j = 0; j < width; ++j) {
      matrix U = lu_triangular_upper(window, roi(i, j));

      for (size_t k = 0; k < window_size; ++k) {
        auto index = static_cast<long>(k);
        coeff[k] = std::abs(U(index, index));
      }

      std::sort(coeff.begin(), coeff.end(), std::greater<T>());

      auto begin = std::next(coeff.begin(), l);
      auto end = coeff.end();
      result[i * width + j] = std::accumulate(begin, end, 0);
    }

  return result;
}
}
#endif
