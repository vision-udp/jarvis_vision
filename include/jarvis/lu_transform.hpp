/// \file lu_transform.hpp
/// \brief Implements the Lu-Trnasform a fast texture descriptor.
/// For detailed reference:
///  http://www.mobvis.org/publications/eccv2006_tavakolibjoerkmanhaymaneklundh.pdf
///
/// \author Jorge Aguirre
/// \version 1.0
/// \date 2015-11-29

#include <pcl/common/common_headers.h>
#include <opencv2/core/eigen.hpp>

using namespace Eigen;
namespace jarvis {

template <typename T, size_t W = 4>
inline std::vector<T> lu_transform(const cv::Mat &mat, size_t l) {
  using matrix = Eigen::Matrix<T, W, W>;
  matrix window;
  int window_size = static_cast<int>(W);
  std::vector<T> result(window_size * window_size);

  for (int i = 0; i + window_size < mat.rows; i += window_size)
    for (int j = 0; j + window_size < mat.cols; j += window_size) {
      auto roi = mat(cv::Rect(j, i, window_size, window_size));
      cv::cv2eigen(roi, window);
      Eigen::FullPivLU<matrix> lu(window);
      matrix U = lu.matrixLU().triangularView<Eigen::Upper>();
      std::vector<T> coeff(window_size);
      for (size_t k = 0; k < window_size; ++k)
        coeff[k] = std::abs(U(k, k));
      std::sort(coeff.begin(), coeff.end(), std::greater<T>());
      auto &elem = result[i * window_size + j];
      for (size_t k = l; k < window_size; ++k)
        elem += coeff[k];
    }

  return result;
}
}
