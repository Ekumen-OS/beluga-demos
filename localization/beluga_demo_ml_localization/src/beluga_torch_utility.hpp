#ifndef BELUGA_TORCH_UTILITY_HPP
#define BELUGA_TORCH_UTILITY_HPP

#include <cstddef>

#include <Eigen/Core>

#include <opencv2/core/mat.hpp>
#include <torch/script.h>

namespace beluga_torch {

template <typename T>
struct tensor_bridge;

template <typename T>
struct tensor_scalar_traits;

template <>
struct tensor_scalar_traits<float> {
  static constexpr auto dtype = torch::kFloat32;
};

template <>
struct tensor_scalar_traits<double> {
  static constexpr auto dtype = torch::kFloat64;
};

template <typename Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
struct tensor_bridge<Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>> {
  static torch::Tensor map(const Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>& matrix) {
    auto options = torch::TensorOptions().dtype(tensor_scalar_traits<Scalar>::dtype).requires_grad(false);
    torch::Tensor tensor = torch::empty({matrix.rows(), matrix.cols()}, options);
    using TensorMap = Eigen::Map<Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>>;
    auto mapped_tensor = TensorMap(tensor.data_ptr<Scalar>(), matrix.rows(), matrix.cols());
    if constexpr (Options & Eigen::ColMajor) {
      mapped_tensor = matrix.transpose();
    } else {
      mapped_tensor = matrix;
    }
    return tensor;
  }
};

template <>
struct tensor_bridge<cv::Mat> {
  static torch::Dtype to_dtype(int image_depth) {
    switch (image_depth) {
      case CV_8U:
        return torch::kByte;
      case CV_8S:
        return torch::kChar;
      case CV_16U:
      case CV_16S:
        return torch::kInt16;
      case CV_32S:
        return torch::kInt32;
      case CV_32F:
        return torch::kFloat32;
      case CV_64F:
        return torch::kFloat64;
      default:
        throw std::invalid_argument{"invalid image depth"};
    }
  }

  static torch::Tensor map(const cv::Mat& image) {
    auto options = torch::TensorOptions().dtype(to_dtype(image.depth())).requires_grad(false);
    auto tensor = torch::from_blob(image.data, {image.rows, image.cols, image.channels()}, options);
    // HWC to CHW
    tensor = tensor.permute({2, 0, 1});
    // Convert to float and normalize
    tensor = tensor.to(torch::kFloat32) / 255.0;
    // Add batch dimension
    return tensor.unsqueeze(0);
  }
};

template<class T>
torch::Tensor to_tensor(T&& value) {
  using U = std::remove_cv_t<std::remove_reference_t<T>>;
  return tensor_bridge<U>::map(std::forward<T>(value));
}

template <class T>
struct ivalue_bridge {
  static torch::jit::IValue wrap(T&& value) {
    return torch::jit::IValue(std::forward<T>(value));
  }
};

template <typename Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
struct ivalue_bridge<Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>> {
  static torch::jit::IValue wrap(const Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>& matrix) {
    return torch::jit::IValue(to_tensor(matrix));
  }
};

template <typename Scalar>
struct ivalue_bridge<Sophus::SE2<Scalar>> {
  static torch::jit::IValue wrap(const Sophus::SE2<Scalar>& pose) {
    return torch::jit::IValue(to_tensor(pose.params()));
  }
};

template <>
struct ivalue_bridge<cv::Mat> {
  static torch::jit::IValue wrap(const cv::Mat& image) {
    return torch::jit::IValue(to_tensor(image));
  }
};

template<class T>
torch::jit::IValue to_ivalue(T&& value) {
  using U = std::remove_cv_t<std::remove_reference_t<T>>;
  return ivalue_bridge<U>::wrap(std::forward<T>(value));
}

using Measurement = cv::Mat;

struct Pose2D {
    double x, y, theta;

    Pose2D() : x(0), y(0), theta(0) {}
    Pose2D(double x, double y, double theta) : x(x), y(y), theta(theta) {}

    double& operator[](size_t i) {
        if (i == 0) return x;
        if (i == 1) return y;
        return theta;
    }

    const double& operator[](size_t i) const {
        if (i == 0) return x;
        if (i == 1) return y;
        return theta;
    }
};

using Weight = double;

}  // namespace beluga_torch

#endif // BELUGA_TORCH_UTILITY_HPP