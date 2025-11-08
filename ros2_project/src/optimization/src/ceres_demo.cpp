#include <iostream>
#include <vector>
#include <chrono>
#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std;

// 代价函数的计算模型
struct CURVE_FITTING_COST {
  CURVE_FITTING_COST(double x, double y) : _x(x), _y(y) {}

  // 残差的计算
  template<typename T>
  bool operator()(
    const T *const abc, // 模型参数，有3维
    T *residual) const {
    residual[0] = T(_y) - ceres::exp(abc[0] * T(_x) * T(_x) + abc[1] * T(_x) + abc[2]);
    return true;
  }

  const double _x, _y;    // x,y数据
};

class CurveFittingNode : public rclcpp::Node {
public:
  CurveFittingNode() : Node("curve_fitting_node") {
    // 创建参数发布者
    param_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "fitted_parameters", 10);
    
    RCLCPP_INFO(this->get_logger(), "Curve fitting node initialized");
    
    // 执行曲线拟合
    perform_fitting();
  }

private:
  void perform_fitting() {
    // 真实参数和初始估计值
    double ar = 1.0, br = 2.0, cr = 1.0;         // 真实参数值
    double ae = 2.0, be = -1.0, ce = 5.0;        // 估计参数值
    int N = 100;                                 // 数据点数量
    double w_sigma = 1.0;                        // 噪声Sigma值
    cv::RNG rng;                                 // OpenCV随机数产生器

    // 生成带噪声的数据
    vector<double> x_data, y_data;
    for (int i = 0; i < N; i++) {
      double x = i / 100.0;
      x_data.push_back(x);
      y_data.push_back(exp(ar * x * x + br * x + cr) + rng.gaussian(w_sigma * w_sigma));
    }

    double abc[3] = {ae, be, ce};  // 待优化参数

    // 构建最小二乘问题
    ceres::Problem problem;
    for (int i = 0; i < N; i++) {
      problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
          new CURVE_FITTING_COST(x_data[i], y_data[i])
        ),
        nullptr,
        abc
      );
    }

    // 配置求解器
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = false;  // 不在stdout输出，改用ROS日志
    options.logging_type = ceres::PER_MINIMIZER_ITERATION;
    // options.log_to_stdout = false;

    ceres::Solver::Summary summary;
    auto t1 = chrono::steady_clock::now();
    ceres::Solve(options, &problem, &summary);
    auto t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);

    // 输出结果到ROS日志
    RCLCPP_INFO(this->get_logger(), "Solve time cost: %.6f seconds", time_used.count());
    RCLCPP_INFO(this->get_logger(), "Optimization summary: %s", summary.BriefReport().c_str());
    RCLCPP_INFO(this->get_logger(), "Estimated a: %.6f, b: %.6f, c: %.6f", abc[0], abc[1], abc[2]);
    RCLCPP_INFO(this->get_logger(), "Ground truth a: %.6f, b: %.6f, c: %.6f", ar, br, cr);

    // 发布拟合参数
    auto msg = std_msgs::msg::Float32MultiArray();
    msg.data = {static_cast<float>(abc[0]), static_cast<float>(abc[1]), static_cast<float>(abc[2])};
    param_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Fitted parameters published");
  }

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr param_pub_;
};

int main(int argc, char **argv) {
  // 初始化ROS 2
  rclcpp::init(argc, argv);
  
  // 创建并运行节点
  auto node = std::make_shared<CurveFittingNode>();
  
  // 保持节点运行
  rclcpp::spin(node);
  
  // 关闭ROS 2
  rclcpp::shutdown();
  return 0;
}