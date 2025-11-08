#include <iostream>
#include <vector>
#include <cmath>
#include <chrono>
#include <memory>  // 新增：包含std::make_unique所需的头文件
#include <g2o/core/g2o_core_api.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std;

// 曲线模型的顶点，模板参数：优化变量维度和数据类型
class CurveFittingVertex : public g2o::BaseVertex<3, Eigen::Vector3d> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // 重置
  virtual void setToOriginImpl() override {
    _estimate << 0, 0, 0;
  }

  // 更新
  virtual void oplusImpl(const double *update) override {
    _estimate += Eigen::Vector3d(update);
  }

  // 存盘和读盘：留空
  virtual bool read(istream &in) { return true; }
  virtual bool write(ostream &out) const { return true; }
};

// 误差模型 模板参数：观测值维度，类型，连接顶点类型
class CurveFittingEdge : public g2o::BaseUnaryEdge<1, double, CurveFittingVertex> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CurveFittingEdge(double x) : BaseUnaryEdge(), _x(x) {}

  // 计算曲线模型误差
  virtual void computeError() override {
    const CurveFittingVertex *v = static_cast<const CurveFittingVertex *> (_vertices[0]);
    const Eigen::Vector3d abc = v->estimate();
    _error(0, 0) = _measurement - std::exp(abc(0, 0) * _x * _x + abc(1, 0) * _x + abc(2, 0));
  }

  // 计算雅可比矩阵
  virtual void linearizeOplus() override {
    const CurveFittingVertex *v = static_cast<const CurveFittingVertex *> (_vertices[0]);
    const Eigen::Vector3d abc = v->estimate();
    double y = exp(abc[0] * _x * _x + abc[1] * _x + abc[2]);
    _jacobianOplusXi[0] = -_x * _x * y;
    _jacobianOplusXi[1] = -_x * y;
    _jacobianOplusXi[2] = -y;
  }

  virtual bool read(istream &in) { return true; }
  virtual bool write(ostream &out) const { return true; }

public:
  double _x;  // x 值， y 值为 _measurement
};

class G2OCurveFittingNode : public rclcpp::Node {
public:
  G2OCurveFittingNode() : Node("g2o_curve_fitting_node") {
    // 创建参数发布者
    param_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "g2o_fitted_parameters", 10);
    
    RCLCPP_INFO(this->get_logger(), "G2O Curve fitting node initialized");
    
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

    // 构建图优化，设定g2o
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, 1>> BlockSolverType;  // 每个误差项优化变量维度为3，误差值维度为1
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType; // 线性求解器类型

    // 梯度下降方法，选择GN方法 - 修改此处：使用std::make_unique替代g2o::make_unique
    auto solver = new g2o::OptimizationAlgorithmGaussNewton(
      std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>()));  // 关键修改
    g2o::SparseOptimizer optimizer;     // 图模型
    optimizer.setAlgorithm(solver);   // 设置求解器
    optimizer.setVerbose(false);      // 关闭g2o自身的输出，改用ROS日志

    // 往图中增加顶点
    CurveFittingVertex *v = new CurveFittingVertex();
    v->setEstimate(Eigen::Vector3d(ae, be, ce));
    v->setId(0);
    optimizer.addVertex(v);

    // 往图中增加边
    for (int i = 0; i < N; i++) {
      CurveFittingEdge *edge = new CurveFittingEdge(x_data[i]);
      edge->setId(i);
      edge->setVertex(0, v);                // 设置连接的顶点
      edge->setMeasurement(y_data[i]);      // 观测数值
      // 信息矩阵：协方差矩阵之逆
      edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * 1 / (w_sigma * w_sigma)); 
      optimizer.addEdge(edge);
    }

    // 执行优化
    RCLCPP_INFO(this->get_logger(), "Starting optimization...");
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.initializeOptimization();
    int iterations = optimizer.optimize(10);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);

    // 输出优化结果到ROS日志
    RCLCPP_INFO(this->get_logger(), "Optimization completed in %d iterations", iterations);
    RCLCPP_INFO(this->get_logger(), "Solve time cost: %.6f seconds", time_used.count());
    
    Eigen::Vector3d abc_estimate = v->estimate();
    RCLCPP_INFO(this->get_logger(), "Estimated parameters: a=%.6f, b=%.6f, c=%.6f", 
                abc_estimate[0], abc_estimate[1], abc_estimate[2]);
    RCLCPP_INFO(this->get_logger(), "Ground truth parameters: a=%.6f, b=%.6f, c=%.6f", 
                ar, br, cr);

    // 发布拟合参数
    auto msg = std_msgs::msg::Float32MultiArray();
    msg.data = {static_cast<float>(abc_estimate[0]), 
                static_cast<float>(abc_estimate[1]), 
                static_cast<float>(abc_estimate[2])};
    param_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Fitted parameters published to topic 'g2o_fitted_parameters'");
  }

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr param_pub_;
};

int main(int argc, char **argv) {
  // 初始化ROS 2
  rclcpp::init(argc, argv);
  
  // 创建并运行节点
  auto node = std::make_shared<G2OCurveFittingNode>();
  
  // 保持节点运行
  rclcpp::spin(node);
  
  // 关闭ROS 2
  rclcpp::shutdown();
  return 0;
}