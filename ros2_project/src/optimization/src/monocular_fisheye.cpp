#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <chrono>
#include "../../../../thirdparties/ORB_SLAM2/include/System.h"

// 命名空间别名，简化代码
namespace ORB_SLAM = ORB_SLAM2;
using CompressedImageMsg = sensor_msgs::msg::CompressedImage;

/**
 * @brief 获取当前系统时间（UTC），返回自epoch以来的秒数
 * @return 时间戳（秒）
 */
double get_current_timestamp() 
{
    auto now = std::chrono::system_clock::now();
    return std::chrono::duration_cast<std::chrono::duration<double>>(
        now.time_since_epoch()
    ).count();
}

/**
 * @brief 压缩图像订阅节点，用于接收图像并传递给ORB-SLAM2系统
 */
class SlamImageSubscriber : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数，初始化节点和ORB-SLAM2系统
     * @param vocab_path ORB字典路径
     * @param config_path 配置文件路径
     */
    SlamImageSubscriber(const std::string& vocab_path, 
                       const std::string& config_path) 
        : Node("slam_image_subscriber"),
          slam_system_(vocab_path, config_path, ORB_SLAM::System::MONOCULAR, true)
    {
        // 从参数服务器获取话题名称，默认使用/camera/image/compressed
        std::string image_topic;
        this->declare_parameter("image_topic", "/camera/image/compressed");
        this->get_parameter("image_topic", image_topic);

        // 创建订阅者，使用QoS配置提高可靠性
        subscription_ = this->create_subscription<CompressedImageMsg>(
            image_topic,
            rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),  // 最佳-effort模式适合图像传输
            std::bind(&SlamImageSubscriber::image_callback, this, std::placeholders::_1)
        );
        
        RCLCPP_INFO(this->get_logger(), "SLAM图像订阅节点已启动，正在监听 %s 话题...", 
                   image_topic.c_str());
    }

    /**
     * @brief 析构函数，关闭SLAM系统
     */
    ~SlamImageSubscriber() override
    {
        RCLCPP_INFO(this->get_logger(), "关闭SLAM系统...");
        slam_system_.Shutdown();
        // 保存轨迹（如果需要）
        slam_system_.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    }

private:
    /**
     * @brief 图像回调函数，处理接收到的压缩图像并传递给SLAM系统
     * @param msg 压缩图像消息
     */
    void image_callback(const CompressedImageMsg::SharedPtr msg)
    {
        if (!msg)
        {
            RCLCPP_WARN(this->get_logger(), "接收到空的图像消息");
            return;
        }

        try
        {
            // 转换压缩图像数据为OpenCV矩阵
            cv::Mat cv_image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
            
            if (cv_image.empty())
            {
                RCLCPP_ERROR(this->get_logger(), "无法解码图像数据");
                return;
            }

            // 使用消息自带的时间戳而非系统时间，提高同步性
            const double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
            
            // 传递图像给SLAM系统
            slam_system_.TrackMonocular(cv_image, timestamp);

            // 周期性打印图像信息（避免日志刷屏）
            static size_t counter = 0;
            if (++counter % 30 == 0)  // 每30帧打印一次
            {
                RCLCPP_INFO(this->get_logger(), "处理图像 - 尺寸: %dx%d, 时间戳: %.6f",
                           cv_image.cols, cv_image.rows, timestamp);
            }
        }
        catch (const cv::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "OpenCV错误: %s", e.what());
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "处理图像时发生错误: %s", e.what());
        }
        catch (...)
        {
            RCLCPP_ERROR(this->get_logger(), "处理图像时发生未知错误");
        }
    }

    rclcpp::Subscription<CompressedImageMsg>::SharedPtr subscription_;  ///< 图像订阅者
    ORB_SLAM::System slam_system_;  ///< ORB-SLAM2系统实例
};

int main(int argc, char * argv[])
{
    argc = 3;
    argv[1] = "/root/workspace/thirdparties/Vocabulary/ORBvoc.txt";
    argv[2] = "/root/workspace/thirdparties/ORB_SLAM2/configs/Monocular/FishEye.yaml";
    // 检查命令行参数
    if (argc != 3)
    {
        std::cerr << "用法: " << argv[0] 
                  << " <ORB字典路径> <配置文件路径>" << std::endl;
        return 1;
    }

    // 初始化ROS 2
    rclcpp::init(argc, argv);
    
    // 创建并运行节点
    auto node = std::make_shared<SlamImageSubscriber>(argv[1], argv[2]);
    rclcpp::spin(node);
    
    // 关闭ROS 2
    rclcpp::shutdown();
    return 0;
}