#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl-1.15/pcl/point_cloud.h"
#include "pcl-1.15/pcl/point_types.h"
#include "pcl-1.15/pcl/io/pcd_io.h"
#include "pcl-1.15/pcl/filters/passthrough.h"

#include <chrono>
#include <iomanip>
#include <sstream>

using std::placeholders::_1;
using std::placeholders::_2;
using std::string;

class PointCloudFilter : public rclcpp::Node
{
private:
    int intensity_threshold_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr input_points_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_points_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

public:
    explicit PointCloudFilter(const string &name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "节点 %s: 已启动.", name.c_str());

        // 参数
        this->declare_parameter<int>("intensity_threshold", 254);
        intensity_threshold_ = this->get_parameter("intensity_threshold").as_int();
        RCLCPP_INFO(this->get_logger(), "强度阈值: %d", intensity_threshold_);

        this->declare_parameter<string>("sub_topic", "/rslidar_points");
        string sub_topic = this->get_parameter("sub_topic").as_string();
        RCLCPP_INFO(this->get_logger(), "订阅话题: %s", sub_topic.c_str());

        this->declare_parameter<string>("pub_topic", "/filtered_points");
        string pub_topic = this->get_parameter("pub_topic").as_string();
        RCLCPP_INFO(this->get_logger(), "订阅话题: %s", pub_topic.c_str());

        bool use_sim_time = this->get_parameter("use_sim_time").as_bool();
        RCLCPP_INFO(this->get_logger(), "是否使用模拟时间: %s", use_sim_time ? "是" : "否");
        
        rclcpp::Parameter use_sim_time_param("use_sim_time", use_sim_time);
        this->set_parameter(use_sim_time_param);
        // 订阅原始点云数据
        input_points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            sub_topic, 10, std::bind(&PointCloudFilter::input_points_callback, this, _1));
        
        // 发布滤波后点云数据
        filtered_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(pub_topic, 10);
        
        // 定时器
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), bind(&PointCloudFilter::timer_callback, this));

    }

    void input_points_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *cloud);
        input_cloud_ = cloud;
    }

    void timer_callback()
    {
        // 创建新点云存储过滤后的点
        filtered_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PassThrough<pcl::PointXYZI> pass;
        pass.setInputCloud(input_cloud_);
        pass.setFilterFieldName("intensity");
        pass.setFilterLimits(intensity_threshold_, std::numeric_limits<double>::max());
        pass.filter(*filtered_cloud_);
        if (filtered_cloud_->empty())
        {
            RCLCPP_WARN(this->get_logger(), "滤波后点云点数: %ld", filtered_cloud_->size());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "滤波后点云点数: %ld", filtered_cloud_->size());

        // 发布过滤后点云
        sensor_msgs::msg::PointCloud2 filtered_msg;
        pcl::toROSMsg(*filtered_cloud_, filtered_msg);
        filtered_msg.header.stamp = this->now();
        filtered_msg.header.frame_id = "rslidar";
        filtered_points_pub_->publish(filtered_msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudFilter>("pointcloud_filter");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}