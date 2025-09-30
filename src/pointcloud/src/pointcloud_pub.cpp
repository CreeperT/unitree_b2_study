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

using namespace std;
using namespace rclcpp;
using namespace std::placeholders;

class PointCloudPub : public Node
{
private:
    Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_raw_;
    Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_filtered_;
    TimerBase::SharedPtr timer_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_;

    void timer_callback()
    {
        sensor_msgs::msg::PointCloud2 raw_msg;
        pcl::toROSMsg(*raw_cloud_, raw_msg);
        raw_msg.header.stamp = this->now();
        raw_msg.header.frame_id = "map";
        cloud_pub_raw_->publish(raw_msg);
        
        sensor_msgs::msg::PointCloud2 filtered_msg;
        pcl::toROSMsg(*filtered_cloud_, filtered_msg);
        filtered_msg.header.stamp = this->now();
        filtered_msg.header.frame_id = "map";
        cloud_pub_filtered_->publish(filtered_msg);
    }

public:
    explicit PointCloudPub(const string &name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "节点 %s: 已启动.", name.c_str());

        // 参数
        this->declare_parameter<string>("raw_cloud_pcd_file", "./");
        string raw_file = this->get_parameter("raw_cloud_pcd_file").as_string();
        RCLCPP_INFO(this->get_logger(), "原始pcd文件: %s", raw_file.c_str());

        this->declare_parameter<string>("filtered_cloud_pcd_file", "./");
        string filtered_file = this->get_parameter("filtered_cloud_pcd_file").as_string();
        RCLCPP_INFO(this->get_logger(), "滤波后pcd文件: %s", filtered_file.c_str());

        this->declare_parameter<int>("interval_time", 5);
        int interval_time = this->get_parameter("interval_time").as_int();
        RCLCPP_INFO(this->get_logger(), "发布间隔时间: %d", interval_time);

        // 读取pcd文件
        if(raw_file.empty() || filtered_file.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "pcd文件为空");
            return;
        }
        raw_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>);
        filtered_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>);
        if(pcl::io::loadPCDFile<pcl::PointXYZI>(raw_file, *raw_cloud_) == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "原始pcd文件读取错误: %s", raw_file.c_str());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "原始点云点数: %ld", raw_cloud_->size());
        if(pcl::io::loadPCDFile<pcl::PointXYZI>(filtered_file, *filtered_cloud_) == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "滤波后pcd文件读取错误: %s", filtered_file.c_str());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "滤波后点云点数: %ld", filtered_cloud_->size());

        // 发布者
        cloud_pub_raw_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/raw_pointcloud", 10);
        cloud_pub_filtered_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_pointcloud", 10);
        
        // 定时器
        timer_ = this->create_wall_timer(
            chrono::seconds(interval_time), bind(&PointCloudPub::timer_callback, this));

        
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudPub>("pointcloud_pub");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}