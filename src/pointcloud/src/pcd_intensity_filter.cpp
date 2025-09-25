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

class PcdIntensityFilter : public Node
{
private:
    double intensity_threshold_;
    Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_raw_;
    TimerBase::SharedPtr timer_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_;

    void timer_callback()
    {
        sensor_msgs::msg::PointCloud2 output_msg;
        filtered_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::toROSMsg(*filtered_cloud_, output_msg);
        output_msg.header.stamp = this->now();
        output_msg.header.frame_id = "map";
        cloud_pub_->publish(output_msg);

        sensor_msgs::msg::PointCloud2 raw_msg;
        input_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::toROSMsg(*input_cloud_, raw_msg);
        output_msg.header.stamp = this->now();
        output_msg.header.frame_id = "map";
        cloud_pub_raw_->publish(raw_msg);
    }

public:
    explicit PcdIntensityFilter(const string &name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "节点 %s: 已启动.", name.c_str());
        
        // 参数
        this->declare_parameter<string>("input_pcd_file", "");
        string input_file = this->get_parameter("input_pcd_file").as_string();
        RCLCPP_INFO(this->get_logger(), "读取文件: %s", input_file.c_str());

        this->declare_parameter<string>("output_directory", "");
        string output_dir = this->get_parameter("output_directory").as_string();
        RCLCPP_INFO(this->get_logger(), "滤波后pcd文件保存目录: %s", output_dir.c_str());

        this->declare_parameter<double>("intensity_threshold", 50.0f);
        intensity_threshold_ = this->get_parameter("intensity_threshold").as_double();
        RCLCPP_INFO(this->get_logger(), "强度阈值: %f", intensity_threshold_);

        this->declare_parameter<bool>("if_publish", false);
        bool if_publish = this->get_parameter("if_publish").as_bool();
        RCLCPP_INFO(this->get_logger(), "是否发布话题: %s", if_publish ? "是" : "否");

        // 读取PCD文件
        if(input_file.empty() || output_dir.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "输入文件路径或输出文件夹无效");
            return;
        }

        input_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>);
        if(pcl::io::loadPCDFile<pcl::PointXYZI>(input_file, *input_cloud_) == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "无法读取输入pcd文件: %s", input_file.c_str());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "原始点云点数: %ld", input_cloud_->size());

        // 创建新点云存储过滤后的点
        filtered_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PassThrough<pcl::PointXYZI> pass;
        pass.setInputCloud(input_cloud_);
        pass.setFilterFieldName("intensity");
        pass.setFilterLimits(intensity_threshold_, numeric_limits<double>::max());
        pass.filter(*filtered_cloud_);
        RCLCPP_INFO(this->get_logger(), "滤波后点云点数: %ld", filtered_cloud_->size());

        // 保存过滤后的点云到新的pcd文件，生成带时间戳的文件名，避免覆盖
        auto now = chrono::system_clock::now();
        auto now_c = chrono::system_clock::to_time_t(now);
        stringstream ss;
        ss << put_time(localtime(&now_c), "filtered_point_cloud_%Y.%m.%d_%H:%M:%S");
        string file_name = ss.str() + ".pcd";
        string full_path = output_dir + file_name;
        try
        {
            if(pcl::io::savePCDFileASCII(full_path, *filtered_cloud_) == 0)
            {
                RCLCPP_INFO(this->get_logger(), "成功保存pcd文件: '%s' ", file_name.c_str());
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "保存pcd文件失败: '%s' ", file_name.c_str());
            }
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "保存时发生错误 '%s': %s", file_name.c_str(), e.what());
        }

        // 如果有需要，发布
        if(if_publish)
        {
            cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_pointcloud", 10);
            cloud_pub_raw_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/raw_pointcloud", 10);
            timer_ = this->create_wall_timer(
                chrono::seconds(1), bind(&PcdIntensityFilter::timer_callback));
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PcdIntensityFilter>("pcd_intensity_filter");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}