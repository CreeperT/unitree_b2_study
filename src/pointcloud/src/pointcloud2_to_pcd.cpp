#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl-1.15/pcl/point_cloud.h"
#include "pcl-1.15/pcl/point_types.h"
#include "pcl-1.15/pcl/io/pcd_io.h"

#include <chrono>
#include <iomanip>
#include <sstream>
#include <mutex>

using namespace std;
using namespace rclcpp;
using namespace std::placeholders;

class PointCloud22Pcd : public Node
{
private:
    Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr rslidar_sub_;
    TimerBase::SharedPtr timer_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr latest_cloud_;
    mutable mutex cloud_mutex_;
    string output_dir_;

    // 订阅者回调函数
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // 将PointCloud2点云消息转换为PCL点云格式
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *cloud);
        lock_guard<mutex> lock(cloud_mutex_); // 互斥锁
        latest_cloud_ = cloud; // 更新最新的点云数据  
    }

    // 定时器回调函数
    void timer_callback()
    {
        lock_guard<mutex> lock(cloud_mutex_); // 互斥锁
        if(latest_cloud_ && !latest_cloud_->empty())
        {
            // 生成带时间戳的文件名，避免覆盖
            auto now = chrono::system_clock::now();
            auto now_c = chrono::system_clock::to_time_t(now);
            stringstream ss;
            ss << put_time(localtime(&now_c), "point_cloud_%Y.%m.%d_%H:%M:%S");
            string file_name = ss.str() + ".pcd";
            string full_path = output_dir_ + file_name;

            // 尝试保存点云文件
            try
            {
                if(pcl::io::savePCDFileASCII(full_path, *latest_cloud_) == 0)
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
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "最新的点云数据没有接收到或为空");
        }
    }

public:
    explicit PointCloud22Pcd(const string &name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "节点 %s: 已启动.", name.c_str());
        // 参数
        this->declare_parameter<string>("pointcloud_topic", "/rslidar_points");
        string topic_name = this->get_parameter("pointcloud_topic").as_string();
        RCLCPP_INFO(this->get_logger(), "订阅话题: %s", topic_name.c_str());

        this->declare_parameter<int>("save_interval", 10);
        int save_interval = this->get_parameter("save_interval").as_int();
        RCLCPP_INFO(this->get_logger(), "保存间隔: %d", save_interval);

        this->declare_parameter<string>("output_directory", "/home/creepert/unitree_B2_creeper_ws/src/02.base/pointcloud/pcd/");
        output_dir_ = this->get_parameter("output_directory").as_string();
        RCLCPP_INFO(this->get_logger(), "保存目录: %s", output_dir_.c_str());  
        
        // 创建订阅者
        rslidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            topic_name, 10, bind(&PointCloud22Pcd::cloud_callback, this, _1));

        // 创建定时器    
        timer_ = this->create_wall_timer(
            chrono::seconds(save_interval), bind(&PointCloud22Pcd::timer_callback, this));
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloud22Pcd>("pointcloud2_to_pcd");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}