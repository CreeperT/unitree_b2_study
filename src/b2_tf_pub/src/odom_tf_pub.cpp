#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
using std::placeholders::_1;
using std::placeholders::_2;
using std::string;

class OdomTfPub : public rclcpp::Node
{
private:
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr dog_imu_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

public:
    explicit OdomTfPub(const string &name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "节点 %s: 已启动.", name.c_str());

        // 参数
        bool use_sim_time = this->get_parameter("use_sim_time").as_bool();
        RCLCPP_INFO(this->get_logger(), "是否使用模拟时间: %s", use_sim_time ? "是" : "否");
        
        rclcpp::Parameter use_sim_time_param("use_sim_time", use_sim_time);
        this->set_parameter(use_sim_time_param);

        // 初始化tf广播器
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // 初始化静态tf广播器
        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        publish_static_tf();

        // 订阅/dog_odom
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/dog_odom", 10, std::bind(&OdomTfPub::odom_callback, this, _1));
        RCLCPP_INFO(this->get_logger(), "已发布TF: odom -> base_link");

        // 订阅/dog_imu_raw
        dog_imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/dog_imu_raw", 10, std::bind(&OdomTfPub::imu_callback, this, _1));

        // 发布/imu
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
    }

private:
    void publish_static_tf()
    {
        geometry_msgs::msg::TransformStamped static_transform;

        static_transform.header.stamp = this->now();
        static_transform.header.frame_id = "base_link";
        static_transform.child_frame_id = "rslidar";

        static_transform.transform.translation.x = 0.34218;
        static_transform.transform.translation.y = 0;
        static_transform.transform.translation.z = 0.17851;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, 0.0);
        static_transform.transform.rotation.x = q.x();
        static_transform.transform.rotation.y = q.y();
        static_transform.transform.rotation.z = q.z();
        static_transform.transform.rotation.w = q.w();

        static_tf_broadcaster_->sendTransform(static_transform);
        RCLCPP_INFO(this->get_logger(), "已发布静态TF: base_link -> rslidar");
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // 添加发布频率限制（最大100Hz）
        static rclcpp::Time last_tf_time(0, 0, get_clock()->get_clock_type());
        double time_since_last_tf = (this->now() - last_tf_time).seconds();
        if (time_since_last_tf < 0.01) { // 10ms间隔，100Hz
            return;
        }
        last_tf_time = this->now();
            
        geometry_msgs::msg::TransformStamped transform;

        transform.header.stamp = this->now();
        transform.header.frame_id = "odom";
        transform.child_frame_id = "base_link";

        transform.transform.translation.x = msg->pose.pose.position.x;
        transform.transform.translation.y = msg->pose.pose.position.y;
        transform.transform.translation.z = msg->pose.pose.position.z;

        transform.transform.rotation = msg->pose.pose.orientation;

        tf_broadcaster_->sendTransform(transform);
    }

    void imu_callback(sensor_msgs::msg::Imu::SharedPtr msg)
    {
        msg->header.stamp = this->now();
        imu_pub_->publish(*msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomTfPub>("odom_tf_pub");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}