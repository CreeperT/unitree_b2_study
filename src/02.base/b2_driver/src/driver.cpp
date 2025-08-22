/*  需求：
 *  1.发布里程计消息
 *  2.广播里程计相关坐标变换
 *  3.发布关节状态消息
 */

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std;
using namespace rclcpp;
using namespace std::placeholders;

class Driver : public Node
{
private:  
    Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_; // 发布里程计
    Subscription<unitree_go::msg::SportModeState>::SharedPtr state_sub_; // 订阅b2状态
    unique_ptr<tf2_ros::TransformBroadcaster> tf_bro_; // 广播坐标变换
    string odom_frame, base_frame;
    bool publish_tf;
    
    void state_callback(const unitree_go::msg::SportModeState::SharedPtr state)
    {
        // 转换并发布里程计
        nav_msgs::msg::Odometry odom;

        // 组织数据
        /// header
        odom.header.stamp.sec = state->stamp.sec;
        odom.header.stamp.nanosec = state->stamp.nanosec;
        odom.header.frame_id = odom_frame; // 里程计坐标系
        odom.child_frame_id = base_frame; // 机器人基坐标系

        /// position & orientation
        odom.pose.pose.position.x = state->position[0];
        odom.pose.pose.position.y = state->position[1];
        odom.pose.pose.position.z = state->position[2];
        odom.pose.pose.orientation.w = state->imu_state.quaternion[0];
        odom.pose.pose.orientation.x = state->imu_state.quaternion[1];
        odom.pose.pose.orientation.y = state->imu_state.quaternion[2];
        odom.pose.pose.orientation.z = state->imu_state.quaternion[3];

        // vel
        odom.twist.twist.linear.x = state->velocity[0];
        odom.twist.twist.linear.y = state->velocity[1];
        odom.twist.twist.linear.z = state->velocity[2];
        odom.twist.twist.angular.z = state->yaw_speed;

        odom_pub_->publish(odom);

        // tf
        if (!publish_tf) return;
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = odom_frame;
        transform.child_frame_id = base_frame;
        /// 设置偏移量和旋转角度
        transform.transform.translation.x = odom.pose.pose.position.x;
        transform.transform.translation.y = odom.pose.pose.position.z;
        transform.transform.translation.z = odom.pose.pose.position.z;
        transform.transform.rotation = odom.pose.pose.orientation;
        tf_bro_->sendTransform(transform);
    }


public:
    explicit Driver(const string &name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "节点 %s: 已启动.", name.c_str());
        // 声明参数
        this->declare_parameter("odom_frame", "odom");
        this->declare_parameter("base_frame", "base_link");
        this->declare_parameter("publish_tf", true);
        // 获取参数
        odom_frame = this->get_parameter("odom_frame").as_string();
        base_frame = this->get_parameter("base_frame").as_string();
        publish_tf = this->get_parameter("publish_tf").as_bool();
        
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        state_sub_ = this->create_subscription<unitree_go::msg::SportModeState>(
            "/lf/sportmodestate", 10, bind(&Driver::state_callback, this, _1));
        
        // 坐标变换广播器
        tf_bro_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Driver>("driver");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}