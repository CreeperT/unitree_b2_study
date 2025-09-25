/*
 * 需求：订阅Twist消息，并将之转换为并b2所需的Request消息以控制机器狗运动
 * 实现：
 *      1.创建一个Request发布对象
 *      2.创建一个Twist订阅对象
 *      3.在回调函数中实现消息的转换以及发布
 */

#include "rclcpp/rclcpp.hpp"
#include "unitree_api/msg/request.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "b2_twist_bridge/sport_model.hpp"
#include "nlohmann/json.hpp"

using namespace std;
using namespace rclcpp;

class TwistBridge : public Node
{
private:
    Publisher<unitree_api::msg::Request>::SharedPtr request_pub;
    Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub;
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr twist)
    {
        unitree_api::msg::Request request;
        // 获取Twist消息的线速度和角速度
        double x = twist->linear.x;
        double y = twist->linear.y;
        double z = twist->angular.z;
        // 默认api_id为平衡站立
        auto api_id = ROBOT_SPORT_API_ID_BALANCESTAND;
        if (x != 0 || y != 0 || z != 0)
        {
            api_id = ROBOT_SPORT_API_ID_MOVE;
            // 设置参数，组织一个字符串样式的速度指令
            nlohmann::json js;
            js["x"] = x;
            js["y"] = y;
            js["z"] = z;
            request.parameter = js.dump();
        }
        request.header.identity.api_id = api_id;
        request_pub->publish(request);
    }

public:
    explicit TwistBridge(const string &name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "节点 %s: 已启动, 可以将geometry_msgs/msg/Twist消息转换为unitree_api/msg/Request消息.", name.c_str());
        request_pub = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
        twist_sub = this->create_subscription<geometry_msgs::msg::Twist>
            ("/cmd_vel", 10, bind(&TwistBridge::twist_callback, this, placeholders::_1));
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TwistBridge>("twist_bridge");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}