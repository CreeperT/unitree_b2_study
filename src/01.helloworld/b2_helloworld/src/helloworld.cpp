#include "rclcpp/rclcpp.hpp"
#include "unitree_api/msg/request.hpp"

using namespace std;
using namespace rclcpp;

class B2Hello : public Node
{
public:
    B2Hello(string name) : Node(name)
    {
        RCLCPP_INFO(get_logger(), "节点已启动：%s.", name.c_str());
        req_puber = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
        timer_ = this->create_wall_timer(chrono::microseconds(100), bind(&B2Hello::hello, this));
    }

private:
    Publisher<unitree_api::msg::Request>::SharedPtr req_puber;
    TimerBase::SharedPtr timer_;
    void hello()
    {
        unitree_api::msg::Request req; // 创建一个运动请求msg
        req.header.identity.api_id = 1016;
        req_puber->publish(req);
    }

};

int main(int argc, char **argv)
{
    init(argc, argv);
    auto node = make_shared<B2Hello>("B2_Helloworld");
    spin(node);
    shutdown();
    return 0;
}