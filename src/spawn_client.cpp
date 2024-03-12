#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    if (argc != 4)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: spawn X Y theta");
        return 1;
    }
    auto node = rclcpp::Node::make_shared("spawn");
    auto client = node->create_client<turtlesim::srv::Spawn>("spawn");
    auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
    request->x = atof(argv[1]);
    request->y = atof(argv[2]);
    request->theta = atof(argv[3]);
    while (!client->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "client interrupted while waiting for service to appear.");
            return 1;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "waiting for service to appear...");
    }
    auto result = client->async_send_request(request);
    rclcpp::spin_until_future_complete(node, result);
    rclcpp::shutdown();
    return 0;
}