#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <string>

class node: public rclcpp::Node
{
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world!";
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }
public:
    node(std::string st): Node(st)
    {
        RCLCPP_INFO(this->get_logger(), "Hello, world!");
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&node::timer_callback, this));
    }
    ~node()
    {
        RCLCPP_INFO(this->get_logger(), "Goodbye, world!");
    }
};

class ROS_EVENT_LOOP
{
public:
    ROS_EVENT_LOOP(int argc, char *argv[],std::string st)
    {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<node>(st));
    }
    ~ROS_EVENT_LOOP()
    {
        rclcpp::shutdown();
    }
};

int main(int argc, char *argv[])
{
    ROS_EVENT_LOOP(argc, argv,"my_node");
    return 0;
}