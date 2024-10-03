#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>

#include "interface/msg/turtle_color.hpp"

class ColorChanger : public rclcpp::Node
{
public:
    ColorChanger();
private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<interface::msg::TurtleColor>::SharedPtr publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
    std::vector<int> colors_;
    std::vector<rclcpp::Parameter> parameters;
    std::shared_ptr<rclcpp::AsyncParametersClient> parameters_client_;

    void timer_callback();
    void callback(turtlesim::msg::Pose::SharedPtr msg);
};
