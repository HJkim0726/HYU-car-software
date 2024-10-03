#include "color_changer.hpp"
#include <cmath>

ColorChanger::ColorChanger() : Node("color_changer_node")
{
    publisher_ = this->create_publisher<interface::msg::TurtleColor>("turtle_color", 10);
    subscription_ = this->create_subscription<turtlesim::msg::Pose>(
        "turtle1/pose", 10, std::bind(&ColorChanger::callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ColorChanger::timer_callback, this));
    colors_ = {0, 0, 0};
    parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "/turtlesim");
}

void ColorChanger::callback(turtlesim::msg::Pose::SharedPtr msg)
{
    float turtle_theta_deg = msg->theta * 180 / M_PI;
    
    if (turtle_theta_deg > 0.0 && turtle_theta_deg <= 90.0)
    {
        colors_[0] = 255;
        colors_[1] = 255;
        colors_[2] = 255;
    } else if (turtle_theta_deg > 90.0 && turtle_theta_deg <= 180.0)
    {
        colors_[0] = 255;
        colors_[1] = 0;
        colors_[2] = 0;
    } else if (turtle_theta_deg >= -180.0 && turtle_theta_deg <= -90.0)
    {
        colors_[0] = 0;
        colors_[1] = 255;
        colors_[2] = 0;
    } else if (turtle_theta_deg > -90.0 && turtle_theta_deg <= 0.0)
    {
        colors_[0] = 0;
        colors_[1] = 0;
        colors_[2] = 255;
    }
}

void ColorChanger::timer_callback()
{   

    parameters.clear();
    parameters.push_back(rclcpp::Parameter("background_r", static_cast<int>(colors_[0])));
    parameters.push_back(rclcpp::Parameter("background_g", static_cast<int>(colors_[1])));
    parameters.push_back(rclcpp::Parameter("background_b", static_cast<int>(colors_[2])));
    parameters_client_->set_parameters(parameters);

    auto msg = interface::msg::TurtleColor();
    msg.r = colors_[0] / 255.0;
    msg.g = colors_[1] / 255.0;
    msg.b = colors_[2] / 255.0;    
    publisher_->publish(msg);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ColorChanger>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}