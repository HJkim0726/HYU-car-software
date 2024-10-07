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
    /**
 * @brief Callback function to change the color based on the turtle's orientation.
 *
 * This function is triggered when a new Pose message is received. It converts the turtle's
 * orientation from radians to degrees and changes the color based on the orientation angle.
 *
 * @param msg Shared pointer to the Pose message containing the turtle's position and orientation.
 *
 * The color is set as follows:
 * - 0 to 90 degrees: White (255, 255, 255)
 * - 90 to 180 degrees: Red (255, 0, 0)
 * - -180 to -90 degrees: Green (0, 255, 0)
 * - -90 to 0 degrees: Blue (0, 0, 255)
 */
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

/**
 * @brief Callback function that updates the background color parameters and publishes the new color.
 * 
 * This function is called periodically by a timer. It performs the following actions:
 * - Clears the current parameters.
 * - Sets new background color parameters (red, green, blue) based on the `colors_` array.
 * - Sends the updated parameters to the parameter client.
 * - Creates a `TurtleColor` message with the new color values normalized to the range [0, 1].
 * - Publishes the `TurtleColor` message to the associated topic.
 */

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