#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <turtlesim/msg/pose.hpp>

#include "interface/msg/sphere.hpp"
#include "interface/msg/turtle_state.hpp"
#include "interface/msg/turtle_color.hpp"

class HMI : public rclcpp::Node
{
public:
    HMI();
private:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Subscription<interface::msg::TurtleColor>::SharedPtr turtle_color_sub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle_pose_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    visualization_msgs::msg::Marker marker;
public:
    void turtle_color_callback(const interface::msg::TurtleColor::SharedPtr msg);
    void turtle_pose_callback(const turtlesim::msg::Pose::SharedPtr msg);
    void timer_callback();
};