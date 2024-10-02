#include "color_changer.hpp"

ColorChanger::ColorChanger()
{
    publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&ColorChanger::timer_callback, this));
    count_ = 0;
}

void ColorChanger::timer_callback()
{
    auto msg = visualization_msgs::msg::Marker();
    msg.header.frame_id = "map";
    msg.header.stamp = this->now();
    msg.ns = "basic_shapes";
    msg.id = 0;
    msg.type = visualization_msgs::msg::Marker::CUBE;
    msg.action = visualization_msgs::msg::Marker::ADD;
    msg.pose.position.x = 0;
    msg.pose.position.y = 0;
    msg.pose.position.z = 0;
    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.0;
    msg.pose.orientation.w = 1.0;
    msg.scale.x = 1.0;
    msg.scale.y = 1.0;
    msg.scale.z = 1.0;
    msg.color.r = 0.0;
    msg.color.g = 0.0;
    msg.color.b = 1.0;
    msg.color.a = 1.0;
    msg.lifetime = rclcpp::Duration(1, 0);
    publisher_->publish(msg);
    count_++;
}