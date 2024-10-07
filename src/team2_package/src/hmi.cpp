#include "hmi.hpp"

HMI::HMI() : Node("hmi")
{
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("marker_pub",10);
    turtle_color_sub_ = this->create_subscription<interface::msg::TurtleColor>("turtle_color", 10, std::bind(&HMI::turtle_color_callback, this, std::placeholders::_1));
    turtle_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>("turtle1/pose", 10, std::bind(&HMI::turtle_pose_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000.0/60.0), std::bind(&HMI::timer_callback, this));
    marker = visualization_msgs::msg::Marker();
}

void HMI::turtle_color_callback(const interface::msg::TurtleColor::SharedPtr msg)
{
/**
 * @brief Callback function to update the turtle marker color.
 *
 * This function is called whenever a new TurtleColor message is received.
 * It updates the color of the turtle marker based on the received message.
 *
 * @param msg Shared pointer to the received TurtleColor message.
 */

    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "turtle";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.color.r = msg->r;
    marker.color.g = msg->g;
    marker.color.b = msg->b;
}

void HMI::turtle_pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
{
/**
 * @brief Callback function to update the marker's pose based on the turtle's pose.
 * 
 * This function is called whenever a new pose message is received from the turtle.
 * It updates the position and orientation of the marker to match the turtle's pose.
 * 
 * @param msg A shared pointer to the Pose message received from the turtle.
 */
    marker.pose.position.x = msg->x;
    marker.pose.position.y = msg->y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    marker.color.a = 1;
}

void HMI::timer_callback()
{
    marker_pub_->publish(marker);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HMI>());
    rclcpp::shutdown();
    return 0;
}
