#include <rclcpp>

class ColorChanger : public rclcpp::Node
{
public:
    ColorChanger();
private:
    void timer_callback();
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
    visualization_msgs::msg::Marker marker_;
    int count_;
};
