#include <rclcpp>

class MovingTurtle : public rclcpp::Node
{
    public:
        MovingTurtle();
    private:
        void timer_callback();
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
        geometry_msgs::msg::Twist twist_;
        int count_;
}
