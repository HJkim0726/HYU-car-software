#include "moving_turtle.hpp"

MovingTurtle::MovingTurtle()
{
    // 선언만 한 것들 초기화
}

void MovingTurtle::timer_callback()
{
    // msg를 인스턴스화 하고 내용 채우기
    auto msg = geometry_msgs::msg::Twist();
    // msg 내용 채우기
    // ex msg.linear.x = 0.5; ... 
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MovingTurtle>());
    rclcpp::shutdown();
    return 0;
}