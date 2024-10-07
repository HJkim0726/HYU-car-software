#include "moving_turtle.hpp"
#include <chrono>

using namespace std::chrono_literals;

//생성자
MovingTurtle::MovingTurtle() : Node("moving_turtle"), loopCnt(0)
{
    //퍼블리셔 생성 
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel" , 10);
    
    //타이머 설정
    timer_ = this->create_wall_timer(10ms , std::bind(&MovingTurtle::repeatMovement , this));
}


void MovingTurtle::linearVel(double linearX)
{
    twist_msg_.linear.x = linearX;
    twist_msg_.linear.y = 0.0;
    twist_msg_.linear.z = 0.0;
}

void MovingTurtle::angularVel(double angularZ)
{
    twist_msg_.angular.x = 0.0;
    twist_msg_.angular.y = 0.0;
    twist_msg_.angular.z = angularZ;
}

void MovingTurtle::repeatMovement()
{
    if(loopCnt < 100)
    {
        linearVel(2.0);
        angularVel(0.0);
    }
    else if(loopCnt < 200)
    {
        linearVel(0.0);
        angularVel(1.0);
    }
    else 
    {
        loopCnt = 0; // 다시 초기화 해서 무한 loop를 돌아버리기
    }

    publisher_ ->publish(twist_msg_);

    loopCnt++;
}

int main(int argc , char *argv[])
{
    rclcpp::init(argc , argv);
    auto node = std::make_shared<MovingTurtle>();
    rclcpp::spin(node);
    return 0;
}