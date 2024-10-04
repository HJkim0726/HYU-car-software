#include <rclcpp>
#include <geometry_msgs/msg/twist>

class MovingTurtle : public rclcpp::Node
{
    public:
    MovingTurtle(); // 생성자

    private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_; // 생성자 객체
    geometry_msgs::msg::Twist twist_msg_; // 메세지 객체
    rclcpp::TimerBase::SharedPtr timer_;

    void linearVel(double linearX); // 직선 운동
    void angularVel(double angularZ); // 화전 운동 
    void repeatMovement(); // 각각 100번씩 반복해야하니까

    int loopCnt = 0;
};

