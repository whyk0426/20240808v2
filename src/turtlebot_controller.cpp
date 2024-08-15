#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <turtlesim/msg/pose.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

const double PI = 3.14159265358979323846;

struct State{
    double x = 0;
    double y = 0;
    double theta = 0;
};

class TurtlebotController : public rclcpp::Node{
public:
    TurtlebotController() : Node("turtlebot_controller"){
    
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel",10);
    
    subscription_ = this->create_subscription<turtlesim::msg::Pose>(
        "turtle1/pose", 10, std::bind(&TurtlebotController::topic_callback, this, _1));
    
    timer_ = this->create_wall_timer(
        10ms, std::bind(&TurtlebotController::timer_callback, this));
}

private:

    int a = 0;
    double dt = 0.01;

    double real_x = 5.544445;
    double real_y = 5.544445;
    double real_th = 0;
    double prev_error_d = 2;
    double prev_error_th = 0;


    void timer_callback(){
        geometry_msgs::msg::Twist cmd;

        double goal_x, goal_y, goal_th;
        
        if (a == 0){
            goal_x = 7.54445;
            goal_y = 5.54445;
            goal_th = 0;
        }else if (a == 1){
            goal_x = 7.54445;
            goal_y = 5.54445;
            goal_th = 0.5 * PI;
        }else if (a == 2){
            goal_x = 7.54445;
            goal_y = 7.54445;
            goal_th = 0.5 * PI;
        }else if (a == 3){
            goal_x = 7.54445;
            goal_y = 7.54445;
            goal_th = 1.0 * PI;
        }else if (a == 4){
            goal_x = 5.54445;
            goal_y = 7.54445;
            goal_th = 1.0 * PI;
        }else if (a == 5){
            goal_x = 5.54445;
            goal_y = 7.54445;
            goal_th = 1.5 * PI;
            if (real_th < 0)
                goal_th = -0.5 * PI;
        }else if (a == 6){
            goal_x = 5.54445;
            goal_y = 5.54445;
            goal_th = -0.5 * PI;
        }else if (a == 7){
            goal_x = 5.54445;
            goal_y = 5.54445;
            goal_th = 0;
        }

        
        
        double d_x = goal_x - real_x;
        double d_y = goal_y - real_y;

        //p error
        double error_d = sqrt(d_x * d_x + d_y * d_y);
        double error_th = goal_th - real_th;

        //d error
        double d_error_d =  (error_d - prev_error_d) / dt;
        double d_error_th =  (error_th - prev_error_th) / dt;

        //i error
        double i_error_d = (i_error_d + prev_error_d) * dt;
        double i_error_th = (i_error_th + prev_error_th) * dt;
    

        cmd.linear.x = 2.4 * error_d + 0.07 * d_error_d + 0.001 * i_error_d;
        cmd.angular.z = 2.4 * error_th + 0.07 * d_error_th + 0.001 * i_error_th;


        prev_error_d = error_d;
        prev_error_th = error_th;

        real_x = turtlebot_state.x;
        real_y = turtlebot_state.y;
        real_th = turtlebot_state.theta;

        RCLCPP_INFO(this->get_logger(), "[%d, %f, %f]", a, goal_th, real_th);
        
        if ((a%2 == 0 && error_d < 0.01) or (a%2 == 1 && error_th * error_th < 0.00001))
            a++;
            a = a % 8;
            i_error_d = 0;
            i_error_th = 0;

        publisher_->publish(cmd);
    } 

    void topic_callback(const turtlesim::msg::Pose &msg){
        turtlebot_state.x = msg.x;
        turtlebot_state.y = msg.y;
        turtlebot_state.theta = msg.theta;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
    State turtlebot_state;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtlebotController>());
  rclcpp::shutdown();
  return 0;
}
