#include <memory>
#include <string>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"

class TurtleController : public rclcpp::Node
{
public:
  TurtleController()
  : Node("turtle_controller")
  {
    RCLCPP_INFO(this->get_logger(), "Turtle Controller Node Started");
    
    // 상태 정보 구독
    turtle1_state_sub_ = this->create_subscription<turtlesim::msg::Pose>(
      "/turtle1/state", 10,
      std::bind(&TurtleController::target_pose_callback, this, std::placeholders::_1));
      
    turtle2_state_sub_ = this->create_subscription<turtlesim::msg::Pose>(
      "/turtle2/state", 10,
      std::bind(&TurtleController::current_pose_callback, this, std::placeholders::_1));
      
    // 제어 명령 발행
    control_command_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/turtle2/control_command", 10);
      
    // 초기 위치 설정
    target_x_ = 5.0;
    target_y_ = 5.0;
  }

private:
  void target_pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
  {
    target_x_ = msg->x;
    target_y_ = msg->y;
    RCLCPP_DEBUG(this->get_logger(), 
      "Updated target position: x=%.2f, y=%.2f", msg->x, msg->y);
  }

  void current_pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
  {
    // 위치 오차 계산
    double err_x = target_x_ - msg->x;
    double err_y = target_y_ - msg->y;
    double err_dist = std::sqrt(err_x * err_x + err_y * err_y);
    
    // 목표 방향 계산
    double desired_theta = std::atan2(err_y, err_x);
    double err_theta = desired_theta - msg->theta;
    
    // 각도 정규화 (-π ~ π)
    while (err_theta > M_PI) {
      err_theta -= 2.0 * M_PI;
    }
    while (err_theta < -M_PI) {
      err_theta += 2.0 * M_PI;
    }
    
    // 제어 게인
    const double Kp_dist = 0.9;
    const double Kp_theta = 2.0;
    
    // 제어 명령 계산
    geometry_msgs::msg::Twist cmd;
    if (err_dist < 0.1) {  // 더 작은 임계값으로 수정
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
    } else {
      cmd.linear.x = Kp_dist * err_dist;
      cmd.angular.z = Kp_theta * err_theta;
    }
    
    // 제어 명령 발행
    control_command_pub_->publish(cmd);
    RCLCPP_DEBUG(this->get_logger(), 
      "Published control command: linear=%.2f, angular=%.2f", 
      cmd.linear.x, cmd.angular.z);
  }

  // ROS2 subscribers
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle1_state_sub_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle2_state_sub_;
  
  // ROS2 publisher
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr control_command_pub_;
  
  // 목표 위치
  double target_x_;
  double target_y_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleController>());
  rclcpp::shutdown();
  return 0;
} 