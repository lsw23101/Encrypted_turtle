#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"

class TurtlePlant : public rclcpp::Node
{
public:
  TurtlePlant()
  : Node("turtle_plant")
  {
    RCLCPP_INFO(this->get_logger(), "Turtle Plant Node Started");
    
    // Subscribers
    turtle1_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
      "/turtle1/pose", 10,
      std::bind(&TurtlePlant::turtle1_pose_callback, this, std::placeholders::_1));
      
    turtle2_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
      "/turtle2/pose", 10,
      std::bind(&TurtlePlant::turtle2_pose_callback, this, std::placeholders::_1));
      
    // Publishers
    turtle1_state_pub_ = this->create_publisher<turtlesim::msg::Pose>(
      "/turtle1/state", 10);
      
    turtle2_state_pub_ = this->create_publisher<turtlesim::msg::Pose>(
      "/turtle2/state", 10);
      
    turtle2_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/turtle2/cmd_vel", 10);
      
    // Control command subscriber
    control_command_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/turtle2/control_command", 10,
      std::bind(&TurtlePlant::control_command_callback, this, std::placeholders::_1));
  }

private:
  void turtle1_pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
  {
    // 상태 정보 전송
    turtle1_state_pub_->publish(*msg);
    RCLCPP_DEBUG(this->get_logger(), 
      "Published turtle1 state: x=%.2f, y=%.2f", msg->x, msg->y);
  }

  void turtle2_pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
  {
    turtle2_state_pub_->publish(*msg);
    RCLCPP_DEBUG(this->get_logger(), 
      "Published turtle2 state: x=%.2f, y=%.2f", msg->x, msg->y);
  }

  void control_command_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    turtle2_cmd_pub_->publish(*msg);
  }

  // ROS2 subscribers
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle1_pose_sub_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle2_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr control_command_sub_;
  
  // ROS2 publishers
  rclcpp::Publisher<turtlesim::msg::Pose>::SharedPtr turtle1_state_pub_;
  rclcpp::Publisher<turtlesim::msg::Pose>::SharedPtr turtle2_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr turtle2_cmd_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtlePlant>());
  rclcpp::shutdown();
  return 0;
}
