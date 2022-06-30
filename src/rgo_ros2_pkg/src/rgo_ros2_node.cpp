#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;


class RgoRos2Node : public rclcpp::Node
{
  public:
    RgoRos2Node ()
    : Node("rgo_ros2_node"), count_(0)
    {
      
      i = 0.0;
      subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("robot_sim_velocity", 1, std::bind(&RgoRos2Node::subscribe_message, this, _1));

      publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("rgo_pos",1);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&RgoRos2Node::timer_callback, this));
    }

  private:
    float i;
    void subscribe_message(const geometry_msgs::msg::Twist::SharedPtr message) const
    {
        RCLCPP_INFO(this->get_logger(), "Recieved - Linear Velocity : '%f', Angular Velocity : '%f'", message->linear.x, message->angular.z);
    }
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;

    void timer_callback()
    {
      auto message = geometry_msgs::msg::Pose();      
      message.position.x = 4.0 + i; 
      message.position.y = 3.0 + i*2;
      message.position.z = 2.0 + i*3;
      RCLCPP_INFO(this->get_logger(), "Sending - position x: '%f', y: '%f', z: '%f' ", message.position.x, message.position.y,message.position.z);
      publisher_->publish(message);
      i += 0.1; 
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RgoRos2Node>());
  rclcpp::shutdown();
  return 0;
}