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

class RobotSimRos2Node : public rclcpp::Node
{
  public:
    RobotSimRos2Node ()
    : Node("robot_sim_ros2_node"), count_(0)
    {
      i = 0.0;
      subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>("rgo_pos", 1, std::bind(&RobotSimRos2Node::subscribe_message, this, _1));

      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("robot_sim_velocity", 1);

      timer_ = this->create_wall_timer(500ms, std::bind(&RobotSimRos2Node::timer_callback, this));
    }

  private:
    float i;
    rclcpp::TimerBase::SharedPtr timer_;    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscriber_;
    size_t count_;

    void subscribe_message(geometry_msgs::msg::Pose::SharedPtr msg)      
    {
        auto rgo_pose = msg->position;
        float x= rgo_pose.x;
        float y= rgo_pose.y;
        float z= rgo_pose.z;
        
        RCLCPP_INFO(this->get_logger(), "Recieved - Position x: '%f', y: '%f', z: '%f' ", x,y,z );
    }
    
    /*the timer call back to public the message*/
    void timer_callback()
    {
      auto message = geometry_msgs::msg::Twist();
      message.linear.x = 4.0; 
      message.angular.z = 2.0 + i;
      RCLCPP_INFO(this->get_logger(), "Sending - Linear Velocity : '%f', Angular Velocity : '%f'", message.linear.x, message.angular.z);
      publisher_->publish(message);
      i += 0.1; 
    }
    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotSimRos2Node>());
  rclcpp::shutdown();
  return 0;
}