#include"rgo_ros2_node.h"


RgoRos2Node::RgoRos2Node (): Node("rgo_ros2_node"), count_(0)
{                  
  //create subscribers 
  subscriber_         = this->create_subscription<geometry_msgs::msg::Twist>("robot_sim_velocity", 1, std::bind(&RgoRos2Node::subscribe_message, this, _1));
  odometry_subscriber_   = this->create_subscription<nav_msgs::msg::Odometry>("robot_sim_odometry", 1, std::bind(&RgoRos2Node::subscribe_odometry_message, this, _1));
  
  //create publishers
  publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("rgo_pos",1);
  _pose_publisher = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/rgo_pose_with_cov", 1);
  _absolute_pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("/absolute_position", 1);
  
  //create a timer and a callback function
  timer_ = this->create_wall_timer(500ms, std::bind(&RgoRos2Node::timer_callback, this));      
  i = 0.0;

  RCLCPP_INFO(this->get_logger(), "2- RgoRos2Node run from refactor\n");
}

void RgoRos2Node::shutdown()
{
  
}
RgoRos2Node::~RgoRos2Node()
{
  
} 

void RgoRos2Node::subscribe_odometry_message(const nav_msgs::msg::Odometry::SharedPtr message) const
{                
    float x= message->twist.twist.linear.x;
    float z=message->twist.twist.angular.z;
    RCLCPP_INFO(this->get_logger(), "Receive Odometry Linear Velocity : '%f', Angular Velocity : '%f'", x, z);
}

void RgoRos2Node::subscribe_message(const geometry_msgs::msg::Twist::SharedPtr message) const
{
    RCLCPP_INFO(this->get_logger(), "Receive Twist Linear Velocity : '%f', Angular Velocity : '%f'", message->linear.x, message->angular.z);
}
  
void RgoRos2Node::timer_callback()
{
  auto message = geometry_msgs::msg::Pose();            
  message.position.x = 4.0 + i; 
  message.position.y = 3.0 + i*2;
  message.position.z = 2.0 + i*3;
  RCLCPP_INFO(this->get_logger(), "Sending - position x: '%f', y: '%f', z: '%f' ", message.position.x, message.position.y,message.position.z);
  publisher_->publish(message);
  i += 0.1; 
  
  //geometry_msgs::msg::PoseWithCovarianceStamped pose{};
  auto pose = geometry_msgs::msg::PoseWithCovarianceStamped();      
  pose.pose.pose.position.x = 4.0 + i; 
  pose.pose.pose.position.y = 3.0 + i*2;
  pose.pose.pose.position.z = 2.0 + i*3;
  pose.pose.pose.orientation.x = 0.0 + i*1;
  pose.pose.pose.orientation.y = 1.0 + i*1;
  pose.pose.pose.orientation.z = 2.0 + i*1;
  pose.pose.pose.orientation.w = 3.0 + i*1;
  RCLCPP_INFO(this->get_logger(), "Sending - position x: '%f', y: '%f', z: '%f' ", pose.pose.pose.position.x, pose.pose.pose.orientation.x,pose.pose.pose.position.z);
  
  _pose_publisher->publish(pose);

}