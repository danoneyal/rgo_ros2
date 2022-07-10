#include"rgo_ros2_node.h"
//#include "SpdLogger.h"


RgoRos2Node::RgoRos2Node (): 
        Node("rgo_ros2_node"), 
        _pe(std::make_shared<Pe::PerceptionEngine>())
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

  RCLCPP_INFO(this->get_logger(), "5- RgoRos2Node run from refactor\n");
  InitPerceptionEngine();
  StartPerceptionEngine();
}


void RgoRos2Node::InitPerceptionEngine()
{
  Pe::Perception::Config config;  
	config.logging_config.log_level = "info";  
	config.logging_config.log_to_console = true;
  config.logging_config.log_file_path = "./rgo_ros2_node.log";
  
  //INFO_LOG("rgo ros2 client", "Init pe")
  RCLCPP_INFO(this->get_logger(), "rgo ros2 client, Init pe");
	_pe->Init(config);

  //register to receive global position 
  _pe->GetLocalizationService()->RegisterForGlobalPoseVelocityUpdates(std::bind(&RgoRos2Node::LocalizationCallback, this, std::placeholders::_1));
}

void RgoRos2Node::StartPerceptionEngine()
{
    //INFO_LOG("rgo ros2 client", "Start: starting pe")
    RCLCPP_INFO(this->get_logger(), "rgo ros2 client, Start: starting pe");
    _pe->Start();
}

void RgoRos2Node::StopPerceptionEngine()
{
    _pe->Stop();    
}

void RgoRos2Node::ClosePerceptionEngine()
{
  _pe->Shutdown();
}

void RgoRos2Node::LocalizationCallback(const Pe::PeService::PoseVelocityFrame &frame)
{  
  geometry_msgs::msg::PoseWithCovarianceStamped pose{};

  RCLCPP_INFO(this->get_logger(), "LocalizationCallback , Sending - position from slam ");
  pose.header.frame_id ="PoseVelocityFrame";

  pose.header.stamp.nanosec =frame.header.ts_arrival_sec * 1e9;
  
  pose.pose.pose.position.x = frame.current_pose_velocity.position.x;
  pose.pose.pose.position.y =frame.current_pose_velocity.position.y; 
  pose.pose.pose.position.z = frame.current_pose_velocity.position.z;
  
  pose.pose.pose.orientation.x = frame.current_pose_velocity.angles.quaternion.x;
  pose.pose.pose.orientation.y =frame.current_pose_velocity.angles.quaternion.y;
  pose.pose.pose.orientation.z =frame.current_pose_velocity.angles.quaternion.z;
  pose.pose.pose.orientation.w = frame.current_pose_velocity.angles.quaternion.w;

  _pose_publisher->publish(pose);

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
    float z= message->twist.twist.angular.z;
    RCLCPP_INFO(this->get_logger(), "Receive Odometry Linear Velocity : '%f', Angular Velocity : '%f'", x, z);
    
    //call to convert ros2 Odometry to RGO_ODOMETRY_REPORT function 
    Pe::RobotProxy::SensorFrame::WheelEncoder encoders = ConvertToWheelEncoder(message);
    //inject WheelEncoder to perception engine     
    _pe->GetRobotProxy()->InjectWheelOdometry(encoders);

}

Pe::RobotProxy::SensorFrame::WheelEncoder RgoRos2Node::ConvertToWheelEncoder(const nav_msgs::msg::Odometry::SharedPtr message) const
{
  float x= message->twist.twist.linear.x;
  float z= message->twist.twist.angular.z;

  RCLCPP_INFO(this->get_logger(), "Convert Odometry to WheelEncoder : '%f', Angular Velocity : '%f'", x, z);
  Pe::RobotProxy::SensorFrame::WheelEncoder wheel_odometry{};
  /// Left encoder hardware timestamp.
  wheel_odometry.ts_left= 0.01;
  /// Left encoder ticks.
  wheel_odometry.enc_left= 0.02;
  /// Right encoder hardware timestamp.
  wheel_odometry.ts_right= 0.03;
  /// Right encoder ticks.
  wheel_odometry.enc_right= 0.04;

  return  wheel_odometry;
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
  //RCLCPP_INFO(this->get_logger(), "Sending - position x: '%f', y: '%f', z: '%f' ", message.position.x, message.position.y,message.position.z);
  //publisher_->publish(message);
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
  //RCLCPP_INFO(this->get_logger(), "Sending - position x: '%f', y: '%f', z: '%f' ", pose.pose.pose.position.x, pose.pose.pose.orientation.x,pose.pose.pose.position.z);  
  //_pose_publisher->publish(pose);

}