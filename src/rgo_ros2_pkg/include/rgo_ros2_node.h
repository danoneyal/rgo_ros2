#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <nav_msgs/msg/odometry.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

class RgoRos2Node : public rclcpp::Node
{
    public:
        RgoRos2Node ();
        ~RgoRos2Node();
        void shutdown();

    private:
        //void init();
        //void initPerceptionEngine();
        void subscribe_odometry_message(const nav_msgs::msg::Odometry::SharedPtr message) const ;
        void subscribe_message(const geometry_msgs::msg::Twist::SharedPtr message) const;
        //void initSubscribers();
        //void initPublishers();
        void timer_callback();
   

    private:    
    //publisher smart pointers 
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;    
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr _pose_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _absolute_pose_publisher;
    
    //subscriber smart pointers 
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;    
        
    float i;
    size_t count_;
    rclcpp::TimerBase::SharedPtr timer_;            
                  
};
