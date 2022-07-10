
#include <signal.h>
#include"rgo_ros2_node.h"


int main(int argc, char * argv[])
{
rclcpp::init(argc, argv);
rclcpp::spin(std::make_shared<RgoRos2Node>());
rclcpp::shutdown();
return 0;
}